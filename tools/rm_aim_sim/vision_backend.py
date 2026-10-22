"""把我们自己那套视觉（HNU_NHS_Vision / deploy_dev 的 Python 同构）接到 rm_aim_sim。

链路完全是我们自己的实现，和真机那套同源：

    仿真原始帧 → HebeiDetector(Yolo26, OpenVINO)
               → solve_pnp（灯条四点 + 相机内参 → 云台系 → 世界系）
               → AlphaBetaTracker（世界坐标滤波）
               → calculate_aim（弹道迭代 + 提前量）
               → FireGate（重投影 / 收敛 / 角度窗）
               → 云台绝对角

和真机版本的差别只有"帧从哪来、指令往哪写"：真机是 Talos 共享内存 + 串口，
这里是 rm_aim_sim 的共享邮箱。中间的检测/PnP/跟踪/弹道/闸门一行没改写。

坐标约定（容易踩）：
  * 我们的 `calculate_aim` 输出 yaw 为世界系 atan2(y, x)，与仿真一致；
  * 它算出的 pitch 是**向上为正**（z 轴抬高为正），而 rm_aim_sim 的指令 pitch
    是**向下为正**，所以下发前要取负；
  * `odom.position` 用仿真的云台原点（0,0,0.65），因为仿真没有底盘，
    枪口就在云台原点；用 (0,0,0) 会让弹道高度差 0.65 m，pitch 直接偏 6°。
"""

import math
import json
import os
import sys
from collections import deque
from pathlib import Path

import numpy as np

for _candidate in (Path(__file__).resolve().parent, Path(__file__).resolve().parent.parent):
    sys.path.insert(0, str(_candidate))

import deploy_dev_talos_bridge as vision  # noqa: E402


class _NhsEkf:
    """HNU_NHS_Vision `solve/predictor.cpp` 的 7 状态等速 EKF（去掉 yaw）。

    NHS 的 planner 只用 position/velocity（yaw 不参与瞄准），所以这里保留
    [x,y,z,vx,vy,vz] 六维，过程/观测噪声沿用其默认值 q_pos=0.01、q_vel=0.1、
    r_pos=0.05。相比"窗口内几帧的二次拟合"，EKF 用整条轨迹的最优加权，
    换向时靠过程噪声 Q 自动放开、匀速时收紧，正是低帧率下缺的那一环。
    """

    def __init__(self, q_pos=0.01, q_vel=0.1, r_pos=0.05):
        self.q_pos = float(q_pos)
        self.q_vel = float(q_vel)
        self.r_pos = float(r_pos)
        self.x = None
        self.P = None
        self.t = 0.0

    def reset(self):
        self.x = None
        self.P = None

    def step(self, timestamp_s, position):
        """预测到 timestamp_s 再用 position 更新（对应 NHS 的 update(dt)）。"""
        z = np.asarray(position, dtype=np.float64)
        if self.x is None:
            self.x = np.concatenate([z, np.zeros(3)])
            self.P = np.eye(6) * 0.5
            self.P[3:, 3:] *= 2.0
            self.t = float(timestamp_s)
            return
        dt = float(timestamp_s) - self.t
        if dt <= 0.0:
            return
        self._predict(dt)
        H = np.zeros((3, 6))
        H[:3, :3] = np.eye(3)
        R = np.eye(3) * self.r_pos
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        self.P = (np.eye(6) - K @ H) @ self.P
        self.t = float(timestamp_s)

    def _predict(self, dt):
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        Q = np.diag([self.q_pos * dt] * 3 + [self.q_vel * dt] * 3)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def position_at(self, timestamp_s):
        if self.x is None:
            return None
        horizon = float(timestamp_s) - self.t
        return self.x[:3] + self.x[3:] * max(horizon, 0.0)


class MotionEstimator:
    """对 PnP 出的世界坐标做二次拟合，拿到"现在"的速度和加速度。

    为什么不直接用管线里的 AlphaBetaTracker：它自己的注释就写了是 fallback
    （"deploy_dev's full EKF remains in its C++ target"），固定增益 0.7/0.12
    在 10~20 Hz 的视觉节拍下换向会滞后半秒，实测让 yaw 的 90 分位偏差到 7°。
    这里改成和真值路径一致的窗口最小二乘（y = c0 + c1·u + c2·u²）。

    板切换（自转靶正面板每 ~0.8 s 换一次，或选板在共视板间抖动）会让位置
    跳 ~0.4 m。旧做法是清空历史，之后 3~4 帧速度估计是垃圾——16 Hz 下就是
    200 ms 的垃圾期，每 0.8 s 来一次，aim_rate 噪声 std 从 0.34 炸到 1.5。
    现在不清历史：把旧样本按刚体旋转换算到新板的轨迹上，
        P_j(t) = P_i(t) + R(ω·(t−t_s))·jump
    （jump = R(θ(t_s))·(p_j−p_i) 是世界系跳变量）。刚体上这是精确等式，
    速度/加速度估计跨切换连续。ω 用切换间隔估：相邻板 body 相差 90°、
    对角板 180°，而切换间隔可以直接测。
    """

    def __init__(self, window=0.12, max_len=32):
        self.window = window
        self.history = []
        self.omega = 0.0                   # 目标自转角速度估计（rad/s，带符号）
        self._last_switch_t = None
        self._switches = 0
        self.ekf = _NhsEkf()

    def observe(self, timestamp_s, position):
        position = np.asarray(position, dtype=np.float64)
        if self.history:
            last_time, last_position = self.history[-1]
            jump = position - last_position
            if timestamp_s - last_time > 0.0 and float(np.linalg.norm(jump)) > 0.35:
                self._switch(timestamp_s, jump)
        self.history.append((float(timestamp_s), position))
        while len(self.history) > 32:
            self.history.pop(0)
        if os.environ.get("AIM_MODEL") == "ekf":
            self.ekf.step(timestamp_s, position)

    # 兼容旧调用名
    add = observe

    def _switch(self, now, jump):
        """板切换：不清历史，把旧样本旋转到新板轨迹上。"""
        self._switches += 1
        self.ekf.reset()
        step = math.pi if float(np.linalg.norm(jump)) > 0.44 else math.pi / 2.0
        if self._last_switch_t is not None:
            interval = now - self._last_switch_t
            if interval > 0.05:
                # Δatan2(jump) = ω·Δt + 90°·sign(ω)（相邻板序列的 body 差每级 +90°）
                # → ω = (Δ − 90°·sign(Δ)) / Δt
                previous = getattr(self, "_last_jump_angle", None)
                if previous is not None:
                    delta = math.remainder(math.atan2(jump[1], jump[0]) - previous,
                                           2.0 * math.pi)
                    if abs(delta) > math.radians(20.0):
                        sense = 1.0 if delta > 0.0 else -1.0
                        rate = (delta - sense * step) / interval
                        if 0.05 < abs(rate) < 8.0:
                            self.omega = 0.6 * self.omega + 0.4 * rate if self.omega else rate
        self._last_switch_t = now
        self._last_jump_angle = math.atan2(jump[1], jump[0])
        if os.environ.get("AIM_DEBUG"):
            print(f"[switch] t={now:.3f} |j|={float(np.linalg.norm(jump)):.3f} "
                  f"omega={self.omega:+.2f} n={self._switches} hist={len(self.history)}",
                  file=sys.stderr, flush=True)
        # rebase：旧样本 + R(ω·(t−now))·jump（z 不随自转变，直接加）
        jx, jy, jz = float(jump[0]), float(jump[1]), float(jump[2])
        for t, pos in self.history:
            angle = self.omega * (t - now)
            c, s = math.cos(angle), math.sin(angle)
            pos[0] += c * jx - s * jy
            pos[1] += s * jx + c * jy
            pos[2] += jz
    # -- 刚体圆周模型 -------------------------------------------------------
    def _orbit_model(self, samples):
        """拟合 p(t) = c0 + v·u + ½a·u² + R(ω·u)·(A, B)（z 轴独立二次）。

        自转靶的装甲板不是直线运动，而是"绕车心的圆周 + 车体平移"。二次多项式
        在 0.25 s 窗口上还能近似（转 30°），到 0.6 s（转 68°）就系统性弯偏——
        这正是"窗口一加大反而 0 命中"的原因。给定 ω 时模型对参数线性，所以
        用 ω 一维搜索 + 线性最小二乘；u = t − 最新样本时刻。
        返回 evaluate(t) 或 None（样本太少/跨度不够时交回二次拟合）。
        """
        count = len(samples)
        if count < 6:
            return None
        times = [s[0] for s in samples]
        if times[-1] - times[0] < 0.35:
            return None
        t_ref = times[-1]
        us = np.asarray([t - t_ref for t in times], dtype=np.float64)
        points = np.asarray([s[1] for s in samples], dtype=np.float64)
        basis_z = np.stack([np.ones_like(us), us, 0.5 * us * us], axis=1)
        z_coef, *_ = np.linalg.lstsq(basis_z, points[:, 2], rcond=None)
        rhs = np.concatenate([points[:, 0], points[:, 1]])

        def solve(omega):
            wu = omega * us
            cw, sw = np.cos(wu), np.sin(wu)
            design = np.zeros((2 * count, 6))
            design[:count, 0] = 1.0
            design[:count, 1] = us
            design[:count, 4] = cw
            design[:count, 5] = -sw
            design[count:, 2] = 1.0
            design[count:, 3] = us
            design[count:, 4] = sw
            design[count:, 5] = cw
            coef, *_ = np.linalg.lstsq(design, rhs, rcond=None)
            residual = design @ coef - rhs
            return float(residual @ residual), coef

        # 有上一帧的 ω 就在附近细搜（16 Hz 下每帧只差 ~0.1 rad/s），否则全量搜。
        # AIM_OMEGA 是标定/上限实验：固定成已知值，跳过搜索。
        fixed = os.environ.get("AIM_OMEGA")
        if fixed:
            _, coef = solve(float(fixed))
            best = (0.0, float(fixed), coef)
        else:
            hint = getattr(self, "_omega_hint", None)
            grid = (np.arange(hint - 1.2, hint + 1.21, 0.1) if hint is not None
                    else np.arange(-8.0, 8.0001, 0.25))
            best = None
            for omega in grid:
                rss, coef = solve(float(omega))
                if best is None or rss < best[0]:
                    best = (rss, float(omega), coef)
            if best is None:
                return None
            center = best[1]
            for omega in np.arange(center - 0.25, center + 0.2501, 0.025):
                rss, coef = solve(float(omega))
                if rss < best[0]:
                    best = (rss, float(omega), coef)
        _, omega, coef = best
        self._omega_hint = omega
        c0x, vx, c0y, vy, rot_a, rot_b = coef
        c0z, vz, _az = z_coef
        # 旋转矢量 (A, B) 就是"车心 → 板心"的水平半径，方向即板法向；给开火门预测到达时朝向用
        self.orbit_state = (t_ref, omega, float(rot_a), float(rot_b))

        def evaluate(timestamp_s):
            u = float(timestamp_s) - t_ref
            wu = omega * u
            cw, sw = math.cos(wu), math.sin(wu)
            return np.array([
                c0x + vx * u + rot_a * cw - rot_b * sw,
                c0y + vy * u + rot_a * sw + rot_b * cw,
                c0z + vz * u,
            ])

        return evaluate

    def _model(self):
        """本帧缓存：同一 observe 时刻会被 assign_basis/选板/解算调很多次。

        AIM_MODEL=linear 时退回一次拟合（对照组），其它值才走圆周模型。
        """
        if os.environ.get("AIM_MODEL", "linear") != "orbit":
            return None
        if not self.history:
            return None
        key = (len(self.history), self.history[-1][0])
        if getattr(self, "_model_key", None) == key:
            return self._model_value
        self._model_key = key
        self._model_value = self._orbit_model(self._window())
        return self._model_value

    def estimate(self):
        model = self._model()
        samples = self._window()
        if model is None:
            if len(samples) < 2:
                return None, None
            times = [s[0] for s in samples]
            velocity, acceleration = [], []
            for axis in range(3):
                values = [float(s[1][axis]) for s in samples]
                if os.environ.get("AIM_MODEL") == "linear":
                    span = times[-1] - times[0]
                    slope = (values[-1] - values[0]) / span if span > 1e-6 else 0.0
                    velocity.append(slope)
                    acceleration.append(0.0)
                    continue
                solution = _fit_quadratic(times, values)
                if solution is None:
                    _, v, _a = _two_point(times, values)
                    a = _a
                else:
                    _, v, c2 = solution
                    a = 2.0 * c2
                velocity.append(v)
                acceleration.append(a)
            return np.array(velocity), np.array(acceleration)
        now = self.history[-1][0]
        step = 0.01
        ahead = model(now + step)
        behind = model(now - step)
        middle = model(now)
        velocity = (ahead - behind) / (2.0 * step)
        acceleration = (ahead - 2.0 * middle + behind) / (step * step)
        return velocity, acceleration

    def predict(self, timestamp_s):
        """模型（或退化后的二次拟合）在 timestamp_s 处的位置。"""
        if not self.history:
            return None
        if os.environ.get("AIM_MODEL") == "ekf":
            return self.ekf.position_at(timestamp_s)
        model = self._model()
        if model is not None:
            return model(float(timestamp_s))
        last_time = self.history[-1][0]
        ahead = float(timestamp_s) - last_time
        samples = self._window()
        if len(samples) < 2:
            return np.asarray(self.history[-1][1], dtype=np.float64)
        times = [s[0] for s in samples]
        if os.environ.get("AIM_MODEL") == "linear":
            span = times[-1] - times[0]
            out = []
            for axis in range(3):
                values = [float(s[1][axis]) for s in samples]
                slope = (values[-1] - values[0]) / span if span > 1e-6 else 0.0
                out.append(values[-1] + slope * ahead)
            return np.array(out)
        return np.array([_motion_at_now(times, [float(s[1][axis]) for s in samples],
                                        ahead=ahead)
                         for axis in range(3)])

    def position_now(self):
        """拟合曲线在最新样本处的值（噪声被整个窗口平均过，比单帧稳）。"""
        if not self.history:
            return None
        return self.predict(self.history[-1][0])

    def _window(self):
        if not self.history:
            return []
        newest = self.history[-1][0]
        rows = [row for row in self.history if newest - row[0] <= self.window + 1e-6]
        if len(rows) < 3:
            rows = list(self.history)[-3:]
        return rows


class _AngleEkf:
    """角域等加速卡尔曼：[yaw, yaw_rate, yaw_acc, elev, elev_rate, elev_acc]。

    角度测量只有 ~0.04°（像素重心），而板的公转让视线角速度以 ~14°/s² 变化；
    等速外推 0.3 s 会差 0.6°。加速度在 0.5 s 窗口上可估（σ_a≈1.4°/s²），
    所以状态里带上它。
    """

    def __init__(self, q_angle=0.002, q_rate=0.02, q_acc=0.02, r_angle=0.05):
        self.q_angle = q_angle
        self.q_rate = q_rate
        self.q_acc = q_acc
        self.r_angle = r_angle
        self.x = None
        self.P = None
        self.t = 0.0

    @staticmethod
    def _block(dt):
        F = np.array([[1.0, dt, 0.5 * dt * dt],
                      [0.0, 1.0, dt],
                      [0.0, 0.0, 1.0]])
        return F

    def _propagate(self, dt):
        F = np.zeros((6, 6))
        F[0:3, 0:3] = self._block(dt)
        F[3:6, 3:6] = self._block(dt)
        Q = np.diag([self.q_angle * dt, self.q_rate * dt, self.q_acc * dt,
                     self.q_angle * dt, self.q_rate * dt, self.q_acc * dt])
        return F, Q

    def update(self, timestamp_s, yaw_deg, elev_deg):
        z = np.array([yaw_deg, elev_deg])
        if self.x is None:
            self.x = np.array([yaw_deg, 0.0, 0.0, elev_deg, 0.0, 0.0])
            self.P = np.diag([1.0, 25.0, 4.0, 1.0, 25.0, 4.0])
            self.t = float(timestamp_s)
            return
        dt = float(timestamp_s) - self.t
        if dt <= 1e-4:
            self.t = float(timestamp_s)
            return
        F, Q = self._propagate(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        H = np.zeros((2, 6))
        H[0, 0] = 1.0
        H[1, 3] = 1.0
        R = np.eye(2) * self.r_angle
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        self.P = (np.eye(6) - K @ H) @ self.P
        self.t = float(timestamp_s)

    def predict_at(self, timestamp_s):
        if self.x is None:
            return None
        state = self.x.copy()
        horizon = float(timestamp_s) - self.t
        if horizon > 0.0:
            F, Q = self._propagate(horizon)
            state = F @ state
        return (state[0], state[3], state[1], state[4])


class PlateSlot:
    """一块可见装甲板的时序轨迹。

    自转靶同时有两块板朝向相机（相邻板对）。单槽位方案在"当前板丢一帧"时
    靠 max(confidence) 回退到另一块板，位置跳 0.4 m → 清历史 → 速度估计
    垃圾 → 下一帧又跳回来，形成逐帧抖动（实测 dt=0.06 s 的背靠背跳变，
    aim_rate 噪声 std 0.34→1.5）。双槽位给每块板各留一条连续轨迹：
      * 指派：候选板归到预测最近的槽（连续性半径内），归不上的开新槽
      * 主板：现任主板 0.2 s 内还有观测就留任（aim 不抖），否则提升
        "最近有观测"的槽——从板历史连续，顶上来没有速度瞬态
      * 清理：0.6 s 没有观测的槽丢弃（板转出视野，回来也是新 acquisition）
    """

    def __init__(self, window):
        self.motion = MotionEstimator(window=window, max_len=64)
        self.angles = []               # 角域历史 [(t, yaw, elev, range)]，每槽独立
        self.angle_ekf = _AngleEkf()
        self.last_seen = None
        self.matched = None            # 本帧指派给它的候选（PnP 结果）
        self.last_candidate = None     # 上一次的候选（本帧没检出时借给它用）
        self.misses = 0                # 连续未命中次数（重捕获用）

    def predict(self, now):
        return self.motion.predict(now)

    def age(self, now):
        return float("inf") if self.last_seen is None else now - self.last_seen

    def assign_basis(self, now):
        """指派用的基准位置：**只平滑、不外推**。

        候选是这一帧的 PnP（时刻 now），槽的最后样本是上一帧（now−60 ms）。
        外推那 60 ms 只会把速度噪声放大进基准（实测 W=0.25 的线性外推把基准
        推偏 0.2 m，指派失败→槽冻结→整轮 0 目标）。而板在 60 ms 内只走
        ~0.03 m，远小于 0.3 m 的指派半径——直接用平滑后的最新位置就够。
        """
        if not self.motion.history:
            return None
        smoothed = self.motion.position_now()
        if smoothed is not None:
            return smoothed
        return np.asarray(self.motion.history[-1][1], dtype=np.float64)

    def commit(self, now, candidate):
        self.motion.observe(now, candidate.world_position)
        self.last_seen = now
        self.last_candidate = candidate
        self.misses = 0
        # facing 用单帧投影宽度算，2 px 角点抖动会让它抖 ±0.05 → 误切换主板。
        # 一阶低通（0.7/0.3）把它压稳，代价是换板判据慢一帧。
        raw_facing = float(candidate.detection.get("facing", 1.0))
        previous = getattr(self, "last_facing", None)
        self.last_facing = raw_facing if previous is None else 0.7 * previous + 0.3 * raw_facing
        history = getattr(self, "facing_history", None)
        if history is None:
            history = self.facing_history = []
        history.append((float(now), raw_facing))
        while history and now - history[0][0] > 0.25:
            history.pop(0)

    def facing_at(self, now, horizon):
        """把 facing 线性外推到 now + horizon；样本不够就返回当前值。"""
        history = getattr(self, "facing_history", None) or []
        current = getattr(self, "last_facing", 1.0)
        if len(history) < 4:
            return current
        times = np.asarray([row[0] for row in history], dtype=np.float64)
        values = np.asarray([row[1] for row in history], dtype=np.float64)
        slope = float(np.polyfit(times - times[-1], values, 1)[0])
        return current + slope * (float(now) - float(times[-1]) + horizon)


class CenterTracker:
    """从两块共视板反推**车心**，再按刚体旋转预测板。

    真值路径 94% 的关键不是拟合得多好，而是它跟踪车心：车心只做平滑平移，
    板只是 `center + R(ωt)·(板−车心)`。视觉路径直接跟踪板，板在 0.25 m 半径
    上以 2 rad/s 公转，16 Hz 下窗口一长二次拟合就系统性弯偏。

    两块相邻板（相差 90°）就能定出车心与板对方向：
        mid   = (pA+pB)/2 = c + (r/√2)·u(θA+45°)
        delta = pB−pA     = r√2·u(θA+135°)
      ⇒ c = mid − (|delta|/2)·u(atan2(delta)−90°),  θA = atan2(delta) − 135°
    板对切换时 θ 跳 90°，按 π/2 取模展开即可连续。
    """

    def __init__(self, window=0.7):
        self.window = window
        # (t, center, pair_direction, span, z_mid)
        self.history = []

    def observe(self, when, plates):
        points = [np.asarray(p, dtype=np.float64) for p in plates]
        if len(points) < 2:
            return
        pair = None
        best = None
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                span = float(np.linalg.norm(points[i] - points[j]))
                cost = abs(span - 0.35)      # 相邻板 r√2 ≈ 0.35 m，对角板 ≈ 0.5 m
                if best is None or cost < best:
                    best, pair = cost, (points[i], points[j])
        if pair is None or best is None or best > 0.15:
            return
        a, b = pair
        delta = b - a
        span = float(np.linalg.norm(delta))
        if span < 1e-6:
            return
        direction = math.atan2(delta[1], delta[0])
        # 顺序帧间一致：检测顺序任意，a/b 反了 direction 就跳 180°。
        # 用连续性判据（选与上一帧方向最接近的那个顺序），z 只做首帧初始化——
        # z 噪声 0.036 与高低差 0.1 接近，单靠 z 会随机翻。
        if self.history:
            previous = self.history[-1][2]
            if abs(math.remainder(direction - previous, 2 * math.pi)) > 1.2:
                a, b = b, a
                delta = -delta
                direction = math.atan2(delta[1], delta[0])
        elif b[2] > a[2]:
            a, b = b, a
            delta = -delta
            direction = math.atan2(delta[1], delta[0])
        mid = (a + b) * 0.5
        center = mid - (span * 0.5) * np.asarray(
            [math.sin(direction), -math.cos(direction), 0.0])
        self.history.append((float(when), center, direction, span, float(mid[2]),
                             a.copy(), b.copy()))
        newest = self.history[-1][0]
        self.history = [row for row in self.history if newest - row[0] <= self.window + 1e-6]
        while len(self.history) > 48:
            self.history.pop(0)

    def _run(self):
        """最近一段"板对没换"的连续帧（方向跳 90° 就是换板对了）。"""
        rows = self.history
        if not rows:
            return []
        run = [rows[-1]]
        for i in range(len(rows) - 2, -1, -1):
            delta = math.remainder(rows[i + 1][2] - rows[i][2], 2 * math.pi)
            if abs(delta) > 1.0:
                break
            run.append(rows[i])
        run.reverse()
        return run

    def omega(self):
        """稳定板对段内的方向线性拟合（比逐帧差分稳：单帧方向噪声 ~11°，
        而每帧真实转角只有 ~7°，差分会把噪声放大成几 rad/s）。"""
        run = self._run()
        if len(run) < 3:
            return None
        times = np.asarray([row[0] for row in run])
        if times[-1] - times[0] < 0.12:
            return None
        raw = [row[2] for row in run]
        unwrapped = [raw[0]]
        for value in raw[1:]:
            unwrapped.append(unwrapped[-1] + math.remainder(value - unwrapped[-1],
                                                            2 * math.pi))
        slope, _ = np.polyfit(times - times[-1], np.asarray(unwrapped), 1)
        return float(slope)

    def _center_fit(self, timestamp_s):
        now = self.history[-1][0]
        rows = [row for row in self.history if now - row[0] <= self.window + 1e-6]
        if len(rows) < 3 or rows[-1][0] - rows[0][0] < 0.2:
            return None
        times = np.asarray([row[0] for row in rows])
        us = times - times[-1]
        out = []
        for axis in range(3):
            values = np.asarray([row[1][axis] for row in rows])
            design = np.stack([np.ones_like(us), us, 0.5 * us * us], axis=1)
            coef, *_ = np.linalg.lstsq(design, values, rcond=None)
            u = timestamp_s - times[-1]
            out.append(coef[0] + coef[1] * u + 0.5 * coef[2] * u * u)
        return np.asarray(out)

    def ready(self, when):
        return (len(self.history) >= 4 and when - self.history[-1][0] <= 0.4
                and self.omega() is not None and self._center_fit(when) is not None)

    def predict(self, timestamp_s, when, plate_position):
        """预测"当前这块板"在 timestamp_s 的位置；数据不足返回 None。

        相位取最近一段稳定板对的平均（每帧按 ω 外推到最新），半径取板对几何，
        都**不用单帧 PnP 的绝对位置**——这正是误差最大的地方。
        """
        if not self.ready(when):
            return None
        run = self._run()
        if not run:
            return None
        omega = self.omega()
        t_last, c_last, _d_last, span_last, _z_last, plate_a, plate_b = self.history[-1]
        plate = np.asarray(plate_position, dtype=np.float64)
        # a/b 已按连续性固定；被瞄的板是哪一个由距离判断（不用 z，z 噪声会翻）
        offset = 0.0 if float(np.linalg.norm(plate - plate_a)) <= \
            float(np.linalg.norm(plate - plate_b)) else 0.5 * math.pi
        phases = []
        for t_i, _c_i, direction_i, _span_i, _z_i, _a_i, _b_i in run:
            phase_i = direction_i - 0.75 * math.pi + offset
            phases.append(math.remainder(phase_i + omega * (t_last - t_i), 2 * math.pi))
        baseline = phases[0]
        phases = [baseline + math.remainder(p - baseline, 2 * math.pi) for p in phases]
        phase = float(np.mean(phases))
        center = self._center_fit(timestamp_s)
        if center is None:
            return None
        radius = float(np.median([row[3] for row in run])) / math.sqrt(2.0)
        angle = phase + omega * (timestamp_s - t_last)
        height = float(plate[2] - c_last[2])
        return center + np.asarray([radius * math.cos(angle),
                                    radius * math.sin(angle),
                                    height])


class _PredictedPlate:
    """主板瞬时没检出时的占位：世界位置用拟合外推，检测属性借上一次的候选，
    让解算/门判/跟踪器在没有新检测的帧上也能正常跑，aim 保持连续。"""

    def __init__(self, world_position, template):
        self.world_position = np.asarray(world_position, dtype=np.float64)
        self.reprojection_error_px = template.reprojection_error_px
        self.detection = template.detection


def _wrap(angle):
    """把角度收回到 (-pi, pi]。"""
    return math.remainder(angle, 2.0 * math.pi)


def _unwrap(angles):
    """把可能跨 ±pi 的角度序列还原成连续序列（拟合角速度用）。"""
    if not angles:
        return []
    out = [angles[0]]
    for value in angles[1:]:
        out.append(out[-1] + _wrap(value - out[-1]))
    return out


def _slope(times, values):
    """一元线性最小二乘斜率。"""
    count = len(times)
    if count < 2:
        return 0.0
    mean_t = sum(times) / count
    mean_v = sum(values) / count
    denominator = sum((t - mean_t) ** 2 for t in times)
    if abs(denominator) < 1e-12:
        return 0.0
    return sum((t - mean_t) * (v - mean_v) for t, v in zip(times, values)) / denominator


def _fit_quadratic(times, values):
    """以最后一帧为原点拟合 y = c0 + c1·u + c2·u²（u≤0），返回 (c0, c1, c2)。
    样本 <3 或矩阵病态时返回 None（调用方退化成两点差分）。"""
    count = len(times)
    if count < 3:
        return None
    us = [t - times[-1] for t in times]
    s = [sum(u ** k for u in us) for k in range(5)]
    rhs = [sum((u ** k) * y for u, y in zip(us, values)) for k in range(3)]
    matrix = [[s[0], s[1], s[2]], [s[1], s[2], s[3]], [s[2], s[3], s[4]]]
    return _solve3(matrix, rhs)


def _two_point(times, values):
    """两点差分：(位置, 速度, 0)。"""
    span = times[-1] - times[0]
    rate = (values[-1] - values[0]) / span if span > 1e-6 else 0.0
    return values[-1], rate, 0.0


def _motion_at_now(times, values, ahead=0.0):
    """拟合（或两点差分）在 ahead 处的值；ahead=0 即"现在"的位置。"""
    count = len(times)
    if count < 3:
        return _two_point(times, values)[0] + _two_point(times, values)[1] * ahead
    solution = _fit_quadratic(times, values)
    if solution is None:
        position, rate, _ = _two_point(times, values)
        return position + rate * ahead
    c0, c1, c2 = solution
    u = ahead
    return c0 + c1 * u + c2 * u * u


def _solve3(matrix, rhs):
    rows = [list(row) + [value] for row, value in zip(matrix, rhs)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda r: abs(rows[r][column]))
        if abs(rows[pivot][column]) < 1e-12:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]
        lead = rows[column][column]
        rows[column] = [value / lead for value in rows[column]]
        for r in range(3):
            if r == column:
                continue
            factor = rows[r][column]
            rows[r] = [value - factor * base for value, base in zip(rows[r], rows[column])]
    return [rows[0][3], rows[1][3], rows[2][3]]


def _ballistic(speed, gravity, barrel, delta):
    """和真值路径同一套弹道迭代（含枪管前伸），返回 (yaw, pitch, flight)。"""
    dx, dy, dz = delta
    horizontal = math.hypot(dx, dy)
    elevation = math.atan2(dz, horizontal)
    for _ in range(8):
        distance = horizontal - barrel * math.cos(elevation)
        height = dz - barrel * math.sin(elevation)
        if distance <= 0.0:
            return None
        if gravity < 1e-9:
            elevation = math.atan2(height, distance)
        else:
            disc = speed ** 4 - gravity * (gravity * distance * distance + 2.0 * height * speed * speed)
            if disc < 0.0:
                return None
            elevation = math.atan((speed * speed - math.sqrt(disc)) / (gravity * distance))
    flight = (horizontal - barrel * math.cos(elevation)) / (speed * math.cos(elevation))
    return math.atan2(dy, dx), -elevation, flight


class RoiCropper:
    """按上一帧的目标框裁一块正方形 ROI 再送检测。

    意义不在省算力（网络输入还是 640×640），而在**有效分辨率**：整幅 1440×1080
    缩到 640 时一块板只有 40×24 px，角点回归必然偏；裁成 300×300 再缩到 640，
    板子变成 200×120 px，角点精度和置信度都会明显上去。

    跟丢 max_miss 帧就退回全图搜索。
    """

    def __init__(self, margin=2.2, min_size=224, max_miss=3):
        self.margin = margin
        self.min_size = min_size
        self.max_miss = max_miss
        self.box = None
        self.miss = 0
        self.hits = 0

    def region(self, width, height):
        if self.box is None:
            return None
        x1, y1, x2, y2 = self.box
        cx, cy = (x1 + x2) * 0.5, (y1 + y2) * 0.5
        side = max(x2 - x1, y2 - y1) * self.margin
        side = max(side, self.min_size)
        side = min(side, min(width, height))
        left = int(round(cx - side * 0.5))
        top = int(round(cy - side * 0.5))
        left = max(0, min(left, width - int(side)))
        top = max(0, min(top, height - int(side)))
        return left, top, left + int(side), top + int(side)

    def observe(self, box, detected):
        if not detected:
            self.miss += 1
            if self.miss > self.max_miss:
                self.box = None
            return
        self.miss = 0
        self.hits += 1
        self.box = box


class BodyTracker:
    """靶车刚体模型：估车心 C 与车身角 θ，预测形式和真值路径逐字一致。

    视觉侧原来直接跟踪"某一块装甲板的中心"。但 4 块板在车体系里相距最多 0.5 m
    （见 PLATES，实测 σ=0 的精确常数），而每帧只有 2 块可见、还在 (0,1)/(0,3) 之间
    来回切——板一跳，中心位置就跳 0.5 m，速度估计直接被废掉。
    实测这就是 yaw 出膛角散布 1~4°（装甲半宽才 1°）的来源。

    解法：用两块可见板的**差值**解车身角（差值与车心无关，可精确解），
    再把每块板换算回车心、对车心做二次拟合、对车身角做线性拟合——
    和 tools/rm_aim_sim_bridge.py 里 Tracker 的做法一致，只是输入从真值换成 PnP。
    """

    # 装甲板在车体系里的固定偏移（仿真实测，多帧平均 σ=0）
    PLATES = {0: (0.2481, 0.0, 0.0), 1: (0.0, 0.2481, -0.1),
              2: (-0.2481, 0.0, 0.0), 3: (0.0, -0.2481, -0.1)}
    # 装甲板在车体系里的法向（朝外的那一面）
    PLATE_NORMALS = {0: (1.0, 0.0, 0.0), 1: (0.0, 1.0, 0.0),
                     2: (-1.0, 0.0, 0.0), 3: (0.0, -1.0, 0.0)}
    MAX_SPEED = 8.0
    MAX_ACCEL = 30.0
    MAX_OMEGA = 20.0

    def __init__(self, window=0.10, max_dt=0.4):
        self.window = window
        self.max_dt = max_dt
        self.center_history = deque(maxlen=64)
        self.angle_history = deque(maxlen=64)
        self.theta = None
        self.theta_pending = None            # bootstrap 双帧确认的挂起候选
        self.omega = 0.0
        self.last_time = None
        self.solved_frames = 0
        self.last_decision = ""            # 遥测：本帧 theta 走了哪个分支

    # -- 观测 ---------------------------------------------------------------
    def observe(self, now, plates, origin=None):
        """plates: {armor_id: 世界坐标}。返回本次是否精确解出了车身角。

        theta 的处理是时序的：两板差解 theta 的单帧噪声 = 切向PnP误差/板间距
        ≈ 0.087·√2/0.3838 ≈ 18°（57px 板实测），而 2 rad/s 自转下帧间只变 2~6°——
        逐帧采用等于把噪声当信号。这里 angle_history 只收与当前解一致的样本，
        self.theta 取窗口线性拟合在 now 的外推值。
        """
        ids = [i for i in plates if i in self.PLATES]
        if not ids:
            self.last_decision = "no_ids"
            return False
        solved, theta_new = self._solve_angle(ids, plates)
        if solved and theta_new is not None and self.theta is not None and \
                abs(_wrap(theta_new - self.theta)) > math.radians(90.0):
            # 解出的 theta 与历史表示差 >90°：试把 id 翻转到历史表示
            # （0↔2、1↔3 互换是 θ+π 等价，同一物理状态的两种标注）。
            # 不翻转的话 EMA 会把两种表示混成中间值（30~90° 的系统误差），
            # center 也会因 id 与 theta 表示不一致而错 0.5 m（板偏移量的两倍）。
            flipped = self._flip_ids(plates)
            flip_ids = [i for i in flipped if i in self.PLATES]
            if len(flip_ids) >= 2:
                flip_solved, flip_theta = self._solve_angle(flip_ids, flipped)
                if flip_solved and flip_theta is not None and \
                        abs(_wrap(flip_theta - self.theta)) < math.radians(90.0):
                    plates = flipped
                    theta_new = flip_theta
        if solved and theta_new is not None:
            if self.theta is None:
                # bootstrap：facing 已在 associate 里挡掉相邻错配，
                # 剩下的候选直接采用，靠 60° 门限 + facing 修正通道纠错
                self.theta = theta_new
                self.angle_history.append((now, theta_new))
                self.last_decision = "bootstrap"
            else:
                # 到这里 theta_new 已与历史同表示（π 等价在上面被翻转消掉）。
                # 收样门限 60°：单帧解噪声 p50 22.6°/p90 ~35°，叠上帧间真实
                # 变化（2 rad/s 下 50 ms ≈ 5.7°），和 ~45°；门限收太紧会样本
                # 饥饿 → omega 乱 → 外冲 → 更饿（死循环）。错配 90° 仍被挡住。
                delta = _wrap(theta_new - self.theta)
                if abs(delta) < math.radians(60.0):
                    # 预测-修正而不是裸 EMA：先用 omega 外推到本帧时刻再修正。
                    # 裸 EMA(alpha 0.25) 对 6.6°/帧 的斜坡信号滞后 26°——这正是
                    # 稳态 theta 误差 30° 的来源；omega 前馈把滞后消掉，alpha 只
                    # 决定抗噪度。omega 本身有 9% 误差（1 s 窗口 Theil-Sen），
                    # 折算到外推只有 ~1°，远小于单帧噪声。
                    predicted = self.theta
                    if self.last_time is not None:
                        step = max(-0.6, min(0.6, self.omega * (now - self.last_time)))
                        predicted = _wrap(self.theta + step)
                    innovation = _wrap(theta_new - predicted)
                    self.theta = _wrap(predicted + 0.4 * innovation)
                    self.angle_history.append((now, theta_new))
                    self.last_decision = f"sample d={math.degrees(delta):.0f}"
                elif math.radians(70.0) < abs(delta) < math.radians(130.0) and \
                        origin is not None and \
                        all(self._facing(theta_new, {i: plates[i]}, origin) for i in ids):
                    # 70~130° 的跳变只可能是 bootstrap 错选（相邻错配 90°）：
                    # 新解下所有板都朝向相机才允许修正，中间地带一律拒绝
                    # （40~70° 的跳变没有物理来源，是噪声解，放了会让 theta 抖）
                    self.theta = theta_new
                    self.angle_history.clear()
                    self.angle_history.append((now, theta_new))
                    self.last_decision = f"fix d={math.degrees(delta):.0f}"
                else:
                    self.last_decision = f"reject d={math.degrees(delta):.0f}"
        elif self.theta is not None:
            self.last_decision = "no_solve"
        if self.theta is None:
            return False
        if not solved and self.last_time is not None and self.omega:
            # 没有新样本的帧按角速度外推。外推量限幅 ±0.5 rad（28.6°）：
            # 真实 2 rad/s 下 0.15 s 只转 17°，限幅挡的是 omega 估错时的
            # 外冲——一帧冲 50° 出去，后续 solved 帧的 delta 全超门限被挡，
            # angle_history 饥饿 → omega 更噪 → 冲更远的死循环。
            step = max(-0.5, min(0.5, self.omega * (now - self.last_time)))
            self.theta = _wrap(self.theta + step)
        cos_t, sin_t = math.cos(self.theta), math.sin(self.theta)
        # ids 可能在 π 等价翻转后变化，centers 必须用翻转后的 plates 重算
        ids = [i for i in plates if i in self.PLATES]
        centers = []
        for i in ids:
            px, py, pz = self.PLATES[i]
            rel = (cos_t * px - sin_t * py, sin_t * px + cos_t * py, pz)
            centers.append([plates[i][k] - rel[k] for k in range(3)])
        # 多块板解出来的车心理应重合；取中位对抗单块的 PnP 野值
        center = [float(sorted(c[k] for c in centers)[len(centers) // 2]) for k in range(3)]
        # id 错配会把车心带偏 ~0.35 m（板偏移量），而 0.15 s 的真实位移只有
        # ~0.23 m。与上帧差超过 0.45 m 的样本一律拒收，护住运动拟合。
        if self.center_history:
            last_time, last_center = self.center_history[-1]
            jump = math.sqrt(sum((center[k] - last_center[k]) ** 2 for k in range(3)))
            if jump > 0.45 and now - last_time < 0.3:
                return solved
        self.center_history.append((now, center))
        self.last_time = now
        if solved:
            self.solved_frames += 1
        return solved

    def _angle_at(self, now):
        """angle_history 的窗口线性拟合外推到 now 的车身角。

        样本不够（bootstrap 初期）就退回当前值。拟合用 unwrap 后的序列，
        避免 ±π 回绕把斜率掰断。
        """
        angles = self._window(self.angle_history)
        if len(angles) >= 3:
            t0 = angles[0][0]
            times = [row[0] - t0 for row in angles]
            values = _unwrap([row[1] for row in angles])
            slope = _slope(times, values)
            mean_t = sum(times) / len(times)
            mean_v = sum(values) / len(values)
            intercept = mean_v - slope * mean_t
            return _wrap(intercept + slope * (now - t0))
        return self.theta if self.theta is not None else 0.0


    def _flip_ids(self, plates):
        """0↔2、1↔3 互换 id：θ+π 等价的另一种标注，板的世界位置不变。"""
        return {(i + 2) % 4: position for i, position in plates.items()}

    def _solve_angle(self, ids, plates):
        """两块可见板的差值与车心无关：d = R(θ)·q，q 是车体系里已知的板间差。

        返回 (是否解出, theta)。theta 不直接写 self.theta——由 observe 做
        历史连续性滤波，错配残值进不了历史。
        """
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                i, j = ids[a], ids[b]
                d = [plates[i][k] - plates[j][k] for k in range(3)]
                q = [self.PLATES[i][k] - self.PLATES[j][k] for k in range(3)]
                # 竖直分量不受旋转影响，对不上说明这对观测不可信
                if abs(d[2] - q[2]) > 0.05:
                    continue
                det = q[0] * q[0] + q[1] * q[1]
                if det < 1e-6:
                    continue
                # [qx -qy; qy qx]·[c; s] = [dx; dy]
                c = (q[0] * d[0] + q[1] * d[1]) / det
                s = (q[0] * d[1] - q[1] * d[0]) / det
                norm = math.hypot(c, s)
                if norm < 1e-6:
                    continue
                return True, math.atan2(s / norm, c / norm)
        return False, None

    # -- 检测 → 装甲板 id 的关联 ---------------------------------------------
    def associate(self, positions, now=None, origin=None):
        """把 PnP 出来的世界坐标对应到装甲板 id。

        真检测器不给 id，但 4 块板在车体系里的位置是已知常数。两块板时穷举配对，
        解出 θ 后从两块板分别反推车心——配对了两者必然重合，配错必然对不上。
        （0↔2 互换只等价于 θ+π，是把前后板换个名字，预测位置完全一样，无害。）

        **相邻错配陷阱**（实测 19~57 px 小板上 gap 分不开）：把相邻两块板
        （如真值 id 0/1）错配成 (1,0)，差向量模长不变（0.3838 m），θ 偏 90°，
        车心照样重合到 gap<0.06——旧实现里这种错配占 gap 合格配对的 80%，
        theta 误差 p50 78°。分辨它靠**法向朝向**：正确 θ 下两块板的应有法向
        （R(θ)·n_id）都朝向相机；错配 90° 后必有一块背对。实测相邻错配
        facing 通过率 0/184，正确配对 34/46。
        """
        count = len(positions)
        if count >= 2:
            # 历史先验：车身角在 20~50 ms 内最多变 3°，与历史差 >20° 的配对
            # 时序上不可能（除非历史本身错了，那由 observe 的 facing 修正通道处理）
            prior = None
            if self.theta is not None and self.last_time is not None and now is not None:
                prior = _wrap(self.theta + self.omega * max(0.0, now - self.last_time))
            best = None
            for i in range(count):
                for j in range(count):
                    if i == j:
                        continue
                    for id_i in self.PLATES:
                        for id_j in self.PLATES:
                            if id_i == id_j:
                                continue
                            theta = self._theta_from_pair(positions[i], positions[j], id_i, id_j)
                            if theta is None:
                                continue
                            first = self._center_of(positions[i], id_i, theta)
                            second = self._center_of(positions[j], id_j, theta)
                            if first is None or second is None:
                                continue
                            gap = math.sqrt(sum((first[k] - second[k]) ** 2 for k in range(3)))
                            if gap > 0.15:
                                continue
                            # facing 必须优先于 prior：prior 一致的候选免检 facing
                            # 会让相邻错配（θ±90°）自洽锁死——bootstrap 一错，
                            # 之后每帧的错配都与"错误历史"一致、正确解反而差 90°
                            # 被门限挡在外面，facing 修正通道也救不回来
                            # （observe 拿到的 plates 本身就是错 id，循环依赖）。
                            # 注意 π 等价：wrap(2Δ)≈0 时 θ 与 θ+π 等价，也算一致。
                            facing = origin is None or self._facing(
                                theta, {id_i: positions[i], id_j: positions[j]}, origin)
                            consistent = prior is not None and \
                                abs(_wrap(2.0 * (theta - prior))) < math.radians(40.0)
                            if not facing and not consistent:
                                continue
                            rank = (0 if facing else 1, 0 if consistent else 1, gap)
                            if best is None or rank < best[0]:
                                best = (rank, theta, {id_i: positions[i], id_j: positions[j]})
            if best is not None and best[0][2] < 0.06:
                return best[2], best[1]
        if self.theta is None:
            return None, None
        plates = {}
        for position in positions:
            best_id, best_gap = None, None
            for plate_id in self.PLATES:
                centre = self._center_of(position, plate_id, self.theta)
                if centre is None:
                    continue
                # 单板指派也要 facing：背对我们的板不可能被检出
                if origin is not None and not self._facing(self.theta, {plate_id: position},
                                                           origin):
                    continue
                predicted = self.plate_now(plate_id, centre, self.theta)
                gap = math.sqrt(sum((predicted[k] - position[k]) ** 2 for k in range(3)))
                if best_gap is None or gap < best_gap:
                    best_id, best_gap = plate_id, gap
            if best_id is not None and (best_gap is None or best_gap < 0.35):
                plates[best_id] = position
        return (plates or None), self.theta

    def _facing(self, theta, assignment, origin):
        """assignment: {plate_id: 世界坐标}。解出 theta 后各块板的应有法向
        （R(theta)·n_id）是否都朝向相机。检出的板必然是正面装甲，背对的配对
        无论 gap 多小都是错配。"""
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        for plate_id, position in assignment.items():
            nx, ny, _ = self.PLATE_NORMALS[plate_id]
            normal = (cos_t * nx - sin_t * ny, sin_t * nx + cos_t * ny, 0.0)
            to_me = [origin[k] - position[k] for k in range(3)]
            if sum(to_me[k] * normal[k] for k in range(3)) <= 0.05:
                return False
        return True

    def _theta_from_pair(self, first, second, id_first, id_second):
        d = [first[k] - second[k] for k in range(3)]
        q = [self.PLATES[id_first][k] - self.PLATES[id_second][k] for k in range(3)]
        if abs(d[2] - q[2]) > 0.06:
            return None
        det = q[0] * q[0] + q[1] * q[1]
        if det < 1e-6:
            return None
        c = (q[0] * d[0] + q[1] * d[1]) / det
        s = (q[0] * d[1] - q[1] * d[0]) / det
        norm = math.hypot(c, s)
        if norm < 1e-6:
            return None
        return math.atan2(s / norm, c / norm)

    def _center_of(self, position, plate_id, theta):
        px, py, pz = self.PLATES[plate_id]
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        rel = (cos_t * px - sin_t * py, sin_t * px + cos_t * py, pz)
        return [position[k] - rel[k] for k in range(3)]

    # -- 运动估计 ------------------------------------------------------------
    def motion(self):
        """返回 (速度, 加速度, 角速度)。车心二次拟合 + 车身角线性拟合。

        超门限时**截断**而不是归零：归零会让 _solve_aim_body 直接 no_motion，
        调用方停火/搜索，刚体模型再也没有样本来自愈——一次错配就永久瘫痪
        （实测 81% 的帧死在 no_motion）。截断最多让预测偏一点，下一帧
        样本正常了拟合自己会收敛回去。
        """
        centers = self._window(self.center_history)
        if len(centers) < 3:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), self.omega
        times = [row[0] for row in centers]
        velocity, acceleration = [], []
        for axis in range(3):
            v, a = _motion_at_now(times, [row[1][axis] for row in centers])
            velocity.append(v)
            acceleration.append(a)
        # angle_history 只收 solved 帧（~30%）：0.3 s 窗口只有 4 个样本，
        # Theil-Sen 中位被 30° 单帧噪声淹没（真实 omega 2 rad/s = 115°/s，
        # 4 样本斜率噪声 33°/s）。拉到 1.0 s：~9 个样本，噪声降到 ~10°/s。
        # omega 是缓变量，长窗口只损失 bootstrap 期的跟随速度。
        angles = [row for row in self.angle_history
                  if centers[-1][0] - row[0] <= 1.0 + 1e-6]
        omega = self.omega
        if len(angles) >= 4:
            t0 = angles[0][0]
            times = [row[0] - t0 for row in angles]
            values = _unwrap([row[1] for row in angles])
            # Theil-Sen：所有点对斜率取中位。单帧 theta 噪声 22° 时最小二乘
            # 斜率被噪声按 √N 压不动，中位对单个野值完全免疫。
            # 样本 <4 不更新：bootstrap 初期的两三帧差分是纯噪声（±6 rad/s），
            # 拿它外推会把 theta 冲飞，后续样本全被门限挡掉（死循环）。
            slopes = [(values[b] - values[a]) / (times[b] - times[a])
                      for a in range(len(times)) for b in range(a + 1, len(times))
                      if times[b] - times[a] > 0.02]
            if slopes:
                slopes.sort()
                # 强平滑：omega 是缓变量（目标 2 rad/s 恒定），0.15 的新值权重
                # 把 Theil-Sen 残余噪声再压一个量级，滞后只有 ~0.1 s
                self.omega = 0.15 * slopes[len(slopes) // 2] + 0.85 * self.omega
                return tuple(velocity), tuple(acceleration), self.omega
        velocity = [max(-self.MAX_SPEED, min(self.MAX_SPEED, v)) for v in velocity]
        acceleration = [max(-self.MAX_ACCEL, min(self.MAX_ACCEL, a)) for a in acceleration]
        omega = max(-self.MAX_OMEGA, min(self.MAX_OMEGA, self.omega))
        self.omega = omega
        return tuple(velocity), tuple(acceleration), omega

    def _window(self, history):
        if not history:
            return []
        newest = history[-1][0]
        rows = [row for row in history if newest - row[0] <= self.window + 1e-6]
        if len(rows) < 3:
            rows = list(history)[-3:]
        return rows

    # -- 预测 ----------------------------------------------------------------
    def plate_world(self, plate_id, ahead, center, velocity, acceleration, omega, theta):
        """把某块板预测到 ahead 秒后。

        形式与真值路径逐字一致：predicted = C + v·t + ½a·t² + R(ωt)·rel，
        其中 rel 是这块板**当前**相对车心的世界偏移（= R(θ)·p），不是车体系偏移 p。
        少了 R(θ) 这一项，板位会整整转错一个车身角。
        """
        px, py, pz = self.PLATES[plate_id]
        c0, s0 = math.cos(theta), math.sin(theta)
        rel = (c0 * px - s0 * py, s0 * px + c0 * py, pz)
        ct, st = math.cos(omega * ahead), math.sin(omega * ahead)
        rel_t = (ct * rel[0] - st * rel[1], st * rel[0] + ct * rel[1], rel[2])
        return [center[k] + velocity[k] * ahead + 0.5 * acceleration[k] * ahead * ahead + rel_t[k]
                for k in range(3)]

    def plate_now(self, plate_id, center, theta):
        """这块板当前的世界坐标（用来判正面朝向和取距离）。"""
        px, py, pz = self.PLATES[plate_id]
        c0, s0 = math.cos(theta), math.sin(theta)
        return [center[0] + c0 * px - s0 * py,
                center[1] + s0 * px + c0 * py,
                center[2] + pz]

    def reset(self):
        self.center_history.clear()
        self.angle_history.clear()
        self.theta = None
        self.theta_pending = None
        self.omega = 0.0
        self.last_time = None


class VisionBackend:
    """一帧进，一个云台指令出。失败时返回 None，由调用方决定停火/保持。"""

    def __init__(self, model_path, config=None, confidence=0.25, device="CPU",
                 enemy_color=None, bullet_speed=None, delay_s=None, large_mode="auto",
                 detector="yolo26", min_hits=2, max_dt=0.4, max_reprojection=5.0,
                 alpha=0.70, beta=0.12, roi=True, roi_margin=2.2,
                 corner_offset=0.0, gate=None, use_body=True,
                 motion_window=0.25):
        config = config or {}
        detectors = {"yolo26": vision.HebeiDetector, "rp0526": vision.Rp0526Detector}
        if detector != "ideal" and detector not in detectors:
            raise ValueError(f"未知检测器 {detector}（可选 ideal/{'/'.join(detectors)}）")
        self.detector_name = detector
        # ideal 不加载网络，直接用仿真的理想角点——把"检测器质量"和"我们的 PnP/跟踪/
        # 弹道链路"分开：它跑 100% 说明链路对，剩下的差距全在模型上。
        self.detector = detectors[detector](Path(model_path), confidence, device) \
            if detector in detectors else None
        self.tracker = vision.AlphaBetaTracker(max_dt, min_hits, alpha=alpha, beta=beta)
        # 拟合窗口要覆盖"解算→出膛"的总提前量：30 Hz 下 flight 0.16 s +
        # delay 0.045 s ≈ 0.21 s。窗口 0.12 s 时外推距离是窗口的 1.75 倍，
        # 二次拟合噪声按 (外推/窗口)² 放大 ~3×——实测出弹散布 ~2.8°，这就是
        # 对准了却打不中的原因。0.25 s 窗口（~8 帧）把噪声平均到 0.05 m，
        # 外推比 0.84 不再放大。代价是机动跟随慢一点（accel 3 m/s² 下无碍）。
        # 注意：这是按 30 Hz 视觉算的。实测推理要 62 ms（~16 Hz），0.25 s 窗口
        # 只有 ~4 个样本，二次拟合自由度 1，aim_rate 噪声 std 从 0.34 炸到
        # 1.5——外推比降了但拟合本身先烂了。A/B 见 ab_motion.py。
        self.motion = MotionEstimator(window=float(motion_window), max_len=64)
        self.motion_window = float(motion_window)
        # 车心跟踪（AIM_MODEL=center）：两块共视板定车心，再按刚体旋转预测板
        self.center = CenterTracker(window=max(0.5, float(motion_window)))
        # 双槽位跟踪（见 PlateSlot）：主板/从板各一条连续轨迹，消灭选板抖动
        self.slots = []
        self.primary = None
        # 刚体模型（车心 + 车身角）。理想检测器带装甲板 id，能精确解车身角；
        # 真检测器没有 id，走几何关联——但 19~57 px 小板的 PnP 噪声（0.15 m）
        # 与板间几何尺度（0.38~0.50 m）信噪比不足：两板差解 theta 单帧 err
        # ~30°，id 指配在 facing 擦边盲区系统出错（center 错 0.4 m）。实测
        # 刚体路线 fired=1/25s，单板时序跟踪（PnP 0.15 m 平均到 0.07 m）反而
        # 更准，所以 use_body 默认关，--use-body 才走刚体。
        self.body = BodyTracker(window=max(0.10, max_dt * 0.5), max_dt=max_dt)
        self.use_body = bool(use_body)
        # ROI 只对真检测器有意义；ideal 直接读仿真角点
        self.roi = RoiCropper(margin=roi_margin) if (roi and detector != "ideal") else None
        self.roi_used = False
        self.roi_size = (0, 0)
        # 沿"离心"方向整体内外移动角点。检测器在仿真图上把角点回归得略靠外，
        # 表现为 PnP 距离系统性偏近约 0.18 m（4 m 处）；负值=向内收，能把这个偏置抹掉。
        self.corner_offset = float(corner_offset)
        # None = 不按颜色过滤。仿真贴图和真装甲的外观分布不同，颜色头在仿真里
        # 会给出 0/1/3 混着的输出（实测），所以仿真默认不拿它当门槛。
        self.enemy_color = enemy_color
        self.max_reprojection = max_reprojection
        self.bullet_speed = float(bullet_speed or config.get("bullet_speed", 23.0))
        self.delay_s = float(delay_s if delay_s is not None else config.get("low_speed_delay_time", 0.015))
        self.base_delay = self.delay_s                 # 不含"取帧→下发"那一段，由调用方叠加
        # "class" 用 deploy_dev 的 class>=6；仿真里模型给不出正确 class，
        # 而靶车本来就是小装甲（140 mm），所以仿真默认 "small"。
        self.large_mode = large_mode
        self.gate = vision.FireGate(gate or {}, True)
        self.aim_rate = (0.0, 0.0)             # 模型解析出来的瞄准角速度，供前馈用
        # 角域跟踪（AIM_MODEL=angle）：记录被瞄板的世界方向 (t, yaw, elev, range)。
        # 方向来自像素重心 + 相机射线，不经过 PnP 的横向/深度噪声——
        # 1 px 在 1303 焦距上只有 0.044°，而 PnP 的世界坐标横向噪声是 1.1°/帧。
        self.angle_history = []
        self._angle_slot = None
        self._last_primary = None
        self.settled_at = 0.0         # 主板最近一次切换的时刻（开火稳定期用）
        self.last = {}                                # 给 reason / 遥测用
        self.frames = 0
        self.detections = 0

    # -- 组装 FrameBundle -------------------------------------------------
    def _bundle(self, state, image_bgr, seq, stamp_ns):
        camera = state.get("camera") or {}
        K = camera.get("K") or []
        if len(K) != 9:
            return None
        width = int(camera.get("width") or image_bgr.shape[1])
        height = int(camera.get("height") or image_bgr.shape[0])
        info = vision.CameraInfo(stamp_ns, K[0], K[4], K[2], K[5],
                                 np.zeros(5, dtype=np.float64), width, height)

        gimbal = state.get("gimbal") or {}
        quaternion = gimbal.get("quaternion_wxyz") or [1.0, 0.0, 0.0, 0.0]
        origin = np.asarray(gimbal.get("origin") or [0.0, 0.0, 0.0], dtype=np.float64)
        gimbal_pose = vision.Pose(seq, origin, np.asarray(quaternion, dtype=np.float64), stamp_ns)
        # 仿真没有底盘：世界原点就是云台原点，pnp 与弹道都以它为基准
        odom_pose = vision.Pose(seq, origin.copy(), np.array([1.0, 0.0, 0.0, 0.0]), stamp_ns)
        offset = np.asarray(camera.get("t_camera2gimbal") or [0.0, 0.0, 0.0], dtype=np.float64)
        camera_pose = vision.Pose(seq, offset, np.asarray(quaternion, dtype=np.float64), stamp_ns)
        return vision.FrameBundle(image_bgr, seq, stamp_ns, gimbal_pose, odom_pose,
                                  gimbal_pose, camera_pose, info)

    def _detect(self, image_bgr):
        """有 ROI 就只裁那块再检测，角点按偏移量还原到整幅图坐标。"""
        height, width = image_bgr.shape[:2]
        region = self.roi.region(width, height) if self.roi else None
        if region is None:
            self.roi_used = False
            return self.detector.detect(image_bgr), (0, 0)
        left, top, right, bottom = region
        detections = self.detector.detect(image_bgr[top:bottom, left:right])
        shift = np.array([left, top], dtype=np.float64)
        for detection in detections:
            detection["points"] = detection["points"] + shift
            x1, y1, x2, y2 = detection["box"]
            detection["box"] = (x1 + left, y1 + top, x2 + left, y2 + top)
        self.roi_used = True
        self.roi_size = (right - left, bottom - top)
        return detections, (left, top)

    def _refine_points(self, image_bgr, points):
        """亚像素角点精修：沿灯条外缘找一维梯度峰，取交点。

        rp0526 的角点有 ~2 px 抖动 + (−0.6,−1.3) px 系统偏；实测给理想角点加
        2 px 高斯噪声，命中率从 70.4% 掉到 54.2%（1 px 时 72.1%），所以把角点
        压到亚像素是当前唯一能把 41% 拉回 ~65-70% 的手段。
        """
        corners = np.asarray(points, dtype=np.float64).copy()
        if corners.shape != (4, 2):
            return corners
        gray = image_bgr if image_bgr.ndim == 2 else image_bgr.mean(axis=2)
        gray = gray.astype(np.float32)
        height, width = gray.shape[:2]
        outward = ((-1, -1), (1, -1), (1, 1), (-1, 1))   # TL/TR/BR/BL

        def edge(profile, centre):
            gradient = np.diff(profile)
            if len(gradient) < 3:
                return None
            index = int(np.argmax(np.abs(gradient)))
            if index <= 0 or index >= len(gradient) - 1:
                return None
            d0, d1, d2 = gradient[index - 1], gradient[index], gradient[index + 1]
            denominator = d0 - 2.0 * d1 + d2
            shift = 0.0 if abs(denominator) < 1e-9 else 0.5 * (d0 - d2) / denominator
            return centre + index + 0.5 + max(-0.5, min(0.5, shift))

        half = 6
        for index, (sign_x, sign_y) in enumerate(outward):
            x, y = corners[index]
            xi, yi = int(round(x)), int(round(y))
            # 垂直外缘：只在朝外那半边找，避免抓到灯条内缘
            x0 = xi - half if sign_x < 0 else xi - 1
            x1 = xi + 1 if sign_x < 0 else xi + half
            y0, y1 = max(0, yi - 2), min(height - 1, yi + 2)
            if 0 <= x0 < x1 < width and y1 > y0:
                profile = gray[y0:y1 + 1, x0:x1 + 1].mean(axis=0)
                found = edge(profile, x0)
                if found is not None and abs(found - x) <= half:
                    corners[index, 0] = found
            # 水平外缘
            y0 = yi - half if sign_y < 0 else yi - 1
            y1 = yi + 1 if sign_y < 0 else yi + half
            x0, x1 = max(0, xi - 2), min(width - 1, xi + 2)
            if 0 <= y0 < y1 < height and x1 > x0:
                profile = gray[y0:y1 + 1, x0:x1 + 1].mean(axis=1)
                found = edge(profile, y0)
                if found is not None and abs(found - y) <= half:
                    corners[index, 1] = found
        return corners

    def _ideal_detections(self, state):
        """仿真自带的理想投影角点（诊断用，不是检测）。

        AIM_IDEAL_NOISE=<px> 给角点加高斯噪声：用来区分"角点抖动"和"角点系统偏"
        各自对命中率的影响（rp0526 实测 σ≈2 px）。
        """
        noise = float(os.environ.get("AIM_IDEAL_NOISE", "0") or 0.0)
        detections = []
        for armor in (state.get("camera") or {}).get("armors") or []:
            points = np.asarray(armor["corners"], dtype=np.float64)
            if noise > 0.0:
                points = points + np.random.normal(0.0, noise, points.shape)
            detections.append({
                "confidence": 1.0, "class_id": 0, "color_id": 0, "large": False,
                "armor_id": armor.get("id"),
                "box": (float(points[:, 0].min()), float(points[:, 1].min()),
                        float(points[:, 0].max()), float(points[:, 1].max())),
                "points": points,
            })
        return detections

    def _large(self, class_id):
        if self.large_mode == "large":
            return True
        if self.large_mode == "small":
            return False
        if self.large_mode == "class":
            return int(class_id) >= 6
        return None                                  # auto：用检测器自己的判断

    # -- 主入口 -----------------------------------------------------------
    def step(self, state, image_bgr, timestamp_ns, delay_s=None, observe_time_s=None):
        """返回 (yaw, pitch, fire, info)；无目标时 yaw/pitch 为 None。

        delay_s 是"这一帧拍到、到子弹真正出膛"的额外时延，会折进弹道提前量。
        视觉链路里它通常是 40~80 ms（取帧 + 推理 + 指令下发生效），
        2 rad/s 下 50 ms 就是 5.7°，不补的话命中率会塌。

        observe_time_s 是这一帧内容对应的**仿真时钟**（state["sim_time"]）。
        必须用它而不是墙钟给运动拟合打时间戳：PnP 解出的位置属于渲染那一刻，
        而取帧时延有 ±10 ms 抖动，用墙钟算出来的速度是系统性的错
        （实测真值后端 |yaw| 0.30°，用墙钟的视觉链路 1.09°，差 3.6 倍）。
        """
        delay = self.delay_s if delay_s is None else float(delay_s)
        self.frames += 1
        observe = float(observe_time_s if observe_time_s is not None else state.get("sim_time") or 0.0)
        if observe <= 0.0:
            observe = timestamp_ns / 1e9
        bundle = self._bundle(state, image_bgr, int(state.get("tick", 0)), timestamp_ns)
        if bundle is None:
            self.last = {"reason": "camera_missing"}
            return None, None, False, self.last

        if self.detector is not None:
            detections, offset = self._detect(image_bgr)
            if os.environ.get("AIM_REFINE", "0") == "1":
                for detection in detections:
                    detection["points"] = self._refine_points(image_bgr, detection["points"])
        else:
            detections, offset = self._ideal_detections(state), (0, 0)
        self.detections += len(detections)
        for detection in detections:            # 需要时用统一规则覆盖检测器自己的 large 判断
            forced = self._large(detection["class_id"])
            if forced is not None:
                detection["large"] = forced

        # 不用 vision.select_target：它把 None 当成"必须等于 None"，会把所有候选滤掉。
        # 这里把颜色过滤做成可选，其余（PnP + 重投影门限 + 取最高置信度）保持一致。
        candidates = []
        for detection in detections:
            if self.enemy_color is not None and detection["color_id"] != self.enemy_color:
                continue
            if self.corner_offset:
                points = detection["points"]
                centre = points.mean(axis=0)
                vector = points - centre
                norm = np.linalg.norm(vector, axis=1, keepdims=True)
                detection["points"] = points + vector / np.maximum(norm, 1e-6) * self.corner_offset
            result = vision.solve_pnp(detection, bundle)
            if result is not None and result.reprojection_error_px <= self.max_reprojection:
                # 正对程度 = 板投影宽度 / 正对时应有的宽度。
                # 侧向（edge-on）的板宽度会显著变窄，子弹只会擦过它的面——
                # 实测 64 发里只有 14 发真正穿越装甲平面，交点横向偏 +11~+39 cm。
                # 真值路径用 normal·视线>0.2 只打正面板，视觉侧要用像素宽度把这件事补上。
                points = np.asarray(detection["points"], dtype=np.float64)
                width_px = float(points[:, 0].max() - points[:, 0].min())
                distance = float(result.distance_m)
                expected = bundle.camera_info.matrix[0, 0] * 0.135 / max(distance, 1e-3)
                facing = width_px / max(expected, 1e-3)
                result.detection["facing"] = max(0.0, min(1.0, facing))
                candidates.append(result)
        # 车心跟踪：喂这一帧的全部候选板（它自己挑相邻板对）
        if os.environ.get("AIM_MODEL") == "center" and len(candidates) >= 2:
            self.center.observe(observe, [np.asarray(c.world_position) for c in candidates])
        # 双槽位指派：候选板归到基准最近的槽，归不上的开新槽（见 PlateSlot）。
        # 连续性半径 0.3 m：平滑基准下被跟踪板在 ~0.1 m 内、共视板在 ~0.4 m 外；
        # 原始基准（年轻槽）下板每帧动 ~0.13 m，也在半径内。
        for slot in self.slots:
            slot.matched = None
        for candidate in candidates:
            position = np.asarray(candidate.world_position, dtype=np.float64)
            best, best_distance = None, None
            for slot in self.slots:
                if slot.matched is not None or not slot.motion.history:
                    continue
                basis = slot.assign_basis(observe)
                if basis is None:
                    continue
                distance = float(np.linalg.norm(position - basis))
                if best_distance is None or distance < best_distance:
                    best, best_distance = slot, distance
            if best is not None and best_distance is not None and best_distance < 0.3:
                best.matched = candidate
            elif len(self.slots) < 3:
                slot = PlateSlot(self.motion_window)
                slot.matched = candidate
                self.slots.append(slot)
        # 连续 3 帧没命中的槽：强制重捕获——把最近的未指派候选塞给它。
        # 没有这条，一次边缘指派失败会让槽冻结、预测漂死，整轮自瞄都瘸。
        for slot in self.slots:
            if slot.matched is None and slot.misses >= 3:
                spare = [c for c in candidates
                         if all(s.matched is not c for s in self.slots)]
                if spare:
                    basis = slot.assign_basis(observe)
                    if basis is not None:
                        slot.matched = min(spare, key=lambda c: float(np.linalg.norm(
                            np.asarray(c.world_position, dtype=np.float64) - basis)))
            if slot.matched is None:
                slot.misses += 1
        # 清理：0.6 s 没有观测的槽丢弃
        self.slots = [s for s in self.slots
                      if s.matched is not None or s.age(observe) <= 0.6]
        # 提交观测（内部检测板切换并做刚体 rebase，不清历史）
        for slot in self.slots:
            if slot.matched is not None:
                slot.commit(observe, slot.matched)
                if os.environ.get("AIM_MODEL") == "angle":
                    self._record_slot_angle(slot, slot.matched, bundle, observe)
        # 选主板：现任 0.2 s 内还有观测就留任（aim 不抖），否则提升最近有观测的
        primary = None
        # 只打"正对"的板：侧向板（facing 低）子弹只会擦过板面，实测交点横偏 +11~+39 cm。
        # 现任主板的 facing 掉下去就换到最正对的那块（真值路径的 normal·视线>0.2）。
        MIN_FACING = float(os.environ.get("AIM_MIN_FACING", "0.35"))
        if self.primary is not None and self.primary in self.slots \
                and self.primary.age(observe) <= 0.2 \
                and getattr(self.primary, "last_facing", 1.0) >= MIN_FACING:
            primary = self.primary
        elif self.slots:
            fresh = [s for s in self.slots if s.age(observe) <= 0.35]
            pool = fresh or self.slots
            primary = max(pool, key=lambda s: (round(getattr(s, "last_facing", 0.0), 2),
                                               -s.age(observe)))
        self.primary = primary
        if primary is not self._last_primary:
            self._last_primary = primary
            self.settled_at = observe
        if primary is not None:
            self.motion = primary.motion
        selected = primary.matched if primary is not None else None
        # 主板本帧没被指派（它的板瞬时没检出）：不切板、不报 no_target，用主板
        # 连续历史的拟合外推顶着——aim 保持连续，速度/位置都来自同一块板
        predicted_only = False
        if selected is None and primary is not None and primary.last_candidate is not None:
            predicted = primary.predict(observe)
            if predicted is not None:
                selected = _PredictedPlate(predicted, primary.last_candidate)
                predicted_only = True
        info = {"detections": len(detections), "selected": selected is not None,
                "slots": len(self.slots), "predicted_only": predicted_only,
                "roi": self.roi_size if self.roi_used else None}
        if selected is None:
            # 丢失目标时不喂跟踪器，让它自然过期（max_dt）；ROI 连续丢帧就退回全图
            if self.roi is not None:
                self.roi.observe(None, False)
            self.last = dict(info, reason="no_target")
            return None, None, False, self.last
        if self.roi is not None:
            self.roi.observe(selected.detection["box"], True)
        info.update(confidence=round(float(selected.detection["confidence"]), 3),
                    class_id=int(selected.detection["class_id"]),
                    large=bool(selected.detection["large"]),
                    color_id=int(selected.detection["color_id"]),
                    reprojection=round(selected.reprojection_error_px, 2),
                    world=[round(float(v), 3) for v in selected.world_position])
        self.tracker.update(selected, int(observe * 1e9))
        # 预测器研究用：把"这一帧被瞄准的板、主槽历史、真值装甲"落盘，
        # 之后离线比较不同预测模型（不用反复跑闭环）。AIM_RECORD=路径 即可。
        recorder = os.environ.get("AIM_RECORD")
        if recorder and primary is not None:
            try:
                truth = (state.get("truth") or {}).get("armors") or []
                with open(recorder, "a", encoding="utf-8") as handle:
                    handle.write(json.dumps({
                        "t": round(float(observe), 5),
                        # 真值来自比这一帧新的状态，必须单独标时间，否则离线评估
                        # 会把它当成"帧时刻的真值"，多算一整段 pipeline 时延。
                        "ts": round(float(state.get("state_sim_time") or observe), 5),
                        "p": [round(float(v), 5) for v in selected.world_position],
                        "hist": [[round(float(t), 5), [round(float(v), 5) for v in pos]]
                                 for t, pos in primary.motion.history],
                        "truth": [[int(armor["id"])] +
                                  [round(float(v), 5) for v in armor["center"]]
                                  for armor in truth],
                    }) + "\n")
            except Exception:
                pass
        # 刚体模型：真检测器没有板 id，用几何关联补上；理想检测器直接带 id
        if self.use_body:
            positions = [[float(v) for v in result.world_position] for result in candidates]
            if positions:
                # 相机的世界位置：odom 原点 + 云台旋转下的相机偏移。
                # facing 约束要用它判断"这块板的法向是否朝向相机"。
                world_from_gimbal = vision.quaternion_matrix_wxyz(
                    bundle.gimbal.quaternion_wxyz)
                camera_world = np.asarray(bundle.odom.position, dtype=np.float64) \
                    + world_from_gimbal @ np.asarray(bundle.camera.position, dtype=np.float64)
                plates = None
                if any(result.detection.get("armor_id") is not None for result in candidates):
                    plates = {}
                    for result in candidates:
                        armor_id = result.detection.get("armor_id")
                        if armor_id is not None:
                            plates[int(armor_id)] = [float(v) for v in result.world_position]
                else:
                    plates, _theta = self.body.associate(positions, now=observe,
                                                         origin=camera_world)
                if plates:
                    self.body.observe(observe, plates, origin=camera_world)

        aim, reason = self._solve_aim(selected, bundle, state, delay, observe)
        if aim is None:
            self.last = dict(info, reason=reason or "no_solution")
            return None, None, False, self.last

        feedback = vision.yaw_pitch_from_rotation(
            vision.quaternion_matrix_wxyz(bundle.gimbal.quaternion_wxyz))
        gate_fire, gate_reason = self.gate.evaluate(aim, selected, bundle, self.tracker.tracking, 0.0)
        # 到达时 facing 门：AIM_MIN_ARRIVAL_FACING>0 时，按子弹到达时刻的预测 facing 卡开火。
        # 侧向板（facing 低）子弹只会横向擦出板面，plane 归因里未命中几乎全是这一类。
        # 独立于 FireGate：bridge 有自己的对齐判定，只认 info["arrival_block"]。
        # 实测（angle 模型，ideal 状态源）0.75 最好：68% → 79%（288 发汇总）。
        arrival_gate = float(os.environ.get("AIM_MIN_ARRIVAL_FACING", "0") or 0.0)
        if arrival_gate > 0.0 and self.primary is not None:
            horizon = float(getattr(self, "last_gap", 0.0)) + float(delay or 0.0) + float(aim.fly_time_s)
            arrival = self._arrival_facing(observe, horizon, aim, bundle)
            if arrival is not None:
                info["arrival_facing"] = round(arrival, 3)
                if arrival < arrival_gate:
                    gate_fire, gate_reason = False, "arrival_facing"
                    info["arrival_block"] = True
        info.update(reason=gate_reason, distance=round(aim.distance_m, 2),
                    fly_time=round(aim.fly_time_s * 1000, 1),
                    gap=round(float(getattr(self, "last_gap", 0.0)), 3),
                    settle=round(float(observe - self.settled_at), 3),
                    tracking=bool(self.tracker.tracking),
                    aim_rate=[round(self.aim_rate[0], 4), round(self.aim_rate[1], 4)],
                    yaw_error=round(math.degrees(math.remainder(aim.yaw_rad - feedback[0], 2 * math.pi)), 2),
                    hardware_fire=gate_fire)
        self.last = info
        # _ballistic 用的是仿真口径（pitch 向下为正），不要再取负；
        # 取负只适用于管线里 calculate_aim 的"向上为正"。
        return aim.yaw_rad, aim.pitch_rad, gate_fire, info

    def _arrival_facing(self, observe, horizon, aim, bundle):
        """子弹到达时板有多正：orbit 模型的半径矢量转到到达时刻 = 板法向，点乘"板 → 我"。

        orbit 不可用时退回像素宽度 facing 的线性外推（无方向，效果差）。
        """
        motion = getattr(self.primary, "motion", None)
        orbit = getattr(motion, "orbit_state", None)
        if orbit is not None and os.environ.get("AIM_MODEL") == "orbit":
            t_ref, omega, rot_a, rot_b = orbit
            radius = math.hypot(rot_a, rot_b)
            if 0.12 < radius < 0.45 and abs(observe + horizon - t_ref) < 0.6:
                angle = omega * (observe + horizon - t_ref)
                c, s_ = math.cos(angle), math.sin(angle)
                nx, ny = (c * rot_a - s_ * rot_b) / radius, (s_ * rot_a + c * rot_b) / radius
                origin = np.asarray(bundle.odom.position, dtype=np.float64)
                to_me = origin[:2] - np.asarray(aim.world_point, dtype=np.float64)[:2]
                norm = float(np.linalg.norm(to_me))
                if norm > 1e-6:
                    return float((nx * to_me[0] + ny * to_me[1]) / norm)
        if hasattr(self.primary, "facing_at"):
            return self.primary.facing_at(observe, horizon)
        return None

    def _solve_aim(self, selected, bundle, state, delay, observe):
        """对目标运动建模后解弹道。

        use_body（理想检测器带装甲板 id）时走刚体模型：车心二次外推 + 板绕车心旋转，
        和 tools/rm_aim_sim_bridge.py 里真值 Tracker 的形式逐字一致——真值后端靠这个
        拿到 97~100%，视觉侧原来缺的就是它。
        """
        weapon = state.get("weapon") or {}
        gravity = float(weapon.get("gravity", 9.7833))
        barrel = float(weapon.get("barrel_length", 0.22))
        origin = np.asarray(bundle.odom.position, dtype=np.float64)
        if self.use_body and self.body.theta is not None and self.body.center_history:
            return self._solve_aim_body(bundle, state, delay)
        return self._solve_aim_plate(selected, bundle, state, delay, observe)

    def _solve_aim_body(self, bundle, state, delay):
        """刚体模型解算：遍历 4 块板，只打正面朝向我们的那块，取最近。"""
        weapon = state.get("weapon") or {}
        gravity = float(weapon.get("gravity", 9.7833))
        barrel = float(weapon.get("barrel_length", 0.22))
        lifetime = float(weapon.get("lifetime", 3.0) or 3.0)
        origin = np.asarray(bundle.odom.position, dtype=np.float64)
        velocity, acceleration, omega = self.body.motion()
        if velocity == (0.0, 0.0, 0.0):
            return None, "no_motion"
        center = self.body.center_history[-1][1]
        pitch_min, pitch_max = -0.5236, 0.3491
        best = None
        for plate_id in BodyTracker.PLATES:
            nx, ny, nz = BodyTracker.PLATE_NORMALS[plate_id]
            cos_t, sin_t = math.cos(self.body.theta), math.sin(self.body.theta)
            normal = (cos_t * nx - sin_t * ny, sin_t * nx + cos_t * ny, nz)
            plate = self.body.plate_now(plate_id, center, self.body.theta)
            to_me = [origin[k] - plate[k] for k in range(3)]
            length = math.sqrt(sum(c * c for c in to_me)) or 1.0
            if sum(to_me[k] * normal[k] for k in range(3)) / length <= 0.2:
                continue                                    # 背面装甲不打
            distance = length
            flight = distance / self.bullet_speed
            solution = None
            for _ in range(12):
                t = flight + delay
                predicted = self.body.plate_world(plate_id, t, center, velocity,
                                                  acceleration, omega, self.body.theta)
                solution = _ballistic(self.bullet_speed, gravity, barrel,
                                      [predicted[k] - origin[k] for k in range(3)])
                if solution is None:
                    break
                if abs(solution[2] - flight) < 1e-6:
                    flight = solution[2]
                    break
                flight = solution[2]
            if solution is None:
                continue
            yaw, pitch, flight = solution
            if not (pitch_min <= pitch <= pitch_max) or flight > lifetime:
                continue
            if best is None or distance < best[0]:
                best = (distance, plate_id, yaw, pitch, flight)
        if best is None:
            return None, "no_plate"
        distance, plate_id, yaw, pitch, flight = best
        point = self.body.plate_world(plate_id, flight + delay, center, velocity,
                                      acceleration, omega, self.body.theta)
        # 角速度从模型解析求（再解一次 t+dt），不要用瞄准值的帧间差分：
        # 板 id 关联存在 θ↔θ+π 的重名等价，瞄准解会在板之间跳，差分出来的速率是噪声。
        step = 0.016
        later = self._plate_aim(plate_id, flight + delay + step, bundle, state, delay,
                                center, velocity, acceleration, omega)
        if later is not None and later[2] == plate_id:
            self.aim_rate = (math.remainder(later[0] - yaw, 2 * math.pi) / step,
                             (later[1] - pitch) / step)
        else:
            self.aim_rate = (0.0, 0.0)
        return vision.AimResult(yaw, pitch, distance, flight, point), None

    def _plate_aim(self, plate_id, ahead, bundle, state, delay, center, velocity,
                   acceleration, omega):
        """只解某一时刻的瞄准角，用于求角速度。

        ahead 是"相对观测时刻的总视野"（= flight + delay [+ step]）。旧写法在这里
        仍然按 flight + delay 预测，等于原地再解一遍，aim_rate 恒为 0——云台拿不到
        前馈，追移动目标的稳态误差 2r/ω ≈ 1.5°，开火门限（0.45°）大部分时间不开。
        """
        weapon = state.get("weapon") or {}
        gravity = float(weapon.get("gravity", 9.7833))
        barrel = float(weapon.get("barrel_length", 0.22))
        origin = np.asarray(bundle.odom.position, dtype=np.float64)
        flight = float(np.linalg.norm(
            np.asarray(self.body.plate_now(plate_id, center, self.body.theta)) - origin)) \
            / self.bullet_speed
        for _ in range(12):
            predicted = self.body.plate_world(plate_id, ahead, center, velocity,
                                              acceleration, omega, self.body.theta)
            solution = _ballistic(self.bullet_speed, gravity, barrel,
                                  [predicted[k] - origin[k] for k in range(3)])
            if solution is None:
                return None
            if abs(solution[2] - flight) < 1e-6:
                break
            flight = solution[2]
        return solution[0], solution[1], plate_id

    def _record_slot_angle(self, slot, candidate, bundle, observe):
        """把这块板的世界方向追加到它自己的角域历史。

        每槽独立：主槽在两块共视板之间切换时，不再把历史清空重建
        （上一版一重置就只剩 3~4 个样本，二次外推立刻发散）。
        """
        direction = self._direction_of(candidate, bundle)
        if direction is None:
            return
        # 槽被重新指派到另一块板时（位置跳 >0.3 m），旧角度历史作废：
        # 两块板的 yaw 差 ~90°，混在一起 unwrap 会把斜率算成几十 rad/s。
        position = np.asarray(candidate.world_position, dtype=np.float64)
        previous = getattr(slot, "angle_last_pos", None)
        if previous is not None and float(np.linalg.norm(position - previous)) > 0.3:
            slot.angles = []
            slot.angle_ekf = _AngleEkf()
            if hasattr(slot, "angle_yaw_deg"):
                del slot.angle_yaw_deg
        slot.angle_last_pos = position
        yaw = math.atan2(direction[1], direction[0])
        elev = math.atan2(direction[2], math.hypot(direction[0], direction[1]))
        world_from_gimbal = vision.quaternion_matrix_wxyz(bundle.gimbal.quaternion_wxyz)
        camera_world = np.asarray(bundle.odom.position, dtype=np.float64) \
            + world_from_gimbal @ np.asarray(bundle.camera.position, dtype=np.float64)
        distance = float(np.linalg.norm(
            np.asarray(candidate.world_position, dtype=np.float64) - camera_world))
        slot.angles.append((float(observe), yaw, elev, distance))
        angle_log = os.environ.get("AIM_ANGLE_LOG")
        if angle_log:
            try:
                with open(angle_log, "a", encoding="utf-8") as handle:
                    handle.write(json.dumps({
                        "slot": id(slot), "t": round(float(observe), 5),
                        "yaw": round(math.degrees(yaw), 4),
                        "elev": round(math.degrees(elev), 4),
                        "r": round(float(distance), 4)}) + "\n")
            except OSError:
                pass
        # 喂角度 EKF：yaw 需要先在"本槽"的连续坐标系里展开
        if not hasattr(slot, "angle_yaw_deg"):
            slot.angle_yaw_deg = math.degrees(yaw)
        else:
            slot.angle_yaw_deg += math.degrees(math.remainder(
                yaw - math.radians(slot.angle_yaw_deg), 2.0 * math.pi))
        slot.angle_ekf.update(float(observe), slot.angle_yaw_deg, math.degrees(elev))
        newest = slot.angles[-1][0]
        span = max(0.8, self.motion_window)
        slot.angles = [row for row in slot.angles if newest - row[0] <= span]
        while len(slot.angles) > 64:
            slot.angles.pop(0)

    def _direction_of(self, candidate, bundle):
        """像素重心 → 相机射线 → 世界方向（从云台原点出发）。

        不直接用 PnP 的 world_position：它的横向噪声是 0.076 m @4 m = 1.1°/帧，
        而同一批角点的像素重心只有 ~1 px = 0.044°。距离只用来定相机偏移项。
        """
        try:
            points = np.asarray(candidate.detection["points"], dtype=np.float64)
            uv = points.mean(axis=0)
            K = bundle.camera_info.matrix
            ray = np.linalg.solve(K, np.array([uv[0], uv[1], 1.0]))
            ray = ray / max(float(np.linalg.norm(ray)), 1e-9)
            world_from_gimbal = vision.quaternion_matrix_wxyz(
                bundle.gimbal.quaternion_wxyz)
            camera_offset = np.asarray(bundle.camera.position, dtype=np.float64)
            camera_world = np.asarray(bundle.odom.position, dtype=np.float64) \
                + world_from_gimbal @ camera_offset
            distance = float(np.linalg.norm(
                np.asarray(candidate.world_position, dtype=np.float64) - camera_world))
            return world_from_gimbal @ (
                vision.R_CV_TO_GIMBAL @ ray * distance + camera_offset)
        except Exception:
            return None

    def _solve_aim_angle(self, bundle, state, delay, observe):
        """角域预测：对 yaw/elev/range 各自拟合，在命中时刻取方向，再解弹道。

        角度的测量噪声是像素级的（~0.04°），所以这里能用比世界坐标长得多、
        也稳得多的窗口；板的圆周运动在角域里同样是光滑的。
        """
        rows = self.primary.angles
        if len(rows) < 3:
            return None, "no_angle"
        last_t = float(rows[-1][0])
        now = float(observe)
        # 二次拟合窗口：离线用录制数据扫过，0.25 s 窗 +0.2 s 预测误差 p50 0.247°
        # （p90 0.655°）；线性 0.60°、三次 0.34°。EKF 反而 1.19°，因为过程噪声
        # 让速率在加速段跟不上。窗口越长越被模型误差吃掉，越短越被噪声吃掉。
        window = float(os.environ.get("AIM_ANGLE_WIN", "0.25"))
        use = [row for row in rows if now - row[0] <= window + 1e-6]
        if len(use) < 4:
            use = rows[-4:]
        times = np.asarray([row[0] for row in use], dtype=np.float64)
        us = times - times[-1]
        yaws = np.asarray(_unwrap([row[1] for row in use]), dtype=np.float64)
        elevs = np.asarray([row[2] for row in use], dtype=np.float64)
        ranges = np.asarray([row[3] for row in use], dtype=np.float64)
        degree = 2 if len(use) >= 4 else 1

        def evaluate(values, horizon):
            coef = np.polyfit(us, values, degree)
            return float(np.polyval(coef, horizon))

        distance_est = float(np.mean(ranges[-6:]))

        origin = np.asarray(bundle.odom.position, dtype=np.float64)
        weapon = state.get("weapon") or {}
        gravity = float(weapon.get("gravity", 9.7833))
        barrel = float(weapon.get("barrel_length", 0.22))
        # 预测时域必须从"这一帧"算起，而拟合的原点是最后一次**真实检出**的时刻。
        # 检出有间隔（丢帧/外推帧不记角度）时，漏掉 observe−times[-1] 这段就会
        # 系统性欠提前量：实测 10/10 发全偏负、中位 −1.6°（≈0.23 s 的缺口）。
        gap = now - last_t
        self.last_gap = gap
        flight = distance_est / max(self.bullet_speed, 1e-6)
        solution = None
        point = None
        for _ in range(10):
            horizon = gap + delay + flight
            yaw = evaluate(yaws, horizon)
            elev = evaluate(elevs, horizon)
            distance = distance_est
            point = origin + distance * np.asarray([
                math.cos(elev) * math.cos(yaw),
                math.cos(elev) * math.sin(yaw),
                math.sin(elev)])
            solution = _ballistic(self.bullet_speed, gravity, barrel, point - origin)
            if solution is None:
                return None, "no_ballistic"
            if abs(solution[2] - flight) < 1e-6:
                flight = solution[2]
                break
            flight = solution[2]
        if solution is None:
            return None, "no_ballistic"
        yaw_aim, pitch_aim, flight = solution
        # 前馈：拟合曲线在命中时刻的角速度。pitch 用仿真口径（向下为正），
        # 所以是 −d(elev)/dt。
        step = 0.016
        base = gap + delay + flight
        yaw_rate = (evaluate(yaws, base + step) - evaluate(yaws, base)) / step
        elev_rate = (evaluate(elevs, base + step) - evaluate(elevs, base)) / step
        self.aim_rate = (yaw_rate, -elev_rate)
        rate_log = os.environ.get("AIM_RATE_LOG")
        if rate_log:
            try:
                with open(rate_log, "a", encoding="utf-8") as handle:
                    handle.write(json.dumps({
                        "t": round(float(observe), 4), "n": len(rows),
                        "gap": round(float(gap), 4),
                        "yaw": round(float(yaw_deg), 3),
                        "rate": round(float(yaw_rate), 3),
                        "acc": round(float(ekf.x[2]), 3),
                        "flight": round(float(flight), 4),
                        "delay": round(float(delay), 4)}) + "\n")
            except OSError:
                pass
        distance = float(np.linalg.norm(point - origin))
        return vision.AimResult(yaw_aim, pitch_aim, distance, flight, point), None

    def _solve_aim_plate(self, selected, bundle, state, delay, observe):
        """跟踪单块板，用刚体圆周模型预测"子弹到达时刻"的板心。

        旧实现是 position + v·t + ½a·t²：速度/加速度来自窗口二次拟合，噪声大，
        而且把圆周运动当抛物线（0.6 s 窗口转 68°，二次项抓不住）。现在直接让
        模型在 t+delay+flight 处取值，圆周项由 R(ω·u) 精确承担。
        """
        if os.environ.get("AIM_MODEL") == "angle" and self.primary is not None \
                and len(self.primary.angles) >= 3:
            if getattr(self.primary, "last_facing", 1.0) < 0.2:
                return None, "edge_on"      # 侧向板不瞄也不打
            return self._solve_aim_angle(bundle, state, delay, observe)
        if not self.motion.history:
            return None, "no_motion"
        if os.environ.get("AIM_MODEL") == "center" and self.center.ready(observe):
            return self._solve_aim_center(selected, bundle, state, delay, observe)
        weapon = state.get("weapon") or {}
        gravity = float(weapon.get("gravity", 9.7833))
        barrel = float(weapon.get("barrel_length", 0.22))
        origin = np.asarray(bundle.odom.position, dtype=np.float64)
        fallback = np.asarray(selected.world_position, dtype=np.float64)
        distance = float(np.linalg.norm(fallback - origin))
        flight = distance / max(self.bullet_speed, 1e-6)
        solution = None
        point = None
        for _ in range(10):
            point = self.motion.predict(observe + delay + flight)
            if point is None:
                point = fallback
            solution = _ballistic(self.bullet_speed, gravity, barrel, point - origin)
            if solution is None:
                return None, "no_ballistic"
            if abs(solution[2] - flight) < 1e-6:
                flight = solution[2]
                break
            flight = solution[2]
        if solution is None:
            return None, "no_ballistic"
        yaw, pitch, flight = solution
        # 前馈：预测 16 ms 后的板位置再解一次瞄准角。没有它，云台追移动目标
        # 的稳态误差 2r/ω ≈ 1.68°（二阶伺服 + 2 rad/s 实测），开火门限
        # 0.45° 永远等不到对齐。
        step = 0.016
        later_point = self.motion.predict(observe + delay + flight + step)
        later = None if later_point is None else _ballistic(
            self.bullet_speed, gravity, barrel, later_point - origin)
        if later is not None:
            self.aim_rate = (math.remainder(later[0] - yaw, 2 * math.pi) / step,
                             (later[1] - pitch) / step)
        else:
            self.aim_rate = (0.0, 0.0)
        distance = float(np.linalg.norm(point - origin))
        return vision.AimResult(yaw, pitch, distance, flight, point), None

    def _solve_aim_center(self, selected, bundle, state, delay, observe):
        """车心模型解算：预测命中时刻那块板在圆周上的位置。

        板的位置 = 车心（二次外推，平滑）+ 半径·u(相位 + ω·t)。半径和相位都由
        两块共视板的几何定出，不依赖单帧 PnP 的绝对精度。
        """
        weapon = state.get("weapon") or {}
        gravity = float(weapon.get("gravity", 9.7833))
        barrel = float(weapon.get("barrel_length", 0.22))
        origin = np.asarray(bundle.odom.position, dtype=np.float64)
        plate_now = np.asarray(selected.world_position, dtype=np.float64)
        flight = float(np.linalg.norm(plate_now - origin)) / max(self.bullet_speed, 1e-6)
        solution = None
        point = None
        for _ in range(10):
            point = self.center.predict(observe + delay + flight, observe, plate_now)
            if point is None:
                return None, "no_center"
            solution = _ballistic(self.bullet_speed, gravity, barrel, point - origin)
            if solution is None:
                return None, "no_ballistic"
            if abs(solution[2] - flight) < 1e-6:
                flight = solution[2]
                break
            flight = solution[2]
        if solution is None:
            return None, "no_ballistic"
        yaw, pitch, flight = solution
        step = 0.016
        later_point = self.center.predict(observe + delay + flight + step, observe, plate_now)
        later = None if later_point is None else _ballistic(
            self.bullet_speed, gravity, barrel, later_point - origin)
        if later is not None:
            self.aim_rate = (math.remainder(later[0] - yaw, 2 * math.pi) / step,
                             (later[1] - pitch) / step)
        else:
            self.aim_rate = (0.0, 0.0)
        distance = float(np.linalg.norm(point - origin))
        return vision.AimResult(yaw, pitch, distance, flight, point), None

    def _aim_for_point(self, position, velocity, acceleration, delay, origin, state):
        """给定板的世界位置和运动，解出 (yaw, pitch, flight, 预测点)。"""
        weapon = state.get("weapon") or {}
        gravity = float(weapon.get("gravity", 9.7833))
        barrel = float(weapon.get("barrel_length", 0.22))
        flight = 0.0
        solution = None
        point = None
        for _ in range(12):
            t = delay + flight
            point = position + velocity * t + 0.5 * acceleration * t * t
            solution = _ballistic(self.bullet_speed, gravity, barrel, point - origin)
            if solution is None:
                return None
            if abs(solution[2] - flight) < 1e-6:
                flight = solution[2]
                break
            flight = solution[2]
        if solution is None:
            return None
        return solution[0], solution[1], flight, point
