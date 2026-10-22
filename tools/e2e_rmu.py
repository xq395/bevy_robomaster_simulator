"""浏览器端到端：页面 ↔ operator_web ↔ Mock/仿真引擎（MQTT + UDP 3334 + 0x0310）。

用法（先起 Mock 或 sim_engine，再起 operator_web）：
    set RMU_TEST_URL=http://127.0.0.1:5002 && python tools/e2e_rmu.py
需要 playwright 和本机 Chrome。地图点击回显、哨兵回执只有 Mock 会回，仿真引擎同样由 Mock 逻辑提供。
"""
import json
import os
import sys
import time
import urllib.request

from playwright.sync_api import sync_playwright

BASE = os.environ.get("RMU_TEST_URL", "http://127.0.0.1:5012")
results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(("PASS " if ok else "FAIL ") + name + (f"  [{detail}]" if detail else ""), flush=True)


def state():
    with urllib.request.urlopen(BASE + "/api/rmu/state", timeout=2) as r:
        return json.loads(r.read())


def wait(pred, timeout=5):
    end = time.time() + timeout
    while time.time() < end:
        v = pred()
        if v:
            return v
        time.sleep(0.05)
    return None


with sync_playwright() as p:
    browser = p.chromium.launch(channel="chrome", headless=True)
    page = browser.new_page(viewport=dict(width=1500, height=1000))
    errors = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(BASE + "/")

    check("RMU 面板出现", bool(wait(lambda: page.is_visible("#rmuPanel"), 5)))
    check("自动进入 RMU 界面", bool(wait(lambda: "rmu-mode" in page.evaluate("document.body.className"), 3)))
    check("RMU 模式隐藏模拟器控件", not page.is_visible("#take") and not page.is_visible("#startNarrow"))
    check("页面不需要滚动", page.evaluate("document.documentElement.scrollHeight <= innerHeight + 1"),
          str(page.evaluate("[document.documentElement.scrollHeight, innerHeight]")))
    check("HUD 比赛阶段", bool(wait(lambda: page.inner_text("#rmuStage") == "比赛中", 5)), page.inner_text("#rmuStage"))
    check("己方/对方血条各 5 条", page.locator("#rmuAlly .hp").count() == 5 and page.locator("#rmuEnemy .hp").count() == 5)
    check("经济已显示", page.inner_text("#rmuEco").strip().isdigit(), page.inner_text("#rmuEco"))

    page.select_option("#videoPath", "rmu")
    got = wait(lambda: page.evaluate("() => { const v=document.getElementById('video'); return v.naturalWidth }"), 6)
    check("3334 画面进入 <img>", bool(got), f"naturalWidth={got}")
    check("图传状态文字", "3334" in page.inner_text("#videoState"), page.inner_text("#videoState")[:80])

    page.select_option("#videoPath", "custom")
    got = wait(lambda: page.evaluate("() => document.getElementById('video').naturalWidth")
               if "0x0310" in page.inner_text("#videoState") and "FPS" in page.inner_text("#videoState") else None, 6)
    check("0x0310 自定义链路出画面", bool(got), page.inner_text("#videoState")[:100])
    cv = state()["custom_video"]
    check("0x0310 带宽 ≤ 120 kbit/s 且 ≤ 50 块/s", 0 < cv["kbps"] <= 120 and cv["block_hz"] <= 50,
          f"{cv['kbps']} kbit/s {cv['block_hz']} 块/s")
    page.select_option("#videoPath", "rmu")

    # 未接管：不发键鼠（上次运行可能留下接管，先显式撤销）
    urllib.request.urlopen(urllib.request.Request(BASE + "/api/rmu/drive", data=b'{"on":false}', method="POST",
                           headers={"Content-Type": "application/json", "Origin": BASE}), timeout=2)
    time.sleep(0.2)
    before = state()["sent_keyboard"]
    time.sleep(0.4)
    check("未接管不发 KeyboardMouseControl", state()["sent_keyboard"] == before)

    page.click("#rmuTake")
    check("接管生效", bool(wait(lambda: state()["driving"], 3)))
    page.click("#rmuPad")
    locked = wait(lambda: page.evaluate("() => document.pointerLockElement && document.pointerLockElement.id"), 2)
    check("鼠标锁定", locked == "rmuPad", str(locked))

    if locked == "rmuPad":
        page.keyboard.down("KeyW")
        page.keyboard.down("ShiftLeft")
        page.mouse.down()
        s = wait(lambda: (lambda st: st if st["last_sent"] and st["last_sent"]["keyboard_value"] == 0b10001 and st["last_sent"]["left"] else None)(state()), 3)
        check("W+Shift+左键 → 位掩码 0x11 与左键", bool(s), json.dumps(state()["last_sent"]))
        n0 = state()["sent_keyboard"]
        time.sleep(1.0)
        rate = state()["sent_keyboard"] - n0
        check("发送频率约 75Hz", 60 <= rate <= 90, f"{rate}/s")
        page.keyboard.up("KeyW")
        page.keyboard.up("ShiftLeft")
        page.mouse.up()
        s = wait(lambda: (lambda st: st if st["last_sent"]["keyboard_value"] == 0 and not st["last_sent"]["left"] else None)(state()), 2)
        check("松开后归零", bool(s))
        # 页面失焦：必须清零
        page.keyboard.down("KeyD")
        wait(lambda: state()["last_sent"]["keyboard_value"] == 8, 2)
        page.evaluate("() => window.dispatchEvent(new Event('blur'))")
        check("失焦清零", bool(wait(lambda: state()["last_sent"]["keyboard_value"] == 0, 2)))
        page.keyboard.up("KeyD")
        # 锁定期间所有点击都进游戏；释放锁定后才能点按钮（符合设计）
        page.evaluate("() => document.exitPointerLock()")
        check("释放鼠标后保持接管但输入清零", bool(wait(lambda: state()["driving"] and state()["last_sent"]["keyboard_value"] == 0, 2)))

    # 指令：兑换发弹量 + 限频
    page.click("text=兑换 50 发 17mm")
    check("指令发送", bool(wait(lambda: "已发送" in page.inner_text("#rmuCmdResult"), 2)), page.inner_text("#rmuCmdResult"))
    time.sleep(0.15)
    # 连点两次（同一事件循环内），第二次必须被 10Hz 限频拒绝
    page.evaluate("() => { const b=[...document.querySelectorAll('[data-cmd]')].find(x=>x.textContent.includes('兑换 50 发 17mm')); b.click(); b.click(); }")
    check("10Hz 限频提示", bool(wait(lambda: "限频" in page.inner_text("#rmuCmdResult"), 2)), page.inner_text("#rmuCmdResult"))
    page.click("summary:has-text('哨兵')")
    page.click("text=哨兵防御姿态")
    check("哨兵回执进入事件列表", bool(wait(lambda: "SentryCtrlResult" in page.inner_text("#rmuEvents"), 3)))

    # 地图点击 → MapClickCmd → Mock 回 MapClickInfo
    box = page.locator("#rmuMap").bounding_box()
    page.mouse.click(box["x"] + box["width"] * 0.5, box["y"] + box["height"] * 0.5)
    check("地图点击回显 MapClickInfo", bool(wait(lambda: "MapClickInfo" in state().get("messages", {}) or any(e["topic"] == "MapClickInfo" for e in state()["events"]), 3)))
    info = [e for e in state()["events"] if e["topic"] == "MapClickInfo"]
    if info:
        d = info[-1]["data"]
        check("地图坐标换算（28×15 中点）", abs(d["map_x"] - 14) < 0.3 and abs(d["map_y"] - 7.5) < 0.3, f"{d['map_x']},{d['map_y']}")

    # 未锁定时 Esc 撤销
    page.keyboard.press("Escape")
    check("Esc 撤销接管", bool(wait(lambda: not state()["driving"], 3)))
    n = state()["sent_keyboard"]
    time.sleep(0.4)
    check("撤销后停止发送", state()["sent_keyboard"] - n <= 1)

    # 跨源请求拒绝
    req = urllib.request.Request(BASE + "/api/rmu/drive", data=b'{"on":true}', method="POST",
                                 headers={"Content-Type": "application/json", "Origin": "http://evil.example"})
    try:
        urllib.request.urlopen(req, timeout=2)
        check("跨源接管被拒", False)
    except urllib.error.HTTPError as e:
        check("跨源接管被拒", e.code == 403, str(e.code))

    check("无页面脚本错误", not errors, "; ".join(errors))
    page.screenshot(path=r"C:\Users\34741\AppData\Local\Temp\opencode\e2e_rmu.png", full_page=True)
    browser.close()

print(f"\n{sum(results)}/{len(results)} 通过")
sys.exit(0 if all(results) else 1)
