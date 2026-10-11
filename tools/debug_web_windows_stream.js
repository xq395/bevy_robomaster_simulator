(() => {
  const refreshIntervalMs = 200;
  const objectUrls = new WeakMap();
  const simulatorRefreshIntervalMs = 500;

  async function refreshImage(image, name) {
    image.dataset.snapshotPending = "1";
    try {
      const response = await fetch(
        `/snapshot/${encodeURIComponent(name)}?t=${Date.now()}`,
        { cache: "no-store" },
      );
      if (!response.ok) throw new Error(`snapshot ${response.status}`);
      const objectUrl = URL.createObjectURL(await response.blob());
      const probe = new Image();
      await new Promise((resolve, reject) => {
        probe.onload = resolve;
        probe.onerror = reject;
        probe.src = objectUrl;
      });
      const previousUrl = objectUrls.get(image);
      image.src = objectUrl;
      image.dataset.snapshotReady = "1";
      objectUrls.set(image, objectUrl);
      if (previousUrl) URL.revokeObjectURL(previousUrl);
    } catch (_) {
      // Keep the last successfully decoded frame visible during retries.
    } finally {
      image.dataset.snapshotPending = "0";
    }
  }

  function refreshSnapshots() {
    document.querySelectorAll(".video-view").forEach((view) => {
      const image = view.querySelector("img");
      const name = view.dataset.view;
      if (!image || !name || image.dataset.snapshotPending === "1") return;
      void refreshImage(image, name);
    });
  }

  function preferAnnotatedView() {
    const raw = document.querySelector('input[data-view="raw"]');
    const annotated = document.querySelector('input[data-view="annotated"]');
    if (!raw || !annotated || !raw.checked || !annotated.checked) return;
    raw.checked = false;
    raw.dispatchEvent(new Event("change", { bubbles: true }));
  }

  function formatMeters(value) {
    return Number.isFinite(value) ? `${value.toFixed(2)} m` : "—";
  }

  function formatTargetLabel(value) {
    if (value == null || value === "") return "—";
    const labels = {
      0: "哨兵",
      1: "1 号",
      2: "2 号",
      3: "3 号",
      4: "4 号",
      5: "5 号",
      6: "前哨站",
      7: "基地",
    };
    return labels[Number(value)] ?? "—";
  }

  function installSimulatorPanel() {
    if (document.querySelector("#simulator-panel")) return;
    const rightColumn = document.querySelector(".right-column");
    if (!rightColumn) return;

    const style = document.createElement("style");
    style.textContent = `
      #simulator-panel .simulator-actions {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 8px;
        margin-top: 9px;
      }
      #simulator-panel .simulator-status {
        min-height: 16px;
        color: #71858e;
        font-size: 10px;
      }
      #simulator-panel .simulator-status.ok { color: #65d9bb; }
      #simulator-panel .simulator-status.error { color: #ff7185; }
      #simulator-panel .target-lock-actions {
        display: flex;
        flex-wrap: wrap;
        gap: 6px;
        margin-top: 9px;
      }
      #simulator-panel button:disabled { cursor: wait; opacity: .55; }
    `;
    document.head.append(style);

    const panel = document.createElement("section");
    panel.id = "simulator-panel";
    panel.innerHTML = `
      <div class="section-title">
        <h2>目标距离与命中</h2>
        <span id="simulator-panel-status">读取中</span>
      </div>
      <div class="metric-grid">
        <div class="metric good"><small>直线距离</small><strong id="target-distance">—</strong></div>
        <div class="metric"><small>水平距离</small><strong id="target-horizontal-distance">—</strong></div>
        <div class="metric"><small>高度差</small><strong id="target-height-delta">—</strong></div>
        <div class="metric"><small>发弹数</small><strong id="projectile-launch-count">—</strong></div>
        <div class="metric safe"><small>装甲板受击</small><strong id="armor-hit-count">—</strong></div>
        <div class="metric good"><small>装甲命中率</small><strong id="armor-hit-rate">—</strong></div>
        <div class="metric"><small>目标模式</small><strong id="target-lock-mode">自动</strong></div>
        <div class="metric"><small>当前 / 可见</small><strong id="target-lock-visible">—</strong></div>
      </div>
      <div class="target-lock-actions">
        <button type="button" data-target-action="lock_current">锁定当前</button>
        <button type="button" data-target-action="next">切换下一个</button>
        <button type="button" data-target-action="unlock">解除锁定</button>
        <span id="target-lock-status" class="simulator-status">按装甲数字锁定</span>
      </div>
      <div class="simulator-actions">
        <span id="projectile-stats-reset-status" class="simulator-status">发弹与命中一起清零</span>
        <button id="reset-projectile-stats" type="button">统计清零</button>
      </div>
    `;
    const tuningPanel = rightColumn.querySelector(".tuning");
    rightColumn.insertBefore(panel, tuningPanel ?? rightColumn.firstChild);
    panel.querySelector("#reset-projectile-stats").addEventListener(
      "click",
      resetProjectileStatistics,
    );
    panel.querySelectorAll("[data-target-action]").forEach(button => {
      button.addEventListener("click", () => {
        void sendTargetLockAction(button.dataset.targetAction);
      });
    });
  }

  async function refreshSimulatorPanel() {
    installSimulatorPanel();
    const status = document.querySelector("#simulator-panel-status");
    if (!status) return;

    const [visionResult, statsResult] = await Promise.allSettled([
      fetch("/data", { cache: "no-store" }).then(async response => {
        if (!response.ok) throw new Error(`vision ${response.status}`);
        return response.json();
      }),
      fetch("/simulator/stats", { cache: "no-store" }).then(async response => {
        const data = await response.json();
        if (!response.ok || !data.ok) throw new Error(data.message || `stats ${response.status}`);
        return data;
      }),
    ]);

    if (visionResult.status === "fulfilled") {
      const data = visionResult.value;
      const state = data.state ?? {};
      const targetValid = Boolean(data.tracker?.target_valid);
      const x = Number(state.x);
      const y = Number(state.y);
      const z = Number(state.z);
      const coordinatesValid = targetValid && [x, y, z].every(Number.isFinite);
      document.querySelector("#target-distance").textContent =
        formatMeters(coordinatesValid ? Math.hypot(x, y, z) : NaN);
      document.querySelector("#target-horizontal-distance").textContent =
        formatMeters(coordinatesValid ? Math.hypot(x, y) : NaN);
      document.querySelector("#target-height-delta").textContent =
        formatMeters(coordinatesValid ? z : NaN);
      const targetLock = data.target_lock ?? {};
      const lockedLabel = targetLock.locked_label;
      const currentLabel = targetLock.current_label;
      const visibleLabels = Array.isArray(targetLock.visible_labels)
        ? targetLock.visible_labels.map(formatTargetLabel).join("、")
        : "";
      document.querySelector("#target-lock-mode").textContent =
        lockedLabel == null ? "自动选择" : `锁定 ${formatTargetLabel(lockedLabel)}`;
      document.querySelector("#target-lock-visible").textContent =
        `${formatTargetLabel(currentLabel)} / ${visibleLabels || "无"}`;
    }

    if (statsResult.status === "fulfilled") {
      const launchCount = Number(statsResult.value.launch_count ?? 0);
      const armorHitCount = Number(statsResult.value.armor_hit_count ?? 0);
      const armorHitRate = Number(statsResult.value.armor_hit_rate);
      const computedRate = launchCount > 0 ? armorHitCount / launchCount : NaN;
      document.querySelector("#projectile-launch-count").textContent =
        String(launchCount);
      document.querySelector("#armor-hit-count").textContent =
        String(armorHitCount);
      document.querySelector("#armor-hit-rate").textContent =
        Number.isFinite(armorHitRate)
          ? `${(armorHitRate * 100).toFixed(1)}%`
          : Number.isFinite(computedRate)
            ? `${(computedRate * 100).toFixed(1)}%`
            : "—";
    }

    const online =
      visionResult.status === "fulfilled" && statsResult.status === "fulfilled";
    status.textContent = online ? "仿真在线" : "部分数据等待中";
    status.className = online ? "ok" : "error";
  }

  async function sendTargetLockAction(action) {
    const buttons = document.querySelectorAll("[data-target-action]");
    const status = document.querySelector("#target-lock-status");
    if (!status) return;
    buttons.forEach(button => { button.disabled = true; });
    status.textContent = "正在应用…";
    status.className = "simulator-status";
    try {
      const response = await fetch("/simulator/target-lock", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action }),
        cache: "no-store",
      });
      const data = await response.json();
      if (!response.ok || !data.ok) throw new Error(data.message || response.status);
      status.textContent = "已应用";
      status.className = "simulator-status ok";
      window.setTimeout(refreshSimulatorPanel, 150);
    } catch (error) {
      status.textContent = `操作失败：${error.message}`;
      status.className = "simulator-status error";
    } finally {
      buttons.forEach(button => { button.disabled = false; });
    }
  }

  async function resetProjectileStatistics() {
    const button = document.querySelector("#reset-projectile-stats");
    const status = document.querySelector("#projectile-stats-reset-status");
    if (!button || !status) return;
    button.disabled = true;
    status.textContent = "正在清零…";
    status.className = "simulator-status";
    try {
      const response = await fetch("/simulator/stats/reset", {
        method: "POST",
        cache: "no-store",
      });
      const data = await response.json();
      if (!response.ok || !data.ok) throw new Error(data.message || response.status);
      status.textContent = "已清零";
      status.className = "simulator-status ok";
      window.setTimeout(refreshSimulatorPanel, 150);
    } catch (error) {
      status.textContent = `清零失败：${error.message}`;
      status.className = "simulator-status error";
    } finally {
      button.disabled = false;
    }
  }

  preferAnnotatedView();
  refreshSnapshots();
  installSimulatorPanel();
  void refreshSimulatorPanel();
  window.setInterval(refreshSnapshots, refreshIntervalMs);
  window.setInterval(refreshSimulatorPanel, simulatorRefreshIntervalMs);
})();
