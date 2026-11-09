/* 两套页面共用的模式入口；不读取任何一套应用的业务状态。 */
(function () {
  "use strict";
  const select = document.getElementById("simMode");
  if (!select) return;
  let switching = false;
  select.value = document.body.dataset.simMode || select.value || "standalone";

  function blockEvent(event) {
    if (!switching) return;
    event.preventDefault();
    event.stopImmediatePropagation();
  }

  function showSwitching(target) {
    switching = true;
    select.disabled = true;
    const mask = document.createElement("div");
    mask.className = "mode-switch-mask";
    mask.innerHTML = "<div><strong>正在切换仿真类型</strong><span>" +
      (target === "nav_process" ? "接入导航进程" : "内部仿真") +
      " · 当前界面动作已停止</span></div>";
    document.body.appendChild(mask);
    window.addEventListener("keydown", blockEvent, true);
    window.addEventListener("keyup", blockEvent, true);
    window.addEventListener("pointerdown", blockEvent, true);
  }

  function reloadTarget(target) {
    // 旧页面在提交新文档前先锁住主题背景，避免浏览器默认白色画布闪一下。
    const background = getComputedStyle(document.documentElement)
      .getPropertyValue("--bg")
      .trim();
    if (background) document.documentElement.style.background = background;
    window.location.replace("/?mode=" + target + "&ts=" + Date.now());
  }

  async function reconcileMode() {
    try {
      const response = await fetch("/api/mode", { cache: "no-store" });
      if (!response.ok) return;
      const state = await response.json();
      const validMode = state.mode === "standalone" || state.mode === "nav_process";
      if (!state.switching && validMode && state.mode !== document.body.dataset.simMode) {
        reloadTarget(state.mode);
      }
    } catch (error) {
      /* 页面自己的业务轮询会显示连接状态。 */
    }
  }

  async function waitAndReload(target) {
    for (let attempt = 0; attempt < 100; attempt += 1) {
      await new Promise((resolve) => window.setTimeout(resolve, 250));
      try {
        const response = await fetch("/api/mode", { cache: "no-store" });
        if (!response.ok) continue;
        const state = await response.json();
        if (state.mode === target && !state.switching) {
          reloadTarget(target);
          return;
        }
      } catch (error) {
        /* 后端重建期间连接会短暂中断，继续等待。 */
      }
    }
    window.location.reload();
  }

  reconcileMode();

  select.addEventListener("change", async () => {
    const target = select.value === "nav_process" ? "nav_process" : "standalone";
    if (target === document.body.dataset.simMode) return;
    showSwitching(target);
    try {
      const response = await fetch("/api/sim_mode", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: target }),
      });
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || "切换失败");
      if (result.mode === target && !result.switching) {
        reloadTarget(target);
        return;
      }
      await waitAndReload(target);
    } catch (error) {
      window.alert(error.message || "切换失败");
      window.location.reload();
    }
  });
})();
