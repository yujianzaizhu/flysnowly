/* 接入导航进程仿真：独立状态、接口与 Canvas，不加载内部仿真代码。 */
(function () {
  "use strict";
  const $ = (id) => document.getElementById(id);
  const canvas = $("arena");
  const ctx = canvas.getContext("2d");
  const configInputs = Array.from(document.querySelectorAll("[data-config]"));
  const driveButtons = Array.from(document.querySelectorAll("[data-drive]"));
  const startButtons = Array.from(document.querySelectorAll("[data-wa-start]"));
  const tabButtons = Array.from(document.querySelectorAll("[data-func-tab]"));
  let current = null;
  let workareaMap = null;
  let mapVersion = -1;
  let mapBusy = false;
  let paramsDirty = false;
  let funcTab = "build";
  let fitNext = true;
  let logSignature = "";
  let refreshBusy = false;
  let refreshFailures = 0;
  const view = { x: 0, y: 0, scale: 70 };

  async function request(path, options) {
    const response = await fetch(path, Object.assign({ cache: "no-store" }, options || {}));
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || ("HTTP " + response.status));
    return data;
  }

  function post(path, payload) {
    return request(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload || {}),
    });
  }

  async function refresh() {
    if (refreshBusy) return;
    refreshBusy = true;
    try {
      const state = await request("/api/nav/state");
      refreshFailures = 0;
      $("linkChip").textContent = "已连接";
      $("linkChip").className = "chip chip--ok";
      ingest(state);
    } catch (error) {
      refreshFailures += 1;
      if (refreshFailures >= 3) {
        $("linkChip").textContent = "连接中断";
        $("linkChip").className = "chip chip--error";
      }
    } finally {
      refreshBusy = false;
    }
  }

  function ingest(state) {
    current = state;
    const pose = state.pose || {};
    const velocity = state.velocity || {};
    $("poseX").textContent = Number(pose.x || 0).toFixed(2) + " m";
    $("poseY").textContent = Number(pose.y || 0).toFixed(2) + " m";
    $("poseTheta").textContent = (Number(pose.theta || 0) * 180 / Math.PI).toFixed(1) + "°";
    $("distance").textContent = Math.round(state.distance || 0) + " m";
    $("linearVelocity").textContent = Number(velocity.linear || 0).toFixed(2) + " m/s";
    $("angularVelocity").textContent = Number(velocity.angular || 0).toFixed(2) + " rad/s";
    $("leftWheel").textContent = Number(velocity.left_wheel || 0).toFixed(2) + " m/s";
    $("rightWheel").textContent = Number(velocity.right_wheel || 0).toFixed(2) + " m/s";
    $("batteryText").textContent = Number(state.battery || 0).toFixed(0) + "%";
    $("batteryFill").style.width = Number(state.battery || 0) + "%";
    if (!paramsDirty) loadConfig(state.config || {});
    updateWorkarea(state);
    renderLog(state.log || []);
    fetchMap(state.workarea);
  }

  function loadConfig(values) {
    configInputs.forEach((input) => {
      if (values[input.dataset.config] != null) input.value = values[input.dataset.config];
    });
  }

  function updateWorkarea(state) {
    const tea = state.workarea || { enabled: false };
    const session = tea.session || { state: "idle" };
    const building = session.state === "building";
    const inWorkarea = state.activity === "workarea";
    $("teaStatus").textContent = !tea.enabled ? "导航进程未启用" : tea.connected ? "导航进程已连接" : "等待导航进程响应";
    $("teaAddress").textContent = "record_path · " + (tea.host || "--") + ":" + (tea.record_port || "--");
    $("waStatus").textContent = building ? "正在构建" : (tea.last_error || "空闲");
    $("waSessionInfo").textContent = building
      ? "轨迹 " + Number(tea.trail_len || 0).toFixed(1) + " m · 位姿发布 " + Number(tea.pub_count || 0)
      : "地图版本 " + Number(tea.map_version || 0) + " · 位姿发布 " + Number(tea.pub_count || 0);
    [$("teaStatusDot"), $("waStatusDot")].forEach((dot) => {
      dot.className = "status-dot" + (building ? " busy" : tea.last_error ? " error" : tea.connected ? " online" : "");
    });
    $("funcTabs").hidden = inWorkarea;
    $("navBuildPane").hidden = inWorkarea || funcTab !== "build";
    $("navTaskPane").hidden = inWorkarea || funcTab !== "task";
    $("workareaPane").hidden = !inWorkarea;
    $("enterWorkareaBtn").disabled = !tea.enabled;
    $("workareaHint").textContent = tea.enabled
      ? "进入后由真实导航进程记录区域、通道或禁区轨迹。"
      : "导航进程桥不可用（" + (tea.reason || "未启用") + "）。";
    startButtons.forEach((button) => { button.disabled = !tea.enabled || building; });
    driveButtons.forEach((button) => { button.disabled = !tea.enabled; });
    $("waSaveBtn").disabled = !building;
    $("waCancelBtn").disabled = !building;
    $("exitWorkareaBtn").disabled = building;
  }

  async function fetchMap(tea) {
    if (!tea || !tea.enabled) {
      workareaMap = null;
      mapVersion = -1;
      return;
    }
    const version = Number(tea.map_version || 0);
    if (mapBusy || (version === mapVersion && version > 0)) return;
    mapBusy = true;
    try {
      const result = await request("/api/nav/workarea/map?v=" + mapVersion);
      if (!result.unchanged) {
        workareaMap = result.map || null;
        mapVersion = Number(result.version || version);
        fitNext = true;
      }
    } catch (error) {
      /* 下一轮状态轮询重试。 */
    } finally {
      mapBusy = false;
    }
  }

  async function action(path, payload) {
    try {
      const state = await post(path, payload);
      ingest(state);
      const result = state.workarea_result;
      if (result && result.ok === false) {
        $("workareaHint").textContent = result.info || ("操作失败 result=" + result.result);
      }
    } catch (error) {
      $("workareaHint").textContent = error.message;
    }
  }

  function drive(kind) {
    const values = (current && current.config) || {};
    const v = Number(values.arc_speed == null ? 0.4 : values.arc_speed);
    const back = Number(values.back_speed == null ? 0.3 : values.back_speed);
    const turn = Number(values.turn_speed == null ? 1.0 : values.turn_speed);
    const radius = Math.max(0.1, Number(values.arc_radius == null ? 1.0 : values.arc_radius));
    const commands = {
      fwd: { v, w: 0 }, back: { v: -back, w: 0 },
      left: { v: 0, w: turn }, right: { v: 0, w: -turn },
      arc_left: { v, w: v / radius }, arc_right: { v, w: -v / radius },
      stop: { v: 0, w: 0 },
    };
    post("/api/nav/drive", commands[kind] || commands.stop).catch(() => {});
  }

  const held = new Map();
  driveButtons.forEach((button) => {
    button.addEventListener("pointerdown", (event) => {
      if (button.disabled) return;
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      button.classList.add("pressed");
      held.set(event.pointerId, button);
      drive(button.dataset.drive);
    });
    const release = (event) => {
      if (!held.has(event.pointerId)) return;
      held.delete(event.pointerId);
      button.classList.remove("pressed");
      drive("stop");
    };
    button.addEventListener("pointerup", release);
    button.addEventListener("pointercancel", release);
    button.addEventListener("lostpointercapture", release);
  });
  window.addEventListener("blur", () => {
    held.clear();
    driveButtons.forEach((button) => button.classList.remove("pressed"));
    drive("stop");
  });

  configInputs.forEach((input) => input.addEventListener("input", () => { paramsDirty = true; }));
  $("applyParamsBtn").addEventListener("click", async () => {
    const values = {};
    configInputs.forEach((input) => { values[input.dataset.config] = Number(input.value); });
    try {
      const state = await post("/api/nav/config", { values });
      paramsDirty = false;
      ingest(state);
      $("paramMessage").textContent = "参数已应用，仅对导航进程仿真生效。";
    } catch (error) { $("paramMessage").textContent = error.message; }
  });
  $("resetParamsBtn").addEventListener("click", async () => {
    const state = await post("/api/nav/config", { action: "reset" });
    paramsDirty = false;
    ingest(state);
  });

  tabButtons.forEach((button) => button.addEventListener("click", () => {
    funcTab = button.dataset.funcTab === "task" ? "task" : "build";
    tabButtons.forEach((item) => item.classList.toggle("is-active", item === button));
    if (current) updateWorkarea(current);
  }));
  $("enterWorkareaBtn").addEventListener("click", () => action("/api/nav/activity", { activity: "workarea" }));
  $("exitWorkareaBtn").addEventListener("click", () => action("/api/nav/activity", { activity: "work" }));
  $("teaProbeBtn").addEventListener("click", () => action("/api/nav/workarea/probe", {}));
  startButtons.forEach((button) => button.addEventListener("click", () => action("/api/nav/workarea/start", { type: Number(button.dataset.waStart), check: true })));
  $("waSaveBtn").addEventListener("click", () => action("/api/nav/workarea/save", { check: true }));
  $("waCancelBtn").addEventListener("click", () => action("/api/nav/workarea/cancel", {}));

  function renderLog(entries) {
    const visible = entries.slice(-100);
    const signature = String(entries.length) + ":" + (visible.length ? visible[visible.length - 1].time : "");
    if (signature === logSignature) return;
    logSignature = signature;
    $("logCount").textContent = entries.length + " 条";
    $("log").textContent = "";
    visible.forEach((entry) => {
      const item = document.createElement("li");
      const time = document.createElement("time");
      const level = document.createElement("span");
      const message = document.createElement("span");
      time.textContent = Number(entry.time || 0).toFixed(1) + "s";
      level.textContent = entry.level || "info";
      level.className = "level " + (entry.level || "info");
      message.textContent = entry.message || "";
      item.append(time, level, message);
      $("log").appendChild(item);
    });
    $("log").scrollTop = $("log").scrollHeight;
  }

  $("logToggleBtn").addEventListener("click", () => {
    const dock = document.querySelector(".log-dock");
    dock.classList.toggle("folded");
    $("logToggleBtn").textContent = dock.classList.contains("folded") ? "展开" : "收起";
  });

  const theme = $("themeSelect");
  try { theme.value = localStorage.getItem("mowerThemeV1") || "dark"; } catch (error) { theme.value = "dark"; }
  theme.addEventListener("change", () => {
    document.documentElement.dataset.theme = theme.value;
    try { localStorage.setItem("mowerThemeV1", theme.value); } catch (error) { /* 忽略隐私模式 */ }
  });
  $("exitBtn").addEventListener("click", async () => {
    if (!window.confirm("退出导航进程仿真并关闭后端服务？")) return;
    drive("stop");
    try { await post("/api/nav/shutdown", {}); } catch (error) { /* 服务关闭后连接中断属正常 */ }
    document.body.innerHTML = "<div class='mode-switch-mask'><div><strong>控制台已退出</strong><span>后端服务已关闭，可关闭此标签页。</span></div></div>";
  });

  function allPoints() {
    const points = [];
    if (workareaMap && Array.isArray(workareaMap.sub_maps)) {
      workareaMap.sub_maps.forEach((sub) => (sub.poses || []).forEach((point) => points.push(point)));
      if (workareaMap.station_pose) points.push(workareaMap.station_pose);
    }
    const trail = (current && current.workarea && current.workarea.trail) || [];
    trail.forEach((point) => points.push(point));
    if (current && current.pose) points.push([current.pose.x, current.pose.y]);
    return points;
  }

  function fitView() {
    const rect = canvas.getBoundingClientRect();
    const points = allPoints();
    if (!points.length) { view.x = 0; view.y = 0; view.scale = 70; fitNext = false; return; }
    const xs = points.map((point) => Number(point[0]));
    const ys = points.map((point) => Number(point[1]));
    const x0 = Math.min.apply(null, xs), x1 = Math.max.apply(null, xs);
    const y0 = Math.min.apply(null, ys), y1 = Math.max.apply(null, ys);
    view.x = (x0 + x1) / 2;
    view.y = (y0 + y1) / 2;
    view.scale = Math.max(4, Math.min(180, Math.min(rect.width / Math.max(5, x1 - x0 + 2), rect.height / Math.max(5, y1 - y0 + 2))));
    fitNext = false;
  }

  function draw() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const width = Math.max(1, Math.round(rect.width * dpr));
    const height = Math.max(1, Math.round(rect.height * dpr));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width; canvas.height = height; fitNext = true;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    if (fitNext) fitView();
    const styles = getComputedStyle(document.documentElement);
    const bg = styles.getPropertyValue("--well").trim();
    const line = styles.getPropertyValue("--line").trim();
    const muted = styles.getPropertyValue("--muted").trim();
    const w = rect.width, h = rect.height;
    const sx = (x) => w / 2 + (Number(x) - view.x) * view.scale;
    const sy = (y) => h / 2 - (Number(y) - view.y) * view.scale;
    ctx.fillStyle = bg; ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = line; ctx.lineWidth = 1; ctx.globalAlpha = 0.42;
    const left = view.x - w / (2 * view.scale), right = view.x + w / (2 * view.scale);
    const bottom = view.y - h / (2 * view.scale), top = view.y + h / (2 * view.scale);
    for (let x = Math.floor(left); x <= Math.ceil(right); x += 1) { ctx.beginPath(); ctx.moveTo(sx(x), 0); ctx.lineTo(sx(x), h); ctx.stroke(); }
    for (let y = Math.floor(bottom); y <= Math.ceil(top); y += 1) { ctx.beginPath(); ctx.moveTo(0, sy(y)); ctx.lineTo(w, sy(y)); ctx.stroke(); }
    ctx.globalAlpha = 1;
    const colors = { 1: ["rgba(55,224,160,.16)", "#37e0a0"], 2: [null, "#ffd166"], 3: ["rgba(255,93,108,.16)", "#ff5d6c"] };
    const subMaps = workareaMap && Array.isArray(workareaMap.sub_maps) ? workareaMap.sub_maps : [];
    subMaps.forEach((sub) => {
      const points = sub.poses || [];
      if (points.length < 2) return;
      const color = colors[sub.type] || [null, "#4fc3ff"];
      ctx.beginPath();
      points.forEach((point, index) => { if (index === 0) ctx.moveTo(sx(point[0]), sy(point[1])); else ctx.lineTo(sx(point[0]), sy(point[1])); });
      if (sub.type !== 2) ctx.closePath();
      if (color[0]) { ctx.fillStyle = color[0]; ctx.fill(); }
      ctx.strokeStyle = color[1]; ctx.lineWidth = sub.type === 2 ? 3 : 1.8; ctx.stroke();
    });
    if (workareaMap && workareaMap.station_pose) {
      ctx.beginPath(); ctx.arc(sx(workareaMap.station_pose[0]), sy(workareaMap.station_pose[1]), 7, 0, Math.PI * 2); ctx.fillStyle = "#4fc3ff"; ctx.fill();
    }
    const trail = (current && current.workarea && current.workarea.trail) || [];
    if (trail.length > 1) {
      ctx.beginPath();
      trail.forEach((point, index) => { if (index === 0) ctx.moveTo(sx(point[0]), sy(point[1])); else ctx.lineTo(sx(point[0]), sy(point[1])); });
      ctx.strokeStyle = "#37e0a0"; ctx.lineWidth = 2.2; ctx.stroke();
    }
    if (current && current.pose) {
      const pose = current.pose, x = sx(pose.x), y = sy(pose.y), radius = Math.max(7, 0.18 * view.scale);
      ctx.beginPath(); ctx.arc(x, y, radius, 0, Math.PI * 2); ctx.fillStyle = "rgba(79,195,255,.25)"; ctx.fill(); ctx.strokeStyle = "#4fc3ff"; ctx.lineWidth = 2; ctx.stroke();
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + Math.cos(pose.theta) * radius * 1.5, y - Math.sin(pose.theta) * radius * 1.5); ctx.stroke();
    }
    ctx.fillStyle = muted; ctx.font = "10px sans-serif"; ctx.fillText("1 m = " + view.scale.toFixed(0) + " px", 12, 18);
    window.requestAnimationFrame(draw);
  }

  let drag = null;
  canvas.addEventListener("pointerdown", (event) => { drag = { x: event.clientX, y: event.clientY }; canvas.setPointerCapture(event.pointerId); });
  canvas.addEventListener("pointermove", (event) => {
    if (!drag) return;
    view.x -= (event.clientX - drag.x) / view.scale;
    view.y += (event.clientY - drag.y) / view.scale;
    drag = { x: event.clientX, y: event.clientY };
  });
  canvas.addEventListener("pointerup", () => { drag = null; });
  canvas.addEventListener("pointercancel", () => { drag = null; });
  canvas.addEventListener("wheel", (event) => { event.preventDefault(); view.scale = Math.max(4, Math.min(240, view.scale * (event.deltaY < 0 ? 1.15 : 0.87))); }, { passive: false });
  canvas.addEventListener("dblclick", () => { fitNext = true; });
  $("resetViewBtn").addEventListener("click", () => { fitNext = true; });

  refresh();
  window.setInterval(refresh, 250);
  window.requestAnimationFrame(draw);
})();
