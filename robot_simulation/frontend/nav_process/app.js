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
  let pickingGoal = false;
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
    updateNav(state);
    renderLog(state.log || []);
    fetchMap(state.workarea);
    updateMapEmptyHint();
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
    // 构建度量来自 tea get_area_info 话题: 通道(type=2)为长度, 区域/禁区为面积
    const measureText = building && tea.build_measure != null
      ? (session.type === 2 ? "长度 " + Number(tea.build_measure).toFixed(2) + " m"
                            : "面积 " + Number(tea.build_measure).toFixed(2) + " m²")
      : "";
    $("waSessionInfo").textContent = building
      ? (measureText ? measureText + " · " : "") + "轨迹 " + Number(tea.trail_len || 0).toFixed(1) + " m · 位姿发布 " + Number(tea.pub_count || 0)
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
    // 点到点导航中速度源归 tea, 手动方向盘锁定(后端 /api/nav/drive 也会 409)
    const navActive = !!(state.nav && state.nav.active);
    driveButtons.forEach((button) => { button.disabled = !tea.enabled || navActive; });
    $("waSaveBtn").disabled = !building;
    $("waCancelBtn").disabled = !building;
    // 删除当前图是编辑命令: 桥可用且不在构建中才允许(与 tea 状态机放行范围一致)
    $("waClearBtn").disabled = !tea.enabled || !tea.connected || building;
    $("exitWorkareaBtn").disabled = building;
  }

  // ----------------------------------------------------------- 点到点任务
  // 选点 -> POST /api/nav/goal -> tea 规划导航并下发 cmd_vel, 桥订阅驱动机器人;
  // 取消/暂停/继续走 nav_task_action, 终态由 nav_task_action_result 话题判定。
  function updateNav(state) {
    const nav = state.nav || { active: false };
    const tea = state.workarea || { enabled: false };
    const pickBtn = $("pickGoalBtn");
    if (pickingGoal && nav.active) {
      // 目标已受理, 退出选点姿态(正常流程 pickGoalAt 已清, 这里兜底)
      pickingGoal = false;
      canvas.classList.remove("picking");
    }
    pickBtn.textContent = pickingGoal ? "取消选点" : "选目标点";
    pickBtn.disabled = !tea.enabled || nav.active;
    $("goalPauseBtn").disabled = !nav.active || nav.paused;
    $("goalContinueBtn").disabled = !nav.active || !nav.paused;
    $("goalCancelBtn").disabled = !nav.active;
    const hint = $("navTaskHint");
    const goal = nav.goal;
    if (pickingGoal) {
      hint.textContent = "选点中：在地图上单击确定目标点，点“取消选点”退出。";
    } else if (nav.active && goal) {
      hint.textContent = (nav.paused ? "已暂停 → " : "导航中 → ")
        + "(" + Number(goal.x).toFixed(2) + ", " + Number(goal.y).toFixed(2) + ")"
        + "，手动遥控已锁定。";
    } else if (nav.last_result) {
      const names = { 0: "已结束", 1: "已到达", 2: "失败" };
      hint.textContent = "上次任务" + (names[nav.last_result.outcome] || "已结束")
        + (nav.last_result.text ? "：" + nav.last_result.text : "") + "。";
    } else {
      hint.textContent = "点“选目标点”后在地图上单击确定目标，由 tea 规划导航。";
    }
  }

  async function navAction(path, payload) {
    try {
      const state = await post(path, payload || {});
      ingest(state);
      const result = state.nav_action_result || {};
      if (result.ok === false) {
        $("navTaskHint").textContent = "操作失败: " + (result.info || result.error || ("status=" + result.status));
      }
    } catch (error) {
      $("navTaskHint").textContent = error.message;
    }
  }

  $("pickGoalBtn").addEventListener("click", () => {
    pickingGoal = !pickingGoal;
    canvas.classList.toggle("picking", pickingGoal);
    if (current) updateNav(current);
  });
  [["goalCancelBtn", "cancel"], ["goalPauseBtn", "pause"], ["goalContinueBtn", "continue"]].forEach((pair) => {
    $(pair[0]).addEventListener("click", () => navAction("/api/nav/goal/" + pair[1]));
  });

  function canvasWorld(event) {
    const rect = canvas.getBoundingClientRect();
    return {
      x: view.x + (event.clientX - rect.left - rect.width / 2) / view.scale,
      y: view.y - (event.clientY - rect.top - rect.height / 2) / view.scale,
    };
  }

  function pickGoalAt(event) {
    const point = canvasWorld(event);
    pickingGoal = false;
    canvas.classList.remove("picking");
    if (current) updateNav(current);
    $("navTaskHint").textContent = "目标点 (" + point.x.toFixed(2) + ", " + point.y.toFixed(2) + ") 下发中…";
    navAction("/api/nav/goal", { x: point.x, y: point.y });
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
        updateMapEmptyHint();
      }
    } catch (error) {
      /* 下一轮状态轮询重试。 */
    } finally {
      mapBusy = false;
    }
  }

  // 空图提示: 桥在线且 tea 已确认读图成功(map_checked, 接入/刷新拉图后置位)仍没有
  // 任何子图时, 在画布中央提示; 构建中(轨迹正在录制)不提示, 避免干扰首次建图。
  function updateMapEmptyHint() {
    const hint = $("mapEmptyHint");
    if (!hint) return;
    const tea = (current && current.workarea) || { enabled: false };
    const building = !!(tea.session && tea.session.state === "building");
    const empty = !workareaMap || !Array.isArray(workareaMap.sub_maps) || workareaMap.sub_maps.length === 0;
    hint.hidden = !(!!tea.enabled && !!tea.connected && !!tea.map_checked && !building && empty);
    hint.textContent = workareaMap ? "工作区地图为空" : "暂未获取到工作区地图";
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
  // 键盘遥控: W/A/S/D 行驶, Q/E 左右圆弧, 方向键同 W/A/S/D, 空格急停。
  // 与按钮同语义: 按下发一次速度指令, 松开停止; 焦点在输入框/下拉时忽略。
  const KEY_COMMANDS = {
    ArrowUp: "fwd", ArrowDown: "back", ArrowLeft: "left", ArrowRight: "right",
    w: "fwd", s: "back", a: "left", d: "right",
    q: "arc_left", e: "arc_right",
  };
  const heldKeys = new Set();
  function keyName(event) {
    return event.key.length === 1 ? event.key.toLowerCase() : event.key;
  }
  function buttonForKind(kind) {
    return driveButtons.find((button) => button.dataset.drive === kind) || null;
  }
  document.addEventListener("keydown", (event) => {
    if (event.ctrlKey || event.metaKey || event.altKey) return;
    const tag = (document.activeElement && document.activeElement.tagName) || "";
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    const key = keyName(event);
    if (key === " ") {
      event.preventDefault();
      if (heldKeys.size) {
        heldKeys.clear();
        driveButtons.forEach((button) => button.classList.remove("pressed"));
        drive("stop");
      }
      return;
    }
    const kind = KEY_COMMANDS[key];
    if (!kind) return;
    const button = buttonForKind(kind);
    if (button && button.disabled) return;
    // 长按重复事件也要拦, 否则方向键会滚动页面。
    event.preventDefault();
    if (heldKeys.has(key)) return;
    heldKeys.add(key);
    if (button) button.classList.add("pressed");
    drive(kind);
  });
  document.addEventListener("keyup", (event) => {
    const key = keyName(event);
    if (!heldKeys.has(key)) return;
    heldKeys.delete(key);
    driveButtons.forEach((button) => button.classList.remove("pressed"));
    drive("stop");
  });

  window.addEventListener("blur", () => {
    held.clear();
    heldKeys.clear();
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
  // 删除当前图: 不带 id = 老协议整图清空(CLEAR_MAP, 成功码 result=1), 后端成功后自动刷新地图缓存
  $("waClearBtn").addEventListener("click", () => {
    if (!window.confirm("删除当前图将删除当前工作区地图的全部区域/通道/禁区，且不可恢复。确定删除？")) return;
    action("/api/nav/workarea/clear", {});
  });

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
    const chargePose = (current && current.workarea && current.workarea.charge_pose) || null;
    if (chargePose && (chargePose.x || chargePose.y)) points.push([chargePose.x, chargePose.y]);
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
    // 充电桩/回充点(charge_pose 话题, 紫色圆环 + 朝向短线; 零值=无桩不画)
    const chargePose = (current && current.workarea && current.workarea.charge_pose) || null;
    if (chargePose && (chargePose.x || chargePose.y)) {
      const cx = sx(chargePose.x), cy = sy(chargePose.y);
      ctx.beginPath(); ctx.arc(cx, cy, 7, 0, Math.PI * 2); ctx.strokeStyle = "#b388ff"; ctx.lineWidth = 2; ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx, cy);
      ctx.lineTo(cx + Math.cos(chargePose.yaw) * 12, cy - Math.sin(chargePose.yaw) * 12);
      ctx.stroke();
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
    // 点到点目标: 机器人->目标虚线 + 旗标(琥珀色, 与通道同色系的任务色)
    const nav = (current && current.nav) || {};
    if (nav.active && nav.goal) {
      const gx = sx(nav.goal.x), gy = sy(nav.goal.y);
      if (current && current.pose) {
        ctx.beginPath(); ctx.setLineDash([5, 4]);
        ctx.moveTo(sx(current.pose.x), sy(current.pose.y)); ctx.lineTo(gx, gy);
        ctx.strokeStyle = "#ffd166"; ctx.lineWidth = 1.4; ctx.stroke();
        ctx.setLineDash([]);
      }
      ctx.beginPath(); ctx.arc(gx, gy, 7, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(255,209,102,.25)"; ctx.fill();
      ctx.strokeStyle = "#ffd166"; ctx.lineWidth = 2; ctx.stroke();
      ctx.beginPath(); ctx.moveTo(gx - 4, gy); ctx.lineTo(gx + 4, gy); ctx.moveTo(gx, gy - 4); ctx.lineTo(gx, gy + 4); ctx.stroke();
    }
    ctx.fillStyle = muted; ctx.font = "10px sans-serif"; ctx.fillText("1 m = " + view.scale.toFixed(0) + " px", 12, 18);
    window.requestAnimationFrame(draw);
  }

  let drag = null;
  canvas.addEventListener("pointerdown", (event) => { drag = { x: event.clientX, y: event.clientY, ox: event.clientX, oy: event.clientY, moved: false }; canvas.setPointerCapture(event.pointerId); });
  canvas.addEventListener("pointermove", (event) => {
    if (!drag) return;
    // 距按下点的位移超过阈值判为平移手势, 否则抬起时按单击处理(选点用)
    if (Math.abs(event.clientX - drag.ox) + Math.abs(event.clientY - drag.oy) > 4) drag.moved = true;
    view.x -= (event.clientX - drag.x) / view.scale;
    view.y += (event.clientY - drag.y) / view.scale;
    drag.x = event.clientX; drag.y = event.clientY;
  });
  canvas.addEventListener("pointerup", (event) => {
    const wasClick = drag && !drag.moved;
    drag = null;
    if (pickingGoal && wasClick) pickGoalAt(event);
  });
  canvas.addEventListener("pointercancel", () => { drag = null; });
  canvas.addEventListener("wheel", (event) => { event.preventDefault(); view.scale = Math.max(4, Math.min(240, view.scale * (event.deltaY < 0 ? 1.15 : 0.87))); }, { passive: false });
  canvas.addEventListener("dblclick", () => { fitNext = true; });
  $("resetViewBtn").addEventListener("click", () => { fitNext = true; });
  // 刷新地图: 触发 tea 重新发布当前 map_info 并回填桥缓存, 成功时 map_version 递增,
  // 随后的状态轮询会拉到新地图; 失败信息写回提示区(下次轮询会被覆盖, 日志里保留)。
  // 功能区文字按钮与地图右上角 ⟳ 共用同一逻辑; 构建中刷新不会打断会话。
  let mapRefreshBusy = false;
  async function refreshWorkareaMap() {
    if (mapRefreshBusy) return;
    mapRefreshBusy = true;
    document.querySelectorAll("[data-refresh-map]").forEach((button) => { button.disabled = true; });
    try {
      const state = await post("/api/nav/workarea/refresh_map", {});
      ingest(state);
      const result = state.workarea_result || {};
      if (result.ok === false || result.map_error) {
        $("workareaHint").textContent = "刷新地图失败: " + (result.info || result.map_error || ("result=" + result.result));
      }
    } catch (error) {
      $("workareaHint").textContent = error.message;
    } finally {
      mapRefreshBusy = false;
      document.querySelectorAll("[data-refresh-map]").forEach((button) => { button.disabled = false; });
    }
  }
  document.querySelectorAll("[data-refresh-map]").forEach((button) => {
    button.addEventListener("click", refreshWorkareaMap);
  });

  // ------------------------------------------------------------- 左侧栏收起
  // 与内部仿真同交互: 顶栏/侧栏把手/左侧标签三处切换, 状态存 localStorage。
  const SIDE_KEY = "navSideHiddenV1";
  const layoutEl = document.querySelector(".layout");
  let sideHidden = false;
  try { sideHidden = localStorage.getItem(SIDE_KEY) === "1"; } catch (error) { sideHidden = false; }
  function applySidebarHidden(hidden) {
    if (!layoutEl) return;
    layoutEl.classList.toggle("is-side-hidden", !!hidden);
    const toggle = $("sidebarToggleBtn");
    if (toggle) {
      toggle.setAttribute("aria-pressed", hidden ? "true" : "false");
      toggle.textContent = hidden ? "◨ 侧栏" : "◧ 侧栏";
    }
    const tab = $("sidebarTab");
    if (tab) tab.hidden = !hidden;
    try { localStorage.setItem(SIDE_KEY, hidden ? "1" : "0"); } catch (error) { /* 忽略隐私模式 */ }
  }
  function toggleSidebar() {
    sideHidden = !sideHidden;
    applySidebarHidden(sideHidden);
  }
  applySidebarHidden(sideHidden);
  ["sidebarToggleBtn", "sidebarHandle", "sidebarTab"].forEach((id) => {
    const element = $(id);
    if (element) element.addEventListener("click", toggleSidebar);
  });

  // ------------------------------------------------------------- 面板单独折叠
  // 与内部仿真同交互: 点面板标题收起/展开, 状态按 data-panel 键存 localStorage。
  const PANEL_KEY = "navPanelCollapsedV1";
  let panelCollapsed = {};
  try {
    panelCollapsed = JSON.parse(localStorage.getItem(PANEL_KEY) || "{}") || {};
  } catch (error) { panelCollapsed = {}; }
  document.querySelectorAll("[data-panel]").forEach((panel) => {
    const key = panel.dataset.panel;
    const heading = panel.querySelector("h2");
    if (!heading) return;
    panel.classList.toggle("is-collapsed", panelCollapsed[key] === true);
    heading.addEventListener("click", () => {
      panelCollapsed[key] = !panelCollapsed[key];
      panel.classList.toggle("is-collapsed", panelCollapsed[key] === true);
      try { localStorage.setItem(PANEL_KEY, JSON.stringify(panelCollapsed)); } catch (error) { /* 忽略隐私模式 */ }
    });
  });

  refresh();
  window.setInterval(refresh, 250);
  window.requestAnimationFrame(draw);
})();
