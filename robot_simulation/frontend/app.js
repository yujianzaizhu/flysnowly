(() => {
  "use strict";

  const POLL_MS = 100;
  // 倍速越高，轮询越稀：仿真线程是单核满负荷跑，前端每轮要 JSON.parse
  // 上百 KB 状态并重绘，轮询太密会把服务端仿真线程（同一进程，GIL）一起拖慢。
  const POLL_TIERS = [
    [16, 150],
    [8, 120],
    [4, 80],
  ];
  let interpMs = POLL_MS;

  function pollInterval(speed) {
    for (let index = 0; index < POLL_TIERS.length; index += 1) {
      if (speed >= POLL_TIERS[index][0]) return POLL_TIERS[index][1];
    }
    return POLL_MS;
  }
  const TRAIL_LIMIT = 700;
  const ROBOT_RADIUS = 0.18;

  const canvas = document.getElementById("arena");
  const ctx = canvas.getContext("2d");
  const mapCanvas = document.createElement("canvas");
  const mapCtx = mapCanvas.getContext("2d");
  let mapVersion = -1;

  const els = {
    modeChip: document.getElementById("modeChip"),
    linkChip: document.getElementById("linkChip"),
    exitBtn: document.getElementById("exitBtn"),
    exitOverlay: document.getElementById("exitOverlay"),
    exitTip: document.getElementById("exitTip"),
    exitHint: document.getElementById("exitHint"),
    exitCloseBtn: document.getElementById("exitCloseBtn"),
    batteryText: document.getElementById("batteryText"),
    batteryFill: document.getElementById("batteryFill"),
    chargeRow: document.getElementById("chargeRow"),
    linearSpeed: document.getElementById("linearSpeed"),
    angularSpeed: document.getElementById("angularSpeed"),
    heading: document.getElementById("heading"),
    distance: document.getElementById("distance"),
    collisions: document.getElementById("collisions"),
    dockDistance: document.getElementById("dockDistance"),
    dockReserve: document.getElementById("dockReserve"),
    simTime: document.getElementById("simTime"),
    leftWheel: document.getElementById("leftWheel"),
    rightWheel: document.getElementById("rightWheel"),
    zoneList: document.getElementById("zoneList"),
    log: document.getElementById("log"),
    phaseChip: document.getElementById("phaseChip"),
    zoneChip: document.getElementById("zoneChip"),
    mapFill: document.getElementById("mapFill"),
    mapText: document.getElementById("mapText"),
    overallFill: document.getElementById("overallFill"),
    overallText: document.getElementById("overallText"),
    taskDistance: document.getElementById("taskDistance"),
    taskDuration: document.getElementById("taskDuration"),
    mowMode: document.getElementById("mowMode"),
    moduleBtn: document.getElementById("moduleBtn"),
    moduleMenu: document.getElementById("moduleMenu"),
    resetLayoutBtn: document.getElementById("resetLayoutBtn"),
    missionMessage: document.getElementById("missionMessage"),
    mapBtn: document.getElementById("mapBtn"),
    mowBtn: document.getElementById("mowBtn"),
    zoneOrder: document.getElementById("zoneOrder"),
    chargeBtn: document.getElementById("chargeBtn"),
    resumeBtn: document.getElementById("resumeBtn"),
    stopBtn: document.getElementById("stopBtn"),
    resetBtn: document.getElementById("resetBtn"),
    estopBtn: document.getElementById("estopBtn"),
    speedButtons: Array.from(document.querySelectorAll(".speed-btn")),
    mapSelect: document.getElementById("mapSelect"),
    deleteMapBtn: document.getElementById("deleteMapBtn"),
    applyParamsBtn: document.getElementById("applyParamsBtn"),
    resetParamsBtn: document.getElementById("resetParamsBtn"),
    paramMessage: document.getElementById("paramMessage"),
    importFileBtn: document.getElementById("importFileBtn"),
    importDirBtn: document.getElementById("importDirBtn"),
    importFileInput: document.getElementById("importFileInput"),
    importDirInput: document.getElementById("importDirInput"),
    importMapMessage: document.getElementById("importMapMessage"),
    zoomInBtn: document.getElementById("zoomInBtn"),
    zoomOutBtn: document.getElementById("zoomOutBtn"),
    zoomResetBtn: document.getElementById("zoomResetBtn"),
    mapCursor: document.getElementById("mapCursor"),
    collapseAllBtn: document.getElementById("collapseAllBtn"),
    expandAllBtn: document.getElementById("expandAllBtn"),
    visionSpec: document.getElementById("visionSpec"),
    themeSelect: document.getElementById("themeSelect"),
    logDock: document.querySelector(".log-dock"),
    logToggleBtn: document.getElementById("logToggleBtn"),
    logCount: document.getElementById("logCount"),
    paramTabs: Array.from(document.querySelectorAll("[data-param-tab]")),
    paramPanes: Array.from(document.querySelectorAll("[data-param-pane]")),
    dockTabs: Array.from(document.querySelectorAll("[data-dock-tab]")),
    bagFolder: document.getElementById("bagFolder"),
    bagList: document.getElementById("bagList"),
    bagQueueBtn: document.getElementById("bagQueueBtn"),
    bagQueueInfo: document.getElementById("bagQueueInfo"),
    sidebarToggleBtn: document.getElementById("sidebarToggleBtn"),
    missionPanel: document.querySelector(".mission"),
    missionFoldBtn: document.getElementById("missionFoldBtn"),
    sidebarHandle: document.getElementById("sidebarHandle"),
    sidebarTab: document.getElementById("sidebarTab"),
    bagSelectAll: document.getElementById("bagSelectAll"),
    bagOrderHint: document.getElementById("bagOrderHint"),
    bagCurrentName: document.getElementById("bagCurrentName"),
    bagBatteryInfo: document.getElementById("bagBatteryInfo"),
    popupBackBtn: document.getElementById("popupBackBtn"),
    bagPopBtn: document.getElementById("bagPopBtn"),
    bagVideoLargeBtn: document.getElementById("bagVideoLargeBtn"),
    bagVideoPopBtn: document.getElementById("bagVideoPopBtn"),
    bagVideoCloseBtn: document.getElementById("bagVideoCloseBtn"),
    bagVideoToggleBtn: document.getElementById("bagVideoToggleBtn"),
    bagVideo: document.getElementById("bagVideo"),
    bagVideoImg: document.getElementById("bagVideoImg"),
    bagVideoTitle: document.getElementById("bagVideoTitle"),
    bagVideoMeta: document.getElementById("bagVideoMeta"),
    bagRefreshBtn: document.getElementById("bagRefreshBtn"),
    bagLoadBtn: document.getElementById("bagLoadBtn"),
    bagPlayBtn: document.getElementById("bagPlayBtn"),
    bagPauseBtn: document.getElementById("bagPauseBtn"),
    bagStopBtn: document.getElementById("bagStopBtn"),
    bagSpeed: document.getElementById("bagSpeed"),
    bagProgress: document.getElementById("bagProgress"),
    bagTime: document.getElementById("bagTime"),
    bagHint: document.getElementById("bagHint"),
    bagFidelity: document.getElementById("bagFidelity"),
    bagErrAvg: document.getElementById("bagErrAvg"),
    bagErrMax: document.getElementById("bagErrMax"),
    bagSamples: document.getElementById("bagSamples"),
    bagTeleports: document.getElementById("bagTeleports"),
    bagTopic: document.getElementById("bagTopic"),
    bagDuration: document.getElementById("bagDuration"),
    bagPoseCount: document.getElementById("bagPoseCount"),
    bagPathLen: document.getElementById("bagPathLen"),
    bagTopicList: document.getElementById("bagTopicList"),
    bagTopicCount: document.getElementById("bagTopicCount"),
    bagMapInfo: document.getElementById("bagMapInfo"),
    bagCostInfo: document.getElementById("bagCostInfo"),
    dockPanes: Array.from(document.querySelectorAll("[data-dock-pane]")),
    editButtons: Array.from(document.querySelectorAll("[data-edit-kind]")),
    editRadius: document.getElementById("editRadius"),
    editSpan: document.getElementById("editSpan"),
    editMapMessage: document.getElementById("editMapMessage"),
    mapEditList: document.getElementById("mapEditList"),
    editClearBtn: document.getElementById("editClearBtn"),
  };

  const visionEls = {};
  document.querySelectorAll("[data-vision]").forEach((row) => {
    visionEls[row.dataset.vision] = {
      bar: row.querySelector(".sensor-bar i"),
      value: row.querySelector(".sensor-value"),
    };
  });

  // ------------------------------------------------------------- 界面配色
  const THEME_KEY = "mowerThemeV1";
  const THEMES = {
    dark: {
      bg: "#070b16",
      grid: "rgba(70, 110, 175, 0.09)",
      fieldTop: "#101f38",
      fieldBottom: "#0a1324",
      fieldGrid: "rgba(84, 128, 196, 0.14)",
      fieldBorder: "rgba(110, 190, 255, 0.35)",
      groundFill: "rgba(33, 76, 116, 0.11)",
      groundStroke: "rgba(98, 157, 217, 0.42)",
      cyan: "79, 195, 255",
      green: "55, 224, 160",
      muted: "139, 163, 199",
      red: "255, 93, 108",
      route: "255, 209, 102",
      obstacleLarge: "255, 143, 90",
      obstacleSmall: "255, 209, 102",
      dockIcon: "#d8fff0",
      rim: "190, 245, 255",
      wheelIdle: "130, 160, 200",
      outlineHalo: "rgba(0, 0, 0, 0.45)",
    },
    light: {
      bg: "#eef3f9",
      grid: "rgba(150, 175, 205, 0.32)",
      fieldTop: "#ffffff",
      fieldBottom: "#e9f1fa",
      fieldGrid: "rgba(150, 178, 208, 0.42)",
      fieldBorder: "rgba(60, 120, 190, 0.5)",
      groundFill: "rgba(140, 170, 200, 0.18)",
      groundStroke: "rgba(90, 130, 175, 0.55)",
      cyan: "31, 127, 196",
      green: "15, 156, 106",
      muted: "92, 117, 144",
      red: "207, 53, 69",
      route: "196, 132, 20",
      obstacleLarge: "217, 98, 47",
      obstacleSmall: "192, 138, 16",
      dockIcon: "#0b5f45",
      rim: "40, 90, 130",
      wheelIdle: "120, 140, 165",
      outlineHalo: "rgba(0, 0, 0, 0.28)",
    },
    warm: {
      bg: "#170f09",
      grid: "rgba(200, 150, 100, 0.1)",
      fieldTop: "#2c1d12",
      fieldBottom: "#1d130c",
      fieldGrid: "rgba(210, 160, 110, 0.16)",
      fieldBorder: "rgba(240, 180, 110, 0.42)",
      groundFill: "rgba(120, 85, 55, 0.2)",
      groundStroke: "rgba(215, 165, 110, 0.5)",
      cyan: "255, 138, 92",
      green: "255, 176, 46",
      muted: "207, 169, 135",
      red: "255, 107, 107",
      route: "255, 205, 140",
      obstacleLarge: "255, 159, 107",
      obstacleSmall: "255, 207, 134",
      dockIcon: "#3a2103",
      rim: "255, 230, 200",
      wheelIdle: "190, 160, 130",
      outlineHalo: "rgba(0, 0, 0, 0.5)",
    },
  };
  let palette = THEMES.dark;

  function applyTheme(name) {
    const key = Object.prototype.hasOwnProperty.call(THEMES, name) ? name : "dark";
    palette = THEMES[key];
    document.documentElement.dataset.theme = key;
    if (els.themeSelect) els.themeSelect.value = key;
    try {
      localStorage.setItem(THEME_KEY, key);
    } catch (error) {
      /* ignore */
    }
  }

  let current = null;
  let previous = null;
  let currentAt = 0;
  let trail = [];
  // 话题图层开关：日志里的 grid_costmap（红色代价方块）、轨迹、图像窗口都可单独屏蔽
  const LAYER_KEY = "mowerTopicLayersV1";
  let layerVisible = { pose: true, costmap: true, image: true, map: true, static_costmap: true };
  try {
    const saved = JSON.parse(localStorage.getItem(LAYER_KEY) || "null");
    if (saved && typeof saved === "object") layerVisible = Object.assign(layerVisible, saved);
  } catch (error) {
    /* ignore */
  }
  let lastSampleTime = -1;
  let lastTeleportsMark = 0;
  let lastRestarts = 0;
  let lastLogName = "";
  let lastBagActive = false;
  let lastBump = 0;
  let bumpFlashUntil = 0;
  let logSignature = "";
  let zoneRows = [];
  let lastMapId = null;
  let mapOptionsSignature = "";
  let viewWorldKey = "";
  let viewZoom = 1;
  let viewCenterX = 0;
  let viewCenterY = 0;
  let viewReady = false;
  let lastGeometry = null;
  let panState = null;
  let zoneAutoValue = "";
  let zoneInputDirty = false;
  let modeTouched = false;
  let paramsDirty = false;
  const configInputs = Array.from(document.querySelectorAll("[data-config]"));

  const wrap = (angle) => {
    const twoPi = Math.PI * 2;
    return ((((angle + Math.PI) % twoPi) + twoPi) % twoPi) - Math.PI;
  };
  const lerp = (a, b, t) => a + (b - a) * t;
  const clamp = (value, low, high) => Math.max(low, Math.min(high, value));

  // ------------------------------------------------------------- 数据同步
  async function request(path, body) {
    const options = body
      ? {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(body),
        }
      : { cache: "no-store" };
    const response = await fetch(path, options);
    if (!response.ok) {
      let message = "HTTP " + response.status;
      try {
        const payload = await response.json();
        if (payload && payload.error) message = payload.error;
      } catch (error) {
        /* ignore */
      }
      throw new Error(message);
    }
    return response.json();
  }

  function ingest(state) {
    previous = current || state;
    current = state;
    currentAt = performance.now();

    const replaying = !!(state.bag && state.bag.loaded);
    if (lastMapId !== null && state.map_id !== lastMapId) {
      // 连续播放日志会重建场地，这里保留轨迹以便对照上一段作业
      if (!replaying) trail = [];
      mapVersion = -1;
      zoneAutoValue = "";
      zoneInputDirty = false;
      els.zoneOrder.value = "";
    }
    if (lastMapId === null || state.map_id !== lastMapId) {
      lastMapId = state.map_id;
      els.mapSelect.value = String(state.map_id);
      modeTouched = false;
    }
    if (!modeTouched && state.mowing_mode) {
      els.mowMode.value = state.mowing_mode;
    }
    if (state.config && !paramsDirty) {
      loadConfig(state.config);
    }
    if (state.world && state.world.vision) {
      els.visionSpec.textContent =
        state.world.vision.fov_deg.toFixed(0) + "° FOV · " + state.world.vision.range.toFixed(2) + " m";
    }
    if (state.world && state.world.maps) {
      updateMapOptions(state.world.maps);
      updateDeleteButton();
    }
    if (state.world) {
      updateEditAvailability(state);
      renderMapEdits(state.world.edits);
    }

    if (state.time + 1e-6 < lastSampleTime && !replaying) {
      trail = [];
      previous = state;
      mapVersion = -1;
    }
    lastSampleTime = state.time;

    const bag = state.bag || {};
    const teleports = Number(bag.teleports || 0);
    const restarts = Number(bag.restarts || 0);
    const logName = bag.name || "";
    const bagRestarted =
      (logName && lastLogName && logName !== lastLogName) ||
      restarts > lastRestarts ||
      (!!bag.active && !lastBagActive);
    lastRestarts = restarts;
    lastBagActive = !!bag.active;
    if (logName) lastLogName = logName;
    if (bagRestarted) {
      // 刚开始回放 / 换日志 / 拖了进度条：从当前位置重新起笔画，不画"初始跳线"
      trail = [];
    }

    const tail = trail[trail.length - 1];
    const step = tail ? Math.hypot(state.pose.x - tail.x, state.pose.y - tail.y) : 0.0;
    if (!tail || step > 0.02) {
      // 回放中发生重定位、或位置突然跳很远：这一段画成红色跳线
      const jump = !!tail && (teleports > lastTeleportsMark || step > 2.0);
      lastTeleportsMark = teleports;
      trail.push({ x: state.pose.x, y: state.pose.y, jump });
      if (trail.length > TRAIL_LIMIT) trail.shift();
    }

    if (state.collisions > lastBump) {
      bumpFlashUntil = performance.now() + 650;
    }
    lastBump = state.collisions;

    if (state.map) {
      lastMapPayload = state.map;
      updateMapImage(state.map);
    }
    updatePanels(state);
    if (state.bag) {
      renderBagState(state.bag);
      refreshCostmap(false);
    }
  }

  function setConnected(connected) {
    els.linkChip.textContent = connected ? "已连接" : "连接中断";
    els.linkChip.className = "chip " + (connected ? "chip--ok" : "chip--down");
  }

  let mapPollTick = 0;
  let lastMapPayload = null;

  let shuttingDown = false;

  async function poll() {
    if (shuttingDown) return;
    const speed = current && current.speed ? current.speed : 1;
    const wait = pollInterval(speed);
    // 大实景地图的栅格位图有 300+ KB，隔几次轮询取一次，其余只取状态
    mapPollTick += 1;
    const mapEvery = speed >= 8 ? 8 : 4;
    const withMap = speed < 4 || mapPollTick % mapEvery === 0;
    try {
      const state = await request("/api/state" + (withMap ? "" : "?map=0"));
      ingest(state);
      setConnected(true);
    } catch (error) {
      setConnected(false);
    }
    interpMs = wait;
    window.setTimeout(poll, wait);
  }

  async function send(path, body) {
    try {
      const state = await request(path, body || {});
      setConnected(true);
      ingest(state);
    } catch (error) {
      if (error.name === "TypeError") {
        setConnected(false);
      } else {
        els.missionMessage.textContent = error.message;
      }
    }
  }

  function updateMapOptions(maps) {
    const signature = maps
      .map((item) => item.id + ":" + item.name + ":" + (item.imported ? 1 : 0))
      .join("|");
    if (signature === mapOptionsSignature) return;
    mapOptionsSignature = signature;
    els.mapSelect.textContent = "";
    maps.forEach((item) => {
      const option = document.createElement("option");
      option.value = String(item.id);
      option.textContent =
        item.id + " · " + item.name + (item.imported ? " · 导入" : "");
      els.mapSelect.append(option);
    });
    if (current && current.map_id != null) {
      els.mapSelect.value = String(current.map_id);
    }
  }

  // ------------------------------------------------------------- 地图编辑
  const EDIT_LABELS = {
    static: "静态障碍",
    dynamic: "动态障碍",
    no_go: "禁区",
  };
  let editKind = null;
  let rectStart = null;
  let rectDraft = null;
  let editLocked = true;
  let editLockReason = "请先完成自动建图";
  let pathEditId = null;
  let pathEditA = null;
  let hoverWorld = null;
  let editSignature = "";
  let editBusy = false;

  function currentEditRadius() {
    const value = parseFloat(els.editRadius ? els.editRadius.value : "0.35");
    if (!isFinite(value)) return 0.35;
    return Math.min(1.5, Math.max(0.12, value));
  }

  function defaultEditRadius(kind) {
    if (kind === "dynamic") return 0.30;
    return 0.35;
  }

  function updateEditHint() {
    els.editButtons.forEach((button) => {
      button.classList.toggle("is-active", button.dataset.editKind === editKind);
    });
    if (els.editMapMessage) {
      if (editLocked) {
        els.editMapMessage.textContent = editLockReason;
      } else if (pathEditId) {
        els.editMapMessage.textContent =
          pathEditA
            ? "动态障碍物 #" + pathEditId + "：已选起点 A，请点击终点 B"
            : "动态障碍物 #" + pathEditId + "：请点击巡逻路径起点 A";
      } else if (!editKind) {
        els.editMapMessage.textContent =
          "点“添加…”后在地图上点击放置；禁区按住拖动框选矩形。静态/动态障碍是实体（视觉可探测），禁区只禁止进入与切割。";
      } else if (editKind === "no_go") {
        els.editMapMessage.textContent = "在地图上按住左键拖出矩形框选禁区，松开即生效；再次点击按钮取消";
      } else {
        els.editMapMessage.textContent =
          "在地图上点击放置" +
          EDIT_LABELS[editKind] +
          "（半径 " +
          currentEditRadius().toFixed(2) +
          " m），再次点击按钮取消";
      }
    }
    canvas.style.cursor = editKind
      ? editKind === "no_go"
        ? "crosshair"
        : "crosshair"
      : "";
  }

  function setEditKind(kind) {
    if (editLocked) {
      if (els.editMapMessage) els.editMapMessage.textContent = editLockReason;
      return;
    }
    if (editKind !== kind) {
      editKind = kind;
      els.editRadius.value = defaultEditRadius(editKind).toFixed(2);
    } else {
      editKind = null;
    }
    updateEditHint();
  }

  async function placeEdit(point) {
    if (!editKind || editBusy) return;
    editBusy = true;
    const payload = {
      action:
        editKind === "static"
          ? "add_static"
          : editKind === "dynamic"
          ? "add_dynamic"
          : "add_no_go",
      x: Number(point.x.toFixed(2)),
      y: Number(point.y.toFixed(2)),
      r: currentEditRadius(),
    };
    if (editKind === "dynamic") {
      const span = parseFloat(els.editSpan ? els.editSpan.value : "1.6");
      payload.span = isFinite(span) ? span : 1.6;
    }
    try {
      const state = await request("/api/map/edit", payload);
      ingest(state);
      if (els.editMapMessage) {
        els.editMapMessage.textContent =
          "已添加" + EDIT_LABELS[editKind] + " @ (" + payload.x.toFixed(2) + ", " + payload.y.toFixed(2) + ")";
      }
    } catch (error) {
      if (els.editMapMessage) els.editMapMessage.textContent = "添加失败：" + error.message;
    } finally {
      editBusy = false;
    }
  }

  function updateEditAvailability(state) {
    const zones = (state.zones || []).length;
    const task = state.task || {};
    const phase = task.phase || "idle";
    const busy = !!task.type && !["idle", "done", "stopped"].includes(phase);
    const locked = zones === 0 || busy;
    const reason = zones === 0 ? "请先完成自动建图，再编辑地图" : busy ? "任务进行中不能编辑地图" : "";
    if (locked) {
      editKind = null;
      pathEditId = null;
      pathEditA = null;
      rectStart = null;
      rectDraft = null;
    }
    const changed = locked !== editLocked || reason !== editLockReason;
    editLocked = locked;
    editLockReason = reason;
    els.editButtons.forEach((button) => {
      button.disabled = locked;
    });
    if (els.editClearBtn) els.editClearBtn.disabled = locked;
    if (changed) updateEditHint();
  }

  async function commitPathEdit(point) {
    if (!pathEditId || editLocked) return;
    if (!pathEditA) {
      pathEditA = point;
      if (els.editMapMessage) {
        els.editMapMessage.textContent = "已选择起点 A，请点击终点 B（路径下限 0.4 m）";
      }
      return;
    }
    const a = pathEditA;
    const id = pathEditId;
    pathEditA = null;
    pathEditId = null;
    editBusy = true;
    try {
      const state = await request("/api/map/edit", {
        action: "update_dynamic",
        id,
        ax: Number(a.x.toFixed(2)),
        ay: Number(a.y.toFixed(2)),
        bx: Number(point.x.toFixed(2)),
        by: Number(point.y.toFixed(2)),
      });
      ingest(state);
      if (els.editMapMessage) {
        els.editMapMessage.textContent =
          "动态障碍物 #" +
          id +
          " 路径已更新：( " +
          a.x.toFixed(2) +
          ", " +
          a.y.toFixed(2) +
          " ) → ( " +
          point.x.toFixed(2) +
          ", " +
          point.y.toFixed(2) +
          " )";
      }
    } catch (error) {
      if (els.editMapMessage) els.editMapMessage.textContent = "更新路径失败：" + error.message;
    } finally {
      editBusy = false;
    }
  }

  function startPathEdit(id) {
    if (editLocked) {
      if (els.editMapMessage) els.editMapMessage.textContent = editLockReason;
      return;
    }
    editKind = null;
    pathEditA = null;
    pathEditId = pathEditId === id ? null : id;
    updateEditHint();
  }

  async function commitRect() {
    const draft = rectDraft;
    rectStart = null;
    rectDraft = null;
    if (!draft || editBusy) return;
    if (draft.x1 - draft.x0 < 0.3 || draft.y1 - draft.y0 < 0.3) {
      if (els.editMapMessage) els.editMapMessage.textContent = "禁区太小（至少 0.3 m），请重新框选";
      return;
    }
    editBusy = true;
    try {
      const state = await request("/api/map/edit", {
        action: "add_no_go",
        x0: Number(draft.x0.toFixed(2)),
        y0: Number(draft.y0.toFixed(2)),
        x1: Number(draft.x1.toFixed(2)),
        y1: Number(draft.y1.toFixed(2)),
      });
      ingest(state);
      if (els.editMapMessage) {
        els.editMapMessage.textContent =
          "已添加矩形禁区 (" +
          draft.x0.toFixed(2) +
          ", " +
          draft.y0.toFixed(2) +
          ") → (" +
          draft.x1.toFixed(2) +
          ", " +
          draft.y1.toFixed(2) +
          ")";
      }
    } catch (error) {
      if (els.editMapMessage) els.editMapMessage.textContent = "添加禁区失败：" + error.message;
    } finally {
      editBusy = false;
    }
  }

  async function deleteEdit(kind, id) {
    try {
      const state = await request("/api/map/edit", { action: "delete", kind, id });
      ingest(state);
      if (els.editMapMessage) els.editMapMessage.textContent = "已删除" + (EDIT_LABELS[kind] || kind);
    } catch (error) {
      if (els.editMapMessage) els.editMapMessage.textContent = "删除失败：" + error.message;
    }
  }

  function renderMapEdits(edits) {
    if (!els.mapEditList) return;
    const data = edits || { static: [], dynamic: [], no_go: [] };
    const signature = JSON.stringify(data);
    if (signature === editSignature) return;
    editSignature = signature;
    els.mapEditList.textContent = "";
    const rows = [];
    ["static", "dynamic", "no_go"].forEach((kind) => {
      (data[kind] || []).forEach((item) => {
        rows.push({ kind, item });
      });
    });
    if (!rows.length) {
      const empty = document.createElement("p");
      empty.className = "hint";
      empty.textContent = "当前地图没有编辑对象";
      els.mapEditList.appendChild(empty);
      return;
    }
    rows.forEach(({ kind, item }) => {
      const row = document.createElement("div");
      row.className = "edit-item";
      const title = document.createElement("strong");
      title.textContent = EDIT_LABELS[kind] + " ";
      const meta = document.createElement("em");
      meta.textContent =
        kind === "dynamic" && item.ax != null
          ? "#" +
            item.id +
            " · 路径 (" +
            item.ax.toFixed(2) +
            ", " +
            item.ay.toFixed(2) +
            ") → (" +
            item.bx.toFixed(2) +
            ", " +
            item.by.toFixed(2) +
            ") · r " +
            item.r.toFixed(2) +
            " m"
          : kind === "no_go" && item.x0 != null
          ? "#" +
            item.id +
            " · (" +
            item.x0.toFixed(2) +
            ", " +
            item.y0.toFixed(2) +
            ") → (" +
            item.x1.toFixed(2) +
            ", " +
            item.y1.toFixed(2) +
            ")"
          : "#" +
            item.id +
            " · (" +
            item.x.toFixed(2) +
            ", " +
            item.y.toFixed(2) +
            ") · r " +
            item.r.toFixed(2) +
            " m";
      if (kind === "dynamic") {
        const pathBtn = document.createElement("button");
        pathBtn.type = "button";
        pathBtn.className = "edit-path";
        pathBtn.textContent = pathEditId === item.id ? "取消编辑" : "编辑路径";
        pathBtn.disabled = editLocked;
        pathBtn.addEventListener("click", () => startPathEdit(item.id));
        row.appendChild(title);
        row.appendChild(meta);
        row.appendChild(pathBtn);
      } else {
        row.appendChild(title);
        row.appendChild(meta);
      }
      const remove = document.createElement("button");
      remove.type = "button";
      remove.textContent = "删除";
      remove.disabled = editLocked;
      remove.addEventListener("click", () => deleteEdit(kind, item.id));
      row.appendChild(remove);
      els.mapEditList.appendChild(row);
    });
  }

  function updateDeleteButton() {
    const maps = (current && current.world && current.world.maps) || [];
    const selected = maps.find(
      (item) => String(item.id) === String(els.mapSelect.value)
    );
    els.deleteMapBtn.disabled = !selected || !selected.imported;
    els.deleteMapBtn.title = selected && selected.imported
      ? "删除当前导入地图"
      : "内置地图不可删除";
  }

  // ------------------------------------------------------------- 地图位图
  function updateMapImage(map) {
    if (!map || map.version === mapVersion) return;
    mapVersion = map.version;
    mapCanvas.width = map.cols;
    mapCanvas.height = map.rows;
    const image = mapCtx.createImageData(map.cols, map.rows);
    const data = image.data;
    const occupancy = map.occupancy;
    const mowed = map.mowed;
    const cols = map.cols;
    const rows = map.rows;
    for (let index = 0; index < cols * rows; index += 1) {
      const pixel = index * 4;
      const occ = occupancy.charCodeAt(index);
      if (occ === 50) {
        data[pixel] = 255;
        data[pixel + 1] = 132;
        data[pixel + 2] = 86;
        data[pixel + 3] = 210;
      }
      if (mowed.charCodeAt(index) === 49) {
        data[pixel] = 55;
        data[pixel + 1] = 224;
        data[pixel + 2] = 160;
        data[pixel + 3] = 105;
      } else if (occ === 49) {
        data[pixel] = 79;
        data[pixel + 1] = 195;
        data[pixel + 2] = 255;
        data[pixel + 3] = 16;
      }
    }
    mapCtx.putImageData(image, 0, 0);
  }

  // ------------------------------------------------------------- 面板
  function updatePanels(state) {
    const task = state.task || {};
    const phase = task.phase || "idle";
    const charging = state.charging;
    const battery = clamp(state.battery, 0, 100);

    const modeLabel =
      task.type === "map"
        ? "建图中"
        : task.type === "mow"
        ? "割草中"
        : phase === "charging" || task.type === null
        ? "待机"
        : "运行中";
    els.modeChip.textContent = charging ? "充电中" : modeLabel;
    els.modeChip.className =
      "chip " +
      (charging
        ? "chip--charge"
        : task.type === "map"
        ? "chip--map"
        : task.type === "mow"
        ? "chip--mow"
        : "chip--manual");

    els.phaseChip.textContent = task.phase_label || "待机";
    els.phaseChip.className =
      "chip " +
      (phase === "charging" || phase === "charge_go" || phase === "charge_return"
        ? "chip--charge"
        : task.type === "map"
        ? "chip--map"
        : task.type === "mow"
        ? "chip--mow"
        : "chip--manual");

    els.zoneChip.textContent = task.current_zone ? "区域 " + task.current_zone : "区域 -";

    els.batteryText.textContent = battery.toFixed(1) + "%";
    els.batteryFill.style.width = battery + "%";
    els.batteryFill.className = battery < 20 ? "low" : battery < 50 ? "mid" : "";
    els.chargeRow.classList.toggle("charging", charging);
    if (charging) {
      els.chargeRow.textContent = "⚡ 正在充电";
    } else if (task.charge_reason === "low") {
      els.chargeRow.textContent = "低电量回充中…";
    } else if (battery < 30) {
      els.chargeRow.textContent = "电量偏低";
    } else {
      els.chargeRow.textContent = "未在充电";
    }

    els.linearSpeed.textContent = state.velocity.linear.toFixed(2);
    els.angularSpeed.textContent = state.velocity.angular.toFixed(2);
    const heading = ((((state.pose.theta * 180) / Math.PI) % 360) + 360) % 360;
    els.heading.textContent = heading.toFixed(0) + "°";
    els.distance.textContent = Math.round(state.distance);
    els.collisions.textContent = String(state.collisions);
    if (state.dock_distance != null) {
      els.dockDistance.textContent = state.dock_distance.toFixed(1);
      els.dockReserve.textContent = (state.dock_distance * 0.65 + 3).toFixed(0);
    }
    els.simTime.textContent = Math.round(state.time);
    els.leftWheel.textContent = (state.velocity.left_wheel || 0).toFixed(2);
    els.rightWheel.textContent = (state.velocity.right_wheel || 0).toFixed(2);

    updateVision(state.vision || {});
    updateZones(state.zones || []);
    updateMission(task, state);

    renderLog(state.log);
  }

  function updateVision(vision) {
    const range = vision.range || 1.0;
    ["front", "left", "right", "nearest"].forEach((name) => {
      const target = visionEls[name];
      if (!target) return;
      const distance = typeof vision[name] === "number" ? vision[name] : range;
      const proximity = 1 - clamp(distance / range, 0, 1);
      target.bar.style.width = Math.round(proximity * 100) + "%";
      target.bar.style.background =
        distance < 0.3 ? "#ff5d6c" : distance < 0.6 ? "#ffb02e" : "#37e0a0";
      target.value.textContent = distance.toFixed(2) + " m";
    });
  }

  function ensureZoneRows(zones) {
    if (zoneRows.length === zones.length) return;
    els.zoneList.textContent = "";
    zoneRows = zones.map(() => {
      const row = document.createElement("div");
      row.className = "zone-item";
      const name = document.createElement("span");
      name.className = "zone-name";
      const bar = document.createElement("div");
      bar.className = "bar";
      const fill = document.createElement("i");
      bar.append(fill);
      const pct = document.createElement("span");
      pct.className = "zone-pct";
      row.append(name, bar, pct);
      els.zoneList.append(row);
      return { row, name, fill, pct };
    });
  }

  function updateZones(zones) {
    ensureZoneRows(zones);
    const joined = zones.map((zone) => zone.id).join(",");
    if (
      zones.length &&
      joined !== zoneAutoValue &&
      (!zoneInputDirty || els.zoneOrder.value === zoneAutoValue)
    ) {
      zoneAutoValue = joined;
      els.zoneOrder.value = joined;
    }
    zones.forEach((zone, index) => {
      const target = zoneRows[index];
      if (!target) return;
      target.name.textContent = String(zone.id);
      const pct = clamp(zone.coverage, 0, 1) * 100;
      target.fill.style.width = pct.toFixed(2) + "%";
      target.pct.textContent = pct.toFixed(2) + "%";
      target.row.className =
        "zone-item" +
        (zone.current ? " current" : "") +
        (zone.selected ? "" : " unselected");
    });
  }

  function formatDuration(seconds) {
    const total = Math.max(0, Math.floor(seconds || 0));
    const minutes = Math.floor(total / 60);
    const rest = total % 60;
    if (minutes >= 60) {
      const hours = Math.floor(minutes / 60);
      const mins = minutes % 60;
      return hours + ":" + String(mins).padStart(2, "0") + ":" + String(rest).padStart(2, "0");
    }
    return String(minutes).padStart(2, "0") + ":" + String(rest).padStart(2, "0");
  }

  function updateMission(task, state) {
    const mapProgress = clamp(task.map_progress || 0, 0, 1);
    const coverage = clamp(task.coverage || 0, 0, 1);
    const overall = clamp(
      typeof task.overall_coverage === "number" ? task.overall_coverage : coverage,
      0,
      1
    );
    els.mapFill.style.width = (mapProgress * 100).toFixed(2) + "%";
    els.mapText.textContent = (mapProgress * 100).toFixed(2) + "%";
    els.overallFill.style.width = (overall * 100).toFixed(2) + "%";
    els.overallText.textContent = (overall * 100).toFixed(2) + "%";
    els.taskDistance.textContent = Math.round(Number(task.distance || state.distance || 0));
    els.taskDuration.textContent = formatDuration(task.duration || 0);
    els.missionMessage.textContent = task.message || "";
    els.speedButtons.forEach((button) => {
      button.classList.toggle("active", Number(button.dataset.speed) === state.speed);
    });
  }

  function renderLog(entries) {
    const last = entries.length ? entries[entries.length - 1] : null;
    const signature = entries.length + "|" + (last ? last.time + "|" + last.message : "");
    if (signature === logSignature) return;
    logSignature = signature;
    if (els.logCount) els.logCount.textContent = entries.length + " 条";
    els.log.textContent = "";
    entries
      .slice(-30)
      .reverse()
      .forEach((entry) => {
        const item = document.createElement("li");
        item.className = "log-item log--" + entry.level;
        const time = document.createElement("span");
        time.className = "log-time";
        time.textContent = entry.time.toFixed(1) + "s";
        const message = document.createElement("span");
        message.className = "log-msg";
        message.textContent = entry.message;
        item.append(time, message);
        els.log.append(item);
      });
  }

  // ------------------------------------------------------------- 绘制
  function fitScale(world, width, height) {
    const padding = 30;
    return Math.min(
      (width - padding * 2) / world.width,
      (height - padding * 2) / world.height
    );
  }

  function makeGeometry(world, width, height) {
    const scale = fitScale(world, width, height) * viewZoom;
    return {
      width,
      height,
      scale,
      world,
      x: (value) => width / 2 + (value - viewCenterX) * scale,
      y: (value) => height / 2 - (value - viewCenterY) * scale,
      toWorld: (sx, sy) => ({
        x: viewCenterX + (sx - width / 2) / scale,
        y: viewCenterY - (sy - height / 2) / scale,
      }),
    };
  }

  function resetView(world) {
    viewZoom = 1;
    viewCenterX = world.width / 2;
    viewCenterY = world.height / 2;
    viewReady = true;
  }

  function zoomAt(screenX, screenY, factor) {
    if (!lastGeometry) return;
    const before = lastGeometry.toWorld(screenX, screenY);
    viewZoom = clamp(viewZoom * factor, 0.35, 12);
    const scale = fitScale(lastGeometry.world, lastGeometry.width, lastGeometry.height) * viewZoom;
    viewCenterX = before.x - (screenX - lastGeometry.width / 2) / scale;
    viewCenterY = before.y + (screenY - lastGeometry.height / 2) / scale;
  }

  function poseAt(now) {
    if (!current) return { x: 5.6, y: 3.5, theta: 0 };
    if (!previous) return current.pose;
    const t = Math.max(0, Math.min(1, (now - currentAt) / interpMs));
    const a = previous.pose;
    const b = current.pose;
    if (Math.hypot(b.x - a.x, b.y - a.y) > 1.5) return b;
    return {
      x: lerp(a.x, b.x, t),
      y: lerp(a.y, b.y, t),
      theta: a.theta + wrap(b.theta - a.theta) * t,
    };
  }

  function draw(now) {
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    if (!width || !height) return;
    const dpr = window.devicePixelRatio || 1;
    const pixelWidth = Math.round(width * dpr);
    const pixelHeight = Math.round(height * dpr);
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = pixelWidth;
      canvas.height = pixelHeight;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);

    const state = current;
    const world = (state && state.world) || {
      width: 10,
      height: 6.4,
      obstacles: [],
      dock: { x: 5.05, y: 3.5, r: 0.45 },
    };
    const worldKey =
      (state ? state.map_id : "none") +
      ":" +
      world.width.toFixed(2) +
      "x" +
      world.height.toFixed(2);
    if (!viewReady || worldKey !== viewWorldKey) {
      viewWorldKey = worldKey;
      resetView(world);
    }
    const geom = makeGeometry(world, width, height);
    lastGeometry = geom;
    drawBackground(geom);
    drawGroundBoundary(geom, world.polygon || []);

    if (state) {
      drawMap(geom, state.map || lastMapPayload);
      drawZones(geom, state.zones || []);
      drawGroundObstacles(
        geom,
        (state.world && state.world.obstacle_shapes) || []
      );
      drawNoGo(geom, (state.world && state.world.no_go) || []);
      drawEditLabels(geom, state.world && state.world.edits, state.world && state.world.dynamic);
      drawPathEditPreview(geom);
      drawRectDraft(geom);
      drawDynamicPaths(geom, (state.world && state.world.dynamic) || []);
      drawObstacles(geom, state.obstacles || []);
      drawPaths(geom, state);
      drawCostmap(geom);
      drawTrail(geom);
      drawDock(geom, world.dock, state.charging);
      const pose = poseAt(now);
      drawVision(geom, pose, state.ray_scan || [], state.vision || {});
      drawRobot(geom, pose, now);
    }
    drawScaleBar(geom);
  }

  function drawBackground(geom) {
    const { width, height, world } = geom;
    ctx.fillStyle = palette.bg;
    ctx.fillRect(0, 0, width, height);

    const xMin = -geom.ox / geom.scale;
    const xMax = (width - geom.ox) / geom.scale;
    const yMin = -geom.oy / geom.scale;
    const yMax = (height - geom.oy) / geom.scale;
    ctx.strokeStyle = palette.grid;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(xMin * 2) / 2; x <= xMax; x += 0.5) {
      const sx = Math.round(geom.x(x)) + 0.5;
      ctx.moveTo(sx, 0);
      ctx.lineTo(sx, height);
    }
    for (let y = Math.ceil(yMin * 2) / 2; y <= yMax; y += 0.5) {
      const sy = Math.round(geom.y(y)) + 0.5;
      ctx.moveTo(0, sy);
      ctx.lineTo(width, sy);
    }
    ctx.stroke();

    const x0 = geom.x(0);
    const x1 = geom.x(world.width);
    const y0 = geom.y(world.height);
    const y1 = geom.y(0);
    const gradient = ctx.createLinearGradient(0, y0, 0, y1);
    gradient.addColorStop(0, palette.fieldTop);
    gradient.addColorStop(1, palette.fieldBottom);
    ctx.fillStyle = gradient;
    ctx.fillRect(x0, y0, x1 - x0, y1 - y0);

    ctx.strokeStyle = palette.fieldGrid;
    ctx.beginPath();
    for (let x = 0.5; x < world.width; x += 0.5) {
      const sx = Math.round(geom.x(x)) + 0.5;
      ctx.moveTo(sx, y0);
      ctx.lineTo(sx, y1);
    }
    for (let y = 0.5; y < world.height; y += 0.5) {
      const sy = Math.round(geom.y(y)) + 0.5;
      ctx.moveTo(x0, sy);
      ctx.lineTo(x1, sy);
    }
    ctx.stroke();

    ctx.strokeStyle = palette.fieldBorder;
    ctx.lineWidth = 2;
    ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
  }

  function offsetPolygon(points, distance) {
    // 顶点沿相邻两边外法线的角平分方向外移，用来把边界线画在可达区外侧。
    const count = points.length;
    let area = 0;
    for (let index = 0; index < count; index += 1) {
      const a = points[index];
      const b = points[(index + 1) % count];
      area += a[0] * b[1] - b[0] * a[1];
    }
    const sign = area >= 0 ? 1 : -1;
    const normals = [];
    for (let index = 0; index < count; index += 1) {
      const a = points[index];
      const b = points[(index + 1) % count];
      const dx = b[0] - a[0];
      const dy = b[1] - a[1];
      const length = Math.hypot(dx, dy) || 1;
      normals.push([(sign * dy) / length, (-sign * dx) / length]);
    }
    const outline = [];
    for (let index = 0; index < count; index += 1) {
      const prev = normals[(index - 1 + count) % count];
      const next = normals[index];
      let nx = prev[0] + next[0];
      let ny = prev[1] + next[1];
      const length = Math.hypot(nx, ny);
      if (length < 1e-6) {
        outline.push([
          points[index][0] + next[0] * distance,
          points[index][1] + next[1] * distance,
        ]);
        continue;
      }
      nx /= length;
      ny /= length;
      const cosHalf = Math.max(0.35, nx * next[0] + ny * next[1]);
      outline.push([
        points[index][0] + (nx * distance) / cosHalf,
        points[index][1] + (ny * distance) / cosHalf,
      ]);
    }
    return outline;
  }

  function drawGroundBoundary(geom, polygon) {
    if (!polygon || polygon.length < 3) return;
    ctx.save();
    ctx.beginPath();
    polygon.forEach((point, index) => {
      const x = geom.x(point[0]);
      const y = geom.y(point[1]);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.fillStyle = palette.groundFill;
    ctx.fill();
    ctx.restore();

    // 边界线画在可达区外侧：沿场地多边形向外偏移半个栅格余量。
    const outline = offsetPolygon(polygon, 0.12);
    if (outline.length < 3) return;
    ctx.save();
    ctx.beginPath();
    outline.forEach((point, index) => {
      const x = geom.x(point[0]);
      const y = geom.y(point[1]);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.lineJoin = "round";
    ctx.lineWidth = 1.6;
    ctx.strokeStyle = palette.groundStroke;
    ctx.stroke();
    ctx.restore();
  }

  function drawMap(geom, map) {
    if (!map || mapCanvas.width === 0) return;
    ctx.save();
    ctx.translate(geom.x(0), geom.y(0));
    ctx.scale(1, -1);
    ctx.imageSmoothingEnabled = false;
    const width = geom.x(geom.world.width) - geom.x(0);
    const height = geom.y(0) - geom.y(geom.world.height);
    ctx.drawImage(mapCanvas, 0, 0, width, height);
    ctx.restore();
  }

  function drawZones(geom, zones) {
    ctx.save();
    zones.forEach((zone) => {
      const outline = zone.outline || [];
      // 区域按真实栅格轮廓绘制，不再用外接矩形，避免相邻区域"看起来重叠"。
      ctx.setLineDash([4, 5]);
      ctx.lineWidth = zone.current ? 1.8 : 1;
      ctx.strokeStyle = zone.current
        ? "rgba(" + palette.green + ", 0.85)"
        : zone.selected
        ? "rgba(" + palette.cyan + ", 0.35)"
        : "rgba(" + palette.muted + ", 0.16)";
      if (outline.length >= 3) {
        ctx.beginPath();
        outline.forEach((point, index) => {
          const px = geom.x(point[0]);
          const py = geom.y(point[1]);
          if (index === 0) ctx.moveTo(px, py);
          else ctx.lineTo(px, py);
        });
        ctx.closePath();
        if (zone.current) {
          ctx.fillStyle = "rgba(" + palette.green + ", 0.05)";
          ctx.fill();
        }
        ctx.stroke();
      } else {
        const x = geom.x(zone.x0);
        const y = geom.y(zone.y1);
        const width = (zone.x1 - zone.x0) * geom.scale;
        const height = (zone.y1 - zone.y0) * geom.scale;
        if (zone.current) {
          ctx.fillStyle = "rgba(" + palette.green + ", 0.05)";
          ctx.fillRect(x, y, width, height);
        }
        ctx.strokeRect(x, y, width, height);
      }
      ctx.setLineDash([]);
      const labelX = geom.x(zone.cx != null ? zone.cx : (zone.x0 + zone.x1) / 2);
      const labelY = geom.y(zone.cy != null ? zone.cy : (zone.y0 + zone.y1) / 2);
      ctx.font = "600 " + Math.max(11, geom.scale * 0.18).toFixed(0) + "px 'Segoe UI', sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillStyle = zone.current
        ? "rgba(" + palette.green + ", 0.95)"
        : zone.selected
        ? "rgba(" + palette.cyan + ", 0.75)"
        : "rgba(" + palette.muted + ", 0.35)";
      ctx.fillText(String(zone.id), labelX, labelY - geom.scale * 0.12);
      ctx.font = Math.max(8, geom.scale * 0.10).toFixed(0) + "px 'Segoe UI', sans-serif";
      ctx.fillStyle = zone.selected
        ? "rgba(" + palette.green + ", 0.8)"
        : "rgba(" + palette.muted + ", 0.35)";
      ctx.fillText(Math.round(zone.coverage * 100) + "%", labelX, labelY + geom.scale * 0.14);
    });
    ctx.restore();
  }

  function drawGroundObstacles(geom, shapes) {
    ctx.save();
    shapes.forEach((shape) => {
      const outline = shape.outline || [];
      if (outline.length < 3) return;
      ctx.beginPath();
      outline.forEach((point, index) => {
        const x = geom.x(point[0]);
        const y = geom.y(point[1]);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.closePath();
      ctx.fillStyle = "rgba(255, 255, 255, 0.12)";
      ctx.fill();
      // 先描一层深色底，浅色主题下白色实体轮廓依然可见
      ctx.lineWidth = 3.8;
      ctx.strokeStyle = palette.outlineHalo;
      ctx.stroke();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = "rgba(255, 255, 255, 0.92)";
      ctx.stroke();
    });
    ctx.restore();
  }

  function noGoRect(zone) {
    if (zone.x0 != null) {
      return {
        x0: Math.min(zone.x0, zone.x1),
        y0: Math.min(zone.y0, zone.y1),
        x1: Math.max(zone.x0, zone.x1),
        y1: Math.max(zone.y0, zone.y1),
      };
    }
    return {
      x0: zone.x - zone.r,
      y0: zone.y - zone.r,
      x1: zone.x + zone.r,
      y1: zone.y + zone.r,
    };
  }

  function strokeNoGoRect(geom, rect, dashed, alpha) {
    const x = geom.x(rect.x0);
    const y = geom.y(rect.y1);
    const width = (rect.x1 - rect.x0) * geom.scale;
    const height = (rect.y1 - rect.y0) * geom.scale;
    ctx.beginPath();
    ctx.rect(x, y, width, height);
    ctx.fillStyle = "rgba(" + palette.red + ", " + (alpha || 0.1) + ")";
    ctx.fill();
    if (dashed) ctx.setLineDash([6, 4]);
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = "rgba(" + palette.red + ", 0.8)";
    ctx.stroke();
    ctx.setLineDash([]);
    // 斜线填充，和实体障碍区分
    ctx.save();
    ctx.beginPath();
    ctx.rect(x, y, width, height);
    ctx.clip();
    ctx.strokeStyle = "rgba(" + palette.red + ", 0.22)";
    ctx.lineWidth = 1;
    for (let offset = -height; offset < width; offset += 9) {
      ctx.beginPath();
      ctx.moveTo(x + offset, y + height);
      ctx.lineTo(x + offset + height, y);
      ctx.stroke();
    }
    ctx.restore();
    return { x: x + width / 2, y: y + height / 2 };
  }

  function drawNoGo(geom, zones) {
    if (!zones || !zones.length) return;
    ctx.save();
    zones.forEach((zone) => {
      const center = strokeNoGoRect(geom, noGoRect(zone), true, 0.1);
      ctx.fillStyle = "rgba(" + palette.red + ", 0.85)";
      ctx.font = "10px 'Segoe UI', sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText("禁区 #" + zone.id, center.x, center.y);
    });
    ctx.restore();
  }

  function drawEditLabels(geom, edits, dynamic) {
    const items = [];
    ((edits && edits.static) || []).forEach((item) => {
      items.push({ text: "#" + item.id, x: item.x, y: item.y + item.r + 0.20 });
    });
    (dynamic || []).forEach((item) => {
      items.push({ text: "#" + item.id, x: item.x, y: item.y + item.r + 0.20 });
    });
    if (!items.length) return;
    ctx.save();
    ctx.font = "600 10px 'Segoe UI', sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    items.forEach((item) => {
      const x = geom.x(item.x);
      const y = geom.y(item.y);
      ctx.lineWidth = 3;
      ctx.strokeStyle = palette.outlineHalo;
      ctx.strokeText(item.text, x, y);
      ctx.fillStyle = "rgba(255, 255, 255, 0.9)";
      ctx.fillText(item.text, x, y);
    });
    ctx.restore();
  }

  function drawPathEditPreview(geom) {
    if (!pathEditId) return;
    const dynamic = (current && current.world && current.world.dynamic) || [];
    const item = dynamic.find((entry) => entry.id === pathEditId);
    if (!item) return;
    const start = pathEditA || { x: item.ax, y: item.ay };
    const end = pathEditA && hoverWorld ? hoverWorld : { x: item.bx, y: item.by };
    ctx.save();
    ctx.beginPath();
    ctx.moveTo(geom.x(start.x), geom.y(start.y));
    ctx.lineTo(geom.x(end.x), geom.y(end.y));
    ctx.setLineDash([5, 5]);
    ctx.lineWidth = 1.8;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.9)";
    ctx.stroke();
    ctx.setLineDash([]);
    [
      { point: start, label: "A" },
      { point: end, label: "B" },
    ].forEach((entry) => {
      const x = geom.x(entry.point.x);
      const y = geom.y(entry.point.y);
      ctx.beginPath();
      ctx.arc(x, y, 4, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(255, 255, 255, 0.95)";
      ctx.fill();
      ctx.fillStyle = "rgba(255, 255, 255, 0.95)";
      ctx.font = "600 10px 'Segoe UI', sans-serif";
      ctx.textAlign = "center";
      ctx.fillText(entry.label, x, y - 8);
    });
    ctx.restore();
  }

  function drawRectDraft(geom) {
    if (!rectDraft) return;
    ctx.save();
    const center = strokeNoGoRect(geom, rectDraft, false, 0.16);
    const width = (rectDraft.x1 - rectDraft.x0).toFixed(2);
    const height = (rectDraft.y1 - rectDraft.y0).toFixed(2);
    ctx.fillStyle = "rgba(" + palette.red + ", 0.95)";
    ctx.font = "11px 'Segoe UI', sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(width + " × " + height + " m", center.x, center.y);
    ctx.restore();
  }

  function drawDynamicPaths(geom, items) {
    if (!items.length) return;
    ctx.save();
    items.forEach((item) => {
      if (item.ax == null) return;
      ctx.beginPath();
      ctx.moveTo(geom.x(item.ax), geom.y(item.ay));
      ctx.lineTo(geom.x(item.bx), geom.y(item.by));
      ctx.setLineDash([2, 6]);
      ctx.lineWidth = 1.2;
      ctx.strokeStyle = "rgba(255, 255, 255, 0.42)";
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.beginPath();
      ctx.arc(geom.x(item.x), geom.y(item.y), Math.max(3, item.r * geom.scale), 0, Math.PI * 2);
      ctx.lineWidth = 1.6;
      ctx.strokeStyle = "rgba(255, 255, 255, 0.92)";
      ctx.stroke();
    });
    ctx.restore();
  }

  function drawObstacles(geom, obstacles) {
    ctx.save();
    obstacles.forEach((obstacle) => {
      const large = obstacle.kind === "large";
      const color = large ? palette.obstacleLarge : palette.obstacleSmall;
      const cx = geom.x(obstacle.x);
      const cy = geom.y(obstacle.y);
      const radius = Math.max(4, obstacle.r * geom.scale);
      ctx.beginPath();
      ctx.arc(cx, cy, radius, 0, Math.PI * 2);
      ctx.setLineDash([5, 4]);
      ctx.lineWidth = 1.6;
      ctx.strokeStyle = "rgba(" + color + ", 0.95)";
      ctx.fillStyle = "rgba(" + color + ", 0.10)";
      ctx.fill();
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "rgba(" + color + ", 0.95)";
      ctx.font = "10px 'Segoe UI', sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(large ? "大" : "小", cx, cy);
    });
    ctx.restore();
  }

  function drawPaths(geom, state) {
    const task = state.task || {};
    if (task.type !== "mow" && task.type !== "map") return;

    if (state.edge_route && state.edge_route.length > 1) {
      ctx.save();
      ctx.strokeStyle = "rgba(" + palette.cyan + ", 0.55)";
      ctx.lineWidth = 1.4;
      ctx.setLineDash([7, 5]);
      ctx.beginPath();
      state.edge_route.forEach((point, index) => {
        const x = geom.x(point[0]);
        const y = geom.y(point[1]);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
      ctx.restore();
    }

    if (state.segments && state.segments.length) {
      ctx.save();
      ctx.strokeStyle = "rgba(" + palette.route + ", 0.75)";
      ctx.lineWidth = 1.3;
      ctx.setLineDash([3, 3]);
      state.segments.forEach((segment) => {
        ctx.beginPath();
        ctx.moveTo(geom.x(segment[0][0]), geom.y(segment[0][1]));
        ctx.lineTo(geom.x(segment[1][0]), geom.y(segment[1][1]));
        ctx.stroke();
      });
      ctx.restore();
    }

    if (state.route && state.route.length > 1) {
      ctx.save();
      ctx.strokeStyle = "rgba(" + palette.cyan + ", 0.9)";
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      state.route.forEach((point, index) => {
        const x = geom.x(point[0]);
        const y = geom.y(point[1]);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
      ctx.restore();
    }

    if (state.goal) {
      const x = geom.x(state.goal[0]);
      const y = geom.y(state.goal[1]);
      ctx.save();
      ctx.strokeStyle = "rgba(" + palette.route + ", 0.9)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x - 5, y);
      ctx.lineTo(x + 5, y);
      ctx.moveTo(x, y - 5);
      ctx.lineTo(x, y + 5);
      ctx.stroke();
      ctx.restore();
    }
  }

  const costCanvas = document.createElement("canvas");
  let costmapLayer = null;
  let costmapVersion = 0;

  function updateCostmapLayer(layer) {
    if (!layer || !layer.cells || !layer.cells.length) {
      costmapLayer = null;
      return;
    }
    const width = layer.width;
    const height = layer.height;
    costCanvas.width = width;
    costCanvas.height = height;
    const cctx = costCanvas.getContext("2d");
    const image = cctx.createImageData(width, height);
    const data = image.data;
    for (let row = 0; row < height; row += 1) {
      const line = layer.cells[row];
      for (let col = 0; col < width; col += 1) {
        const digit = line.charCodeAt(col) - 48;
        if (digit <= 0) continue;
        const offset = (row * width + col) * 4;
        if (digit >= 9) {
          data[offset] = 255;
          data[offset + 1] = 96;
          data[offset + 2] = 72;
          data[offset + 3] = 210;
        } else if (digit >= 5) {
          data[offset] = 255;
          data[offset + 1] = 176;
          data[offset + 2] = 46;
          data[offset + 3] = 150;
        } else {
          data[offset] = 79;
          data[offset + 1] = 195;
          data[offset + 2] = 255;
          data[offset + 3] = 70;
        }
      }
    }
    cctx.putImageData(image, 0, 0);
    layer.flipRows = true;
    costmapLayer = layer;
    costmapVersion += 1;
  }

  function drawCostmap(geom) {
    if (layerVisible.costmap === false) return;
    if (!costmapLayer) return;
    const width = costmapLayer.width;
    const height = costmapLayer.height;
    const resolution = costmapLayer.resolution;
    if (!width || !height || !resolution) return;
    // 栅格行 0 是最小 y：画布 y 轴向下，需要用负高度翻转绘制
    const left = geom.x(costmapLayer.origin[0]);
    const top = geom.y(costmapLayer.origin[1] + height * resolution);
    ctx.save();
    ctx.globalAlpha = 0.6;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(
      costCanvas,
      left,
      top,
      width * resolution * geom.scale,
      height * resolution * geom.scale
    );
    ctx.restore();
  }

  // 日志自带地图说明当时是在割草作业，实际轨迹画粗一点
  function bagPathBold() {
    return !!(bagState && bagState.loaded && bagState.maps && bagState.maps.whole_map);
  }

  function drawTrail(geom) {
    if (layerVisible.pose === false) return;
    if (trail.length < 2) return;
    const bold = bagPathBold();
    const normal = bold ? Math.max(2.2, geom.scale * 0.05) : Math.max(1.4, geom.scale * 0.03);
    // 跳线（回放中重定位）：红色细线
    const jumpWidth = Math.max(1.0, geom.scale * 0.008);
    ctx.save();
    ctx.lineCap = "round";
    for (let index = 1; index < trail.length; index += 1) {
      const point = trail[index];
      const alpha = point.jump
        ? 0.7
        : bold
        ? 0.08 + 0.4 * (index / trail.length)
        : 0.05 + 0.3 * (index / trail.length);
      ctx.strokeStyle =
        "rgba(" + (point.jump ? palette.red : palette.cyan) + ", " + alpha.toFixed(3) + ")";
      ctx.lineWidth = point.jump ? jumpWidth : normal;
      ctx.beginPath();
      ctx.moveTo(geom.x(trail[index - 1].x), geom.y(trail[index - 1].y));
      ctx.lineTo(geom.x(point.x), geom.y(point.y));
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawDock(geom, dock, charging) {
    if (!dock) return;
    const cx = geom.x(dock.x);
    const cy = geom.y(dock.y);
    const r = Math.max(12, dock.r * geom.scale);
    const pulse = 0.5 + 0.5 * Math.sin(performance.now() / 420);
    ctx.save();
    if (charging) {
      const glow = ctx.createRadialGradient(cx, cy, r * 0.4, cx, cy, r * 2.4);
      glow.addColorStop(0, "rgba(" + palette.green + ", " + (0.22 + pulse * 0.18).toFixed(3) + ")");
      glow.addColorStop(1, "rgba(" + palette.green + ", 0)");
      ctx.fillStyle = glow;
      ctx.beginPath();
      ctx.arc(cx, cy, r * 2.4, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(" + palette.green + ", 0.10)";
    ctx.fill();
    ctx.setLineDash([5, 5]);
    ctx.lineDashOffset = -performance.now() / 90;
    ctx.strokeStyle = "rgba(" + palette.green + ", " + (charging ? 0.95 : 0.7) + ")";
    ctx.lineWidth = 1.6;
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = charging ? palette.dockIcon : "rgba(" + palette.green + ", 0.92)";
    ctx.font = "bold " + Math.max(13, r * 0.8).toFixed(0) + "px 'Segoe UI', sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("⚡", cx, cy);
    ctx.font = Math.max(9, r * 0.3).toFixed(0) + "px 'PingFang SC', sans-serif";
    ctx.fillStyle = "rgba(" + palette.green + ", 0.9)";
    ctx.fillText("充电桩", cx, cy + r * 1.35);
    ctx.restore();
  }

  function drawVision(geom, pose, rays, vision) {
    const originX = geom.x(pose.x);
    const originY = geom.y(pose.y);
    const range = vision.range || 1.0;
    const points = [];
    rays.forEach(([degrees, distance]) => {
      const angle = pose.theta + (degrees * Math.PI) / 180;
      points.push([
        geom.x(pose.x + Math.cos(angle) * distance),
        geom.y(pose.y + Math.sin(angle) * distance),
      ]);
    });
    if (!points.length) return;

    ctx.save();
    ctx.beginPath();
    ctx.moveTo(originX, originY);
    points.forEach((point) => ctx.lineTo(point[0], point[1]));
    ctx.closePath();
    ctx.fillStyle = "rgba(" + palette.cyan + ", 0.09)";
    ctx.fill();
    ctx.strokeStyle = "rgba(" + palette.cyan + ", 0.30)";
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.strokeStyle = "rgba(" + palette.cyan + ", 0.16)";
    ctx.lineWidth = 0.8;
    ctx.beginPath();
    for (let index = 0; index < points.length; index += 4) {
      ctx.moveTo(originX, originY);
      ctx.lineTo(points[index][0], points[index][1]);
    }
    ctx.stroke();
    ctx.restore();
  }

  function roundRect(context, x, y, width, height, radius) {
    const r = Math.min(radius, width / 2, height / 2);
    context.beginPath();
    context.moveTo(x + r, y);
    context.arcTo(x + width, y, x + width, y + height, r);
    context.arcTo(x + width, y + height, x, y + height, r);
    context.arcTo(x, y + height, x, y, r);
    context.arcTo(x, y, x + width, y, r);
    context.closePath();
  }

  function drawRobot(geom, pose, now) {
    const cx = geom.x(pose.x);
    const cy = geom.y(pose.y);
    const radius = Math.max(8, ROBOT_RADIUS * geom.scale);
    const flashing = now < bumpFlashUntil;

    ctx.save();
    const glow = ctx.createRadialGradient(cx, cy, radius * 0.3, cx, cy, radius * 2.4);
    glow.addColorStop(0, "rgba(" + palette.cyan + ", 0.30)");
    glow.addColorStop(1, "rgba(" + palette.cyan + ", 0)");
    ctx.fillStyle = glow;
    ctx.beginPath();
    ctx.arc(cx, cy, radius * 2.4, 0, Math.PI * 2);
    ctx.fill();

    ctx.translate(cx, cy);
    ctx.rotate(-pose.theta);

    // 四轮：后轮驱动（青色），前轮随动（灰色）
    const track = radius * 1.15;
    const axle = radius * 0.85;
    [
      [-axle, -track, true],
      [-axle, track, true],
      [axle, -track, false],
      [axle, track, false],
    ].forEach(([wx, wy, driven]) => {
      roundRect(ctx, wx - radius * 0.30, wy - radius * 0.20, radius * 0.60, radius * 0.40, radius * 0.12);
      ctx.fillStyle = driven ? "#0e3f52" : "#121a2b";
      ctx.fill();
      ctx.strokeStyle = driven
        ? "rgba(" + palette.cyan + ", 0.85)"
        : "rgba(" + palette.wheelIdle + ", 0.45)";
      ctx.lineWidth = 1.2;
      ctx.stroke();
    });

    const body = ctx.createRadialGradient(-radius * 0.3, -radius * 0.3, radius * 0.12, 0, 0, radius);
    body.addColorStop(0, "#a8efff");
    body.addColorStop(0.55, "#3cb6e8");
    body.addColorStop(1, "#11587f");
    ctx.fillStyle = body;
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = flashing ? "rgba(" + palette.red + ", 0.95)" : "rgba(" + palette.rim + ", 0.85)";
    ctx.lineWidth = flashing ? 3 : 1.6;
    ctx.stroke();

    // 割草盘
    ctx.beginPath();
    ctx.arc(0, 0, radius * 0.62, 0, Math.PI * 2);
    ctx.strokeStyle = "rgba(" + palette.green + ", 0.55)";
    ctx.lineWidth = 1.2;
    ctx.stroke();

    ctx.fillStyle = "#04121c";
    ctx.beginPath();
    ctx.moveTo(radius * 0.74, 0);
    ctx.lineTo(radius * 0.08, -radius * 0.32);
    ctx.lineTo(radius * 0.08, radius * 0.32);
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = "rgba(255, 246, 196, 0.95)";
    ctx.beginPath();
    ctx.arc(radius * 0.72, 0, radius * 0.09, 0, Math.PI * 2);
    ctx.fill();

    if (flashing) {
      ctx.strokeStyle = "rgba(" + palette.red + ", 0.55)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(0, 0, radius * 1.55, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawScaleBar(geom) {
    const x = geom.x(geom.world.width - 1.35);
    const y = geom.y(0.28);
    const length = geom.scale;
    ctx.strokeStyle = "rgba(" + palette.muted + ", 0.6)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + length, y);
    ctx.moveTo(x, y - 4);
    ctx.lineTo(x, y + 4);
    ctx.moveTo(x + length, y - 4);
    ctx.lineTo(x + length, y + 4);
    ctx.stroke();
    ctx.fillStyle = "rgba(" + palette.muted + ", 0.85)";
    ctx.font = "11px 'Segoe UI', sans-serif";
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    ctx.fillText("1 m", x + length + 8, y);
  }

  // ------------------------------------------------------------- 交互
  const KEY_COMMANDS = {
    ArrowUp: "forward",
    ArrowDown: "backward",
    ArrowLeft: "left",
    ArrowRight: "right",
    w: "forward",
    s: "backward",
    a: "left",
    d: "right",
  };
  const heldKeys = new Set();

  function keyName(event) {
    return event.key.length === 1 ? event.key.toLowerCase() : event.key;
  }

  document.querySelectorAll("[data-command]").forEach((button) => {
    const command = button.dataset.command;
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      button.classList.add("pressed");
      send("/api/command", { command });
    });
    const release = () => {
      button.classList.remove("pressed");
      if (command !== "stop") send("/api/command", { command: "stop" });
    };
    button.addEventListener("pointerup", release);
    button.addEventListener("pointercancel", release);
    button.addEventListener("pointerleave", (event) => {
      if (event.buttons) release();
    });
  });

  document.addEventListener("keydown", (event) => {
    const key = keyName(event);
    if (key === " ") {
      event.preventDefault();
      send("/api/command", { command: "stop" });
      return;
    }
    if (key === "r") {
      event.preventDefault();
      resetAll();
      return;
    }
    const command = KEY_COMMANDS[key];
    if (!command) return;
    event.preventDefault();
    if (heldKeys.has(key)) return;
    heldKeys.add(key);
    buttonForCommand(command, true);
    send("/api/command", { command });
  });

  document.addEventListener("keyup", (event) => {
    const key = keyName(event);
    const command = KEY_COMMANDS[key];
    if (!command || !heldKeys.has(key)) return;
    heldKeys.delete(key);
    buttonForCommand(command, false);
    send("/api/command", { command: "stop" });
  });

  function buttonForCommand(command, pressed) {
    const button = document.querySelector('[data-command="' + command + '"]');
    if (button) button.classList.toggle("pressed", pressed);
  }

  window.addEventListener("blur", () => {
    if (heldKeys.size) {
      heldKeys.clear();
      document.querySelectorAll(".pad-btn.pressed").forEach((button) => {
        button.classList.remove("pressed");
      });
      send("/api/command", { command: "stop" });
    }
  });

  els.mapBtn.addEventListener("click", () => send("/api/task", { task: "map" }));

  els.mowBtn.addEventListener("click", () => {
    const zones = els.zoneOrder.value
      .split(/[,，\s]+/)
      .map((value) => parseInt(value, 10))
      .filter((value) => !Number.isNaN(value));
    if (!current || !current.zones || !current.zones.length) {
      els.missionMessage.textContent = "请先完成自动建图，生成割草区域";
      return;
    }
    send("/api/task", { task: "mow", zones, mode: els.mowMode.value });
  });

  els.deleteMapBtn.addEventListener("click", async () => {
    const selected = current && current.world && current.world.maps
      ? current.world.maps.find(
          (item) => String(item.id) === String(els.mapSelect.value)
        )
      : null;
    if (!selected || !selected.imported) return;
    if (!window.confirm("删除导入地图“" + selected.name + "”？")) return;
    try {
      const state = await request("/api/map/delete", {
        map: Number(selected.id),
      });
      trail = [];
      mapVersion = -1;
      viewWorldKey = "";
      zoneAutoValue = "";
      zoneInputDirty = false;
      els.zoneOrder.value = "";
      ingest(state);
      els.importMapMessage.textContent =
        "已删除导入地图 " + selected.name + "，当前切换回 1 号内置地图";
    } catch (error) {
      els.importMapMessage.textContent = "删除失败：" + error.message;
    }
  });

  els.mapSelect.addEventListener("change", () => {
    trail = [];
    mapVersion = -1;
    viewWorldKey = "";
    zoneAutoValue = "";
    zoneInputDirty = false;
    updateDeleteButton();
    send("/api/map", { map: Number(els.mapSelect.value) });
  });

  function applyImportedMap(state) {
    trail = [];
    mapVersion = -1;
    viewWorldKey = "";
    zoneAutoValue = "";
    zoneInputDirty = false;
    els.zoneOrder.value = "";
    ingest(state);
    const meta = (state.world && state.world.import_meta) || {};
    els.importMapMessage.textContent =
      "已导入 " +
      state.map_name +
      " · " +
      (meta.free_area_m2 != null ? meta.free_area_m2 + " m² 自由区" : "地图就绪") +
      " · " +
      (meta.obstacle_count != null ? meta.obstacle_count + " 个内部障碍" : "");
  }

  function setImportBusy(busy, message) {
    [els.importFileBtn, els.importDirBtn].forEach((button) => {
      button.disabled = busy;
    });
    if (message) els.importMapMessage.textContent = message;
  }

  function allowedMapFile(name) {
    return /\.(pgm|json|ya?ml)$/i.test(name);
  }

  function uploadRelativePath(file) {
    const relative = (file.webkitRelativePath || "").replace(/\\/g, "/");
    if (relative) return relative;
    if (file.name.toLowerCase() === "default.json") return "path/default.json";
    return file.name;
  }

  function readFileBase64(file) {
    return new Promise((resolve, reject) => {
      const reader = new FileReader();
      reader.onload = () => {
        const result = String(reader.result || "");
        const comma = result.indexOf(",");
        resolve(comma >= 0 ? result.slice(comma + 1) : result);
      };
      reader.onerror = () => reject(new Error("无法读取文件 " + file.name));
      reader.readAsDataURL(file);
    });
  }

  async function uploadMapFiles(fileList) {
    const selected = Array.from(fileList || []).filter((file) =>
      allowedMapFile(file.name)
    );
    if (!selected.length) {
      throw new Error("请选择 .pgm 文件，或包含 .pgm/.json 的地图目录");
    }
    const entries = [];
    let total = 0;
    for (const file of selected) {
      if (file.size > 16 * 1024 * 1024) {
        throw new Error(file.name + " 超过 16 MB");
      }
      total += file.size;
      if (total > 24 * 1024 * 1024) {
        throw new Error("所选文件总大小超过 24 MB");
      }
      entries.push({
        path: uploadRelativePath(file),
        data: await readFileBase64(file),
      });
    }
    if (!entries.some((entry) => /\.pgm$/i.test(entry.path))) {
      throw new Error("缺少 .pgm 栅格地图文件");
    }
    return request("/api/map/upload", { files: entries });
  }

  async function chooseMapFiles(fileList) {
    setImportBusy(true, "正在读取文件并上传…");
    try {
      const state = await uploadMapFiles(fileList);
      applyImportedMap(state);
    } catch (error) {
      els.importMapMessage.textContent = "导入失败：" + error.message;
    } finally {
      setImportBusy(false);
    }
  }

  els.importFileBtn.addEventListener("click", () => els.importFileInput.click());
  els.importDirBtn.addEventListener("click", () => els.importDirInput.click());
  els.importFileInput.addEventListener("change", () => {
    const files = Array.from(els.importFileInput.files || []);
    els.importFileInput.value = "";
    chooseMapFiles(files);
  });
  els.importDirInput.addEventListener("change", () => {
    const files = Array.from(els.importDirInput.files || []);
    els.importDirInput.value = "";
    chooseMapFiles(files);
  });

  els.zoneOrder.addEventListener("input", () => {
    zoneInputDirty = true;
  });

  els.mowMode.addEventListener("change", () => {
    modeTouched = true;
  });

  els.chargeBtn.addEventListener("click", () => send("/api/charge", {}));
  els.resumeBtn.addEventListener("click", () => send("/api/resume", {}));
  els.stopBtn.addEventListener("click", () => {
    trail = [];
    send("/api/stop", {});
  });

  els.speedButtons.forEach((button) => {
    button.addEventListener("click", () => {
      send("/api/speed", { speed: Number(button.dataset.speed) });
    });
  });

  function resetAll() {
    trail = [];
    lastSampleTime = -1;
    mapVersion = -1;
    viewWorldKey = "";
    zoneAutoValue = "";
    zoneInputDirty = false;
    els.zoneOrder.value = "";
    send("/api/reset", {});
  }

  els.resetBtn.addEventListener("click", resetAll);

  // 退出：关闭后端服务并释放端口，界面切到退出提示
  // 浏览器只允许脚本关闭“由脚本打开”的窗口；地址栏直接打开的标签页会被拒绝，
  // 这时退化为“控制台已退出”的整屏提示 + 快捷键说明。
  function tryClosePage() {
    try {
      window.close();
    } catch (error) {
      /* 被浏览器拒绝时继续走提示分支 */
    }
    window.setTimeout(() => {
      if (!window.closed) els.exitHint.hidden = false;
    }, 350);
  }

  async function shutdownConsole() {
    const running = current && current.task && current.task.type;
    const warning = running
      ? "当前任务会被停止（不会保存断点），确定退出并关闭服务吗？"
      : "确定退出控制台并关闭后端服务吗？";
    if (!window.confirm(warning)) return;
    shuttingDown = true;
    let message = "后端服务已关闭，端口已释放。";
    try {
      await request("/api/shutdown", {});
    } catch (error) {
      message = "服务已关闭（连接已断开），端口已释放。";
    }
    document.title = "已退出 · 机器人控制台";
    els.exitTip.textContent = message;
    els.exitOverlay.hidden = false;
    tryClosePage();
  }

  els.exitBtn.addEventListener("click", shutdownConsole);
  els.exitCloseBtn.addEventListener("click", tryClosePage);
  els.estopBtn.addEventListener("click", () => send("/api/command", { command: "stop" }));

  // ------------------------------------------------------------- 左侧栏收起
  const SIDE_KEY = "mowerSideHiddenV1";
  const layoutEl = document.querySelector(".layout");
  let sideHidden = false;
  try {
    sideHidden = localStorage.getItem(SIDE_KEY) === "1";
  } catch (error) {
    sideHidden = false;
  }

  function applySidebarHidden(hidden) {
    if (!layoutEl) return;
    layoutEl.classList.toggle("is-side-hidden", !!hidden);
    if (els.sidebarToggleBtn) {
      els.sidebarToggleBtn.setAttribute("aria-pressed", hidden ? "true" : "false");
      els.sidebarToggleBtn.textContent = hidden ? "◨ 侧栏" : "◧ 侧栏";
    }
    if (els.sidebarTab) els.sidebarTab.hidden = !hidden;
    try {
      localStorage.setItem(SIDE_KEY, hidden ? "1" : "0");
    } catch (error) {
      /* ignore */
    }
  }

  applySidebarHidden(sideHidden);
  if (els.popupBackBtn) els.popupBackBtn.addEventListener("click", recallPopups);
  function toggleSidebar() {
    sideHidden = !sideHidden;
    applySidebarHidden(sideHidden);
  }
  if (els.sidebarToggleBtn) els.sidebarToggleBtn.addEventListener("click", toggleSidebar);
  if (els.sidebarHandle) els.sidebarHandle.addEventListener("click", toggleSidebar);
  if (els.sidebarTab) els.sidebarTab.addEventListener("click", toggleSidebar);

  // ------------------------------------------------------------- 任务操作收起
  const MISSION_KEY = "mowerMissionFoldedV1";
  let missionFolded = false;
  try {
    missionFolded = localStorage.getItem(MISSION_KEY) === "1";
  } catch (error) {
    missionFolded = false;
  }

  function applyMissionFolded(folded) {
    if (els.missionPanel) {
      els.missionPanel.classList.toggle("is-folded", !!folded);
    }
    if (els.missionFoldBtn) {
      els.missionFoldBtn.textContent = folded ? "展开" : "收起";
      els.missionFoldBtn.setAttribute("aria-expanded", folded ? "false" : "true");
    }
    try {
      localStorage.setItem(MISSION_KEY, folded ? "1" : "0");
    } catch (error) {
      /* ignore */
    }
  }

  applyMissionFolded(missionFolded);
  if (els.missionFoldBtn) {
    els.missionFoldBtn.addEventListener("click", () => {
      missionFolded = !missionFolded;
      applyMissionFolded(missionFolded);
    });
  }

  // ------------------------------------------------------------- 独立窗口（弹窗）
  const POPUP_CSS = [
    "html,body{margin:0;height:100%;background:var(--bg);color:var(--text);",
    "font-family:'Segoe UI','PingFang SC','Microsoft YaHei',system-ui,sans-serif;font-size:12px}",
    ".popup-host{height:100%;box-sizing:border-box}",
  ].join("");
  const popups = {};
  const bagPlayerEl = document.querySelector(".bag-player");

  function popupWindow(key, node, options) {
    const opts = options || {};
    const known = popups[key];
    if (known && !known.win.closed) {
      known.win.focus();
      return known.win;
    }
    const size =
      "width=" + (opts.width || 560) + ",height=" + (opts.height || 420) +
      ",menubar=no,toolbar=no,location=no,status=no,alwaysRaised=yes";
    const win = window.open("", "mower-" + key, size);
    if (!win) {
      setBagHint("浏览器拦截了弹出窗口，请允许弹窗后重试", 6000);
      return null;
    }
    const doc = win.document;
    doc.open();
    doc.write(
      "<!DOCTYPE html><html><head><meta charset='utf-8'><title>" +
        (opts.title || "机器人控制台") +
        "</title></head><body></body></html>"
    );
    doc.close();
    doc.documentElement.dataset.theme = document.documentElement.dataset.theme || "dark";
    const link = doc.createElement("link");
    link.rel = "stylesheet";
    link.href = location.origin + "/styles.css";
    doc.head.appendChild(link);
    const style = doc.createElement("style");
    style.textContent = POPUP_CSS;
    doc.head.appendChild(style);
    const host = doc.createElement("div");
    host.className = "popup-host";
    doc.body.appendChild(host);
    const parent = node.parentNode;
    const next = node.nextSibling;
    node.classList.add("is-popped");
    host.appendChild(node);
    popups[key] = { win, node, parent, next };
    // 只在弹窗真的被关掉时收回节点（失焦 / 切到主窗口时不能消失）
    refreshPopupBack();
    try {
      win.focus();
    } catch (error) {
      /* 某些浏览器不允许脚本抢焦点，忽略 */
    }
    return win;
  }

  function restorePopup(key) {
    const record = popups[key];
    if (!record) return;
    delete popups[key];
    if (!record.win.closed) {
      try {
        record.win.close();
      } catch (error) {
        /* 关闭失败时只把节点搬回来 */
      }
    }
    record.node.classList.remove("is-popped");
    if (record.parent && record.parent.isConnected && record.node.parentNode !== record.parent) {
      const anchor = record.next && record.next.parentNode === record.parent ? record.next : null;
      record.parent.insertBefore(record.node, anchor);
    }
    refreshPopupBack();
  }

  function refreshPopupBack() {
    if (!els.popupBackBtn) return;
    const open = Object.keys(popups).some((key) => !popups[key].win.closed);
    els.popupBackBtn.hidden = !open;
  }

  function recallPopups() {
    Object.keys(popups).forEach((key) => restorePopup(key));
  }

  function syncPopups() {
    Object.keys(popups).forEach((key) => {
      if (popups[key].win.closed) restorePopup(key);
    });
  }

  // ------------------------------------------------------------- 日志播放
  let bagState = null;
  let bagPath = [];
  let bagDragging = false;
  let bagEntries = [];
  let bagGroups = [];
  let bagVisible = [];
  let bagCurrentKey = "";
  let bagChecked = new Set();
  let bagOrder = [];

  function setLayerVisible(layer, visible) {
    layerVisible[layer] = !!visible;
    try {
      localStorage.setItem(LAYER_KEY, JSON.stringify(layerVisible));
    } catch (error) {
      /* ignore */
    }
    if (layer === "image") {
      bagVideoClosed = !visible;
      if (visible && bagState && bagState.loaded) refreshBagImage(true);
      updateBagVideo(bagState);
    }
  }

  let bagQueue = null;
  let bagImageState = { index: -1, key: "", busy: false };
  let bagImageTimer = 0;
  let bagLiveName = "";
  let bagVideoClosed = false;

  // 提示语：操作类提示短暂优先显示，避免被每秒两次的状态刷新覆盖
  let bagHintUntil = 0;

  function setBagHint(text, stickyMs) {
    const now = performance.now();
    if (!stickyMs && now < bagHintUntil) return;
    if (stickyMs) bagHintUntil = now + stickyMs;
    els.bagHint.textContent = text;
  }

  // 日志编号：zmqbag_15_2026-09-15-... → 15，用于“从大到小”排序
  function bagNumber(bag) {
    const name = (bag && bag.name) || "";
    const matched = name.match(/(\d+)/g);
    if (!matched) return -1;
    return parseInt(matched[0], 10);
  }

  function sortBagsDesc(bags) {
    return bags
      .slice()
      .sort((a, b) => {
        const diff = bagNumber(b) - bagNumber(a);
        if (diff !== 0) return diff;
        return (b.mtime || 0) - (a.mtime || 0);
      });
  }

  function selectAllVisible() {
    const ordered = sortBagsDesc(bagVisible);
    bagChecked = new Set(ordered.map((bag) => bag.key));
    bagOrder = ordered.map((bag) => bag.key);
    renderBagList();
    renderBagQueue(bagQueue);
  }

  function clearBagSelection() {
    bagChecked = new Set();
    bagOrder = [];
    renderBagList();
    renderBagQueue(bagQueue);
  }

  function formatSize(bytes) {
    return ((bytes || 0) / 1048576).toFixed(1) + " MB";
  }

  function renderBagFolders() {
    if (!els.bagFolder) return;
    const current = els.bagFolder.value;
    els.bagFolder.textContent = "";
    const all = document.createElement("option");
    all.value = "";
    all.textContent = "全部（" + bagEntries.length + " 个）";
    els.bagFolder.append(all);
    bagGroups.forEach((group) => {
      const option = document.createElement("option");
      option.value = group;
      const count = bagEntries.filter((bag) => bag.group === group).length;
      option.textContent = group + "（" + count + "）";
      els.bagFolder.append(option);
    });
    els.bagFolder.value = bagGroups.includes(current) ? current : "";
  }

  function renderBagList() {
    if (!els.bagList) return;
    const group = els.bagFolder ? els.bagFolder.value : "";
    bagVisible = sortBagsDesc(bagEntries.filter((bag) => !group || bag.group === group));
    els.bagList.textContent = "";
    if (!bagVisible.length) {
      const empty = document.createElement("p");
      empty.className = "bag-hint";
      empty.textContent = "这个文件夹里没有 .bag 日志，可以导入其他文件夹";
      els.bagList.append(empty);
      return;
    }
    bagVisible.forEach((bag) => {
      const row = document.createElement("div");
      row.className = "bag-item" + (bag.key === bagCurrentKey ? " is-active" : "");
      row.title = bag.path;
      const check = document.createElement("input");
      check.type = "checkbox";
      check.checked = bagChecked.has(bag.key);
      check.addEventListener("change", () => {
        if (check.checked) {
          bagChecked.add(bag.key);
          if (!bagOrder.includes(bag.key)) bagOrder.push(bag.key);
        } else {
          bagChecked.delete(bag.key);
          bagOrder = bagOrder.filter((key) => key !== bag.key);
        }
        renderBagList();
        renderBagQueue(bagQueue);
      });
      const orderIndex = bagOrder.indexOf(bag.key);
      const name = document.createElement("span");
      name.className = "bag-item-name";
      name.textContent = bag.name;
      row.append(check);
      if (orderIndex >= 0) {
        const order = document.createElement("span");
        order.className = "bag-item-order";
        order.textContent = String(orderIndex + 1);
        row.append(order);
      }
      row.append(name);
      const meta = document.createElement("em");
      meta.textContent = formatSize(bag.size);
      row.append(meta);
      if (orderIndex >= 0) {
        const tools = document.createElement("span");
        tools.className = "bag-item-move";
        const up = document.createElement("button");
        up.type = "button";
        up.textContent = "▲";
        up.title = "提前播放";
        up.disabled = orderIndex === 0;
        up.addEventListener("click", (event) => {
          event.stopPropagation();
          moveBagOrder(bag.key, -1);
        });
        const down = document.createElement("button");
        down.type = "button";
        down.textContent = "▼";
        down.title = "延后播放";
        down.disabled = orderIndex === bagOrder.length - 1;
        down.addEventListener("click", (event) => {
          event.stopPropagation();
          moveBagOrder(bag.key, 1);
        });
        tools.append(up, down);
        row.append(tools);
      }
      row.addEventListener("click", (event) => {
        if (event.target === check || event.target.closest(".bag-item-move")) return;
        bagCurrentKey = bag.key;
        renderBagList();
      });
      row.addEventListener("dblclick", () => {
        bagCurrentKey = bag.key;
        renderBagList();
        loadBag();
      });
      els.bagList.append(row);
    });
    if (els.bagSelectAll) {
      const allChecked =
        bagVisible.length > 0 && bagVisible.every((bag) => bagChecked.has(bag.key));
      els.bagSelectAll.checked = allChecked;
    }
    if (els.bagOrderHint) {
      els.bagOrderHint.textContent = bagOrder.length
        ? "已按编号从大到小排好，可 ▲▼ 微调"
        : "按日志编号从大到小";
    }
  }

  function moveBagOrder(key, delta) {
    const from = bagOrder.indexOf(key);
    const to = from + delta;
    if (from < 0 || to < 0 || to >= bagOrder.length) return;
    bagOrder.splice(to, 0, bagOrder.splice(from, 1)[0]);
    renderBagList();
    renderBagQueue(bagQueue);
  }

  const LAYER_LABELS = {
    pose: "仿真实际轨迹（跳变处画细线）",
    costmap: "日志代价地图叠加（机器人周围的红/橙方块就是它）",
    static_costmap: "静态代价地图（当前未叠加显示）",
    image: "日志图像窗口",
    map: "日志真实地图（已作为仿真场地，不能隐藏）",
  };

  function renderBagTopics(topics) {
    if (!els.bagTopicList) return;
    els.bagTopicList.textContent = "";
    if (els.bagTopicCount) els.bagTopicCount.textContent = String((topics || []).length);
    (topics || []).forEach((topic) => {
      const layer = topic.layer;
      const row = document.createElement("label");
      row.className = "bag-topic";
      const check = document.createElement("input");
      check.type = "checkbox";
      if (layer) {
        check.checked = layerVisible[layer] !== false;
        row.title = "勾选=显示，“" + topic.name + "”对应：" + (LAYER_LABELS[layer] || layer);
        check.addEventListener("change", () => {
          setLayerVisible(layer, check.checked);
          renderBagTopics(topics);
        });
        if (layer === "map" || layer === "static_costmap") {
          row.classList.add("bag-topic--static");
          check.disabled = true;
        }
      } else {
        check.checked = true;
        check.disabled = true;
        row.title = "该话题只参与回放数据，不单独绘制图层";
      }
      const name = document.createElement("span");
      name.className = "bag-topic-name";
      name.textContent = topic.name;
      const meta = document.createElement("em");
      meta.textContent = topic.count + " · " + topic.hz + " Hz";
      row.append(check, name, meta);
      if (layer && layerVisible[layer] === false) row.classList.add("bag-topic--off");
      els.bagTopicList.append(row);
    });
    if (!topics || !topics.length) {
      const empty = document.createElement("p");
      empty.className = "bag-hint";
      empty.textContent = "载入日志后显示话题列表";
      els.bagTopicList.append(empty);
    }
  }

  function renderBagQueue(queue) {
    bagQueue = queue || null;
    if (!els.bagQueueInfo) return;
    const total = queue && queue.total ? queue.total : 0;
    const index = queue && queue.total ? queue.index + 1 : 0;
    if (!total) {
      els.bagQueueInfo.textContent = bagOrder.length
        ? "顺序 " + bagOrder.length + " 个"
        : "队列 --";
      return;
    }
    els.bagQueueInfo.textContent = (queue.active ? "连续播放 " : "队列 ") + index + "/" + total;
  }

  function renderBagState(state) {
    if (!state || !els.bagTime) return;
    bagState = state;
    const duration = state.duration || 0;
    const time = state.time || 0;
    if (!bagDragging) {
      const ratio = duration > 0 ? time / duration : 0;
      els.bagProgress.value = String(Math.round(ratio * 1000));
    }
    els.bagTime.textContent = time.toFixed(1) + " / " + duration.toFixed(1) + " s";
    const percent = state.samples ? (state.fidelity * 100).toFixed(1) : "--";
    els.bagFidelity.textContent = percent === "--" ? "--" : percent + "%";
    els.bagFidelity.classList.toggle("is-good", state.samples > 0 && state.fidelity >= 0.9);
    els.bagFidelity.classList.toggle("is-bad", state.samples > 0 && state.fidelity < 0.9);
    els.bagErrAvg.textContent = state.samples ? state.error_avg.toFixed(3) : "--";
    els.bagErrMax.textContent = state.samples ? state.error_max.toFixed(2) : "--";
    els.bagSamples.textContent = String(state.samples || 0);
    els.bagTeleports.textContent = String(state.teleports || 0);
    els.bagTopic.textContent = state.pose_topic || "--";
    if (els.bagCurrentName) {
      els.bagCurrentName.textContent = state.loaded && state.name ? state.name : "未载入";
    }
    if (els.bagBatteryInfo) {
      els.bagBatteryInfo.textContent = state.loaded
        ? state.battery_topic
          ? state.battery_topic + "（日志）"
          : "无电量话题 · 固定 100%"
        : "--";
    }
    els.bagDuration.textContent = duration ? duration.toFixed(1) : "--";
    const summary = state.summary;
    els.bagPoseCount.textContent = summary ? String(summary.poses) : "--";
    els.bagPathLen.textContent = summary ? summary.path_length.toFixed(1) : "--";
    const maps = state.maps || {};
    if (els.bagMapInfo) {
      const whole = maps.whole_map;
      els.bagMapInfo.textContent = whole
        ? whole.size[0] + "×" + whole.size[1] + " m"
        : "--";
    }
    if (els.bagCostInfo) {
      const cost = maps.grid_costmap;
      els.bagCostInfo.textContent = cost
        ? cost.size[0] + "×" + cost.size[1] + " m · " + (cost.frames || 1) + " 帧"
        : "--";
    }
    if (state.loaded && state.name && state.name !== bagLiveName) {
      // 连续播放会自动切换日志：同步新的日志轨迹与话题
      bagLiveName = state.name;
      syncBagDetail();
    }
    if (state.loaded) {
      const noMap = !(maps.whole_map || {}).size
        ? "（该日志没有地图数据，已切换为不限制区域的开阔场地）"
        : "";
      const paused = state.active && !state.playing && !state.finished;
      setBagHint(
        (state.playing
          ? "回放中：青色线是仿真实际轨迹，红色细线是跳变（重定位/换日志）"
          : state.finished
          ? "回放结束，可拖动进度条重播或重新载入其他日志"
          : paused
          ? "已暂停，点“继续”从当前位置接着回放"
          : "已载入，点“播放”开始把日志轨迹映射到仿真机器人上") + noMap,
        0
      );
    } else {
      setBagHint("选择日志后点“载入”，再点“播放”；勾选多个可“连续播放”。", 0);
    }
    if (els.bagPlayBtn) {
      const paused = state.loaded && state.active && !state.playing && !state.finished;
      els.bagPlayBtn.textContent = paused ? "继续" : "播放";
      els.bagPlayBtn.title = paused
        ? "从暂停位置继续回放"
        : "从头播放（已暂停时可直接继续）";
      els.bagPauseBtn.disabled = !state.playing;
    }
    updateBagVideo(state);
    if (els.bagVideoToggleBtn) {
      els.bagVideoToggleBtn.setAttribute(
        "aria-pressed",
        els.bagVideo && !els.bagVideo.hidden ? "true" : "false"
      );
    }
    syncPopups();
  }

  let costmapPollAt = 0;
  let costmapTime = -1;

  async function refreshCostmap(force) {
    const state = bagState;
    if (!state || !state.loaded) {
      costmapLayer = null;
      return;
    }
    const time = state.time || 0;
    // 暂停 / 拖动进度条后也要更新代价地图，按时间变化判定即可
    if (!force && !state.playing && Math.abs(time - costmapTime) < 0.3) return;
    const now = performance.now();
    if (!force && now - costmapPollAt < 500) return;
    costmapPollAt = now;
    costmapTime = time;
    try {
      const data = await request("/api/bag/costmap?t=" + time);
      if (data.costmap) updateCostmapLayer(data.costmap);
    } catch (error) {
      /* 回放切换/结束期间的请求失败忽略 */
    }
  }

  // ------------------------------------------------------------- 日志图像窗口
  function updateBagVideo(state) {
    if (!els.bagVideo) return;
    const loaded = !!(state && state.loaded && state.image_frames);
    els.bagVideo.hidden = !loaded || bagVideoClosed;
    if (!loaded) {
      bagImageState = { index: -1, key: "", busy: false };
      if (bagImageTimer) {
        clearInterval(bagImageTimer);
        bagImageTimer = 0;
      }
      if (popups.video && !popups.video.win.closed) popups.video.win.close();
      return;
    }
    if (els.bagVideoTitle && state.image_topic) {
      els.bagVideoTitle.textContent = state.image_topic;
    }
    const key = (state.name || "") + ":" + (state.time || 0).toFixed(1);
    if (!bagImageTimer) {
      bagImageTimer = setInterval(() => refreshBagImage(false), 160);
    }
    if (bagImageState.key !== key) refreshBagImage(true);
  }

  async function refreshBagImage(force) {
    const state = bagState;
    if (!state || !state.loaded || !state.image_frames) return;
    if (bagImageState.busy) return;
    bagImageState.busy = true;
    try {
      const data = await request("/api/bag/frame?t=" + (state.time || 0));
      const image = data.image;
      if (!image) {
        bagImageState = { index: -1, key: "", busy: false };
        return;
      }
      const stamp = (state.name || "") + ":" + image.index;
      if (stamp !== bagImageState.key) {
        bagImageState.key = stamp;
        if (els.bagVideoImg) els.bagVideoImg.src = image.image;
      }
      if (els.bagVideoMeta) {
        els.bagVideoMeta.textContent =
          image.index + 1 + "/" + image.count + " · " + image.width + "×" + image.height;
      }
      bagImageState.index = image.index;
    } catch (error) {
      /* 回放切换/结束期间的请求失败忽略 */
    } finally {
      bagImageState.busy = false;
    }
  }

  // ------------------------------------------------------------- 日志文件与队列
  async function syncBagDetail() {
    try {
      const data = await request("/api/bag");
      if (data.path && data.path.length) bagPath = data.path;
      if (data.state) {
        bagVideoClosed = false;
        renderBagState(data.state);
        renderBagTopics(data.topics || []);
      }
      renderBagQueue(data.queue);
      refreshCostmap(true);
      refreshBagImage(true);
    } catch (error) {
      /* 切换瞬间的失败忽略，下一次轮询会补上 */
    }
  }

  async function refreshBags() {
    try {
      const data = await request("/api/bag");
      bagEntries = data.bags || [];
      bagGroups = data.groups || [];
      renderBagFolders();
      if (!bagCurrentKey && bagEntries.length) bagCurrentKey = bagEntries[0].key;
      renderBagList();
      renderBagQueue(data.queue);
      renderBagState(data.state);
      if (data.path && data.path.length) bagPath = data.path;
    } catch (error) {
      setBagHint("刷新失败：" + error.message, 6000);
    }
  }

  async function loadBag() {
    const key = bagCurrentKey;
    if (!key) {
      setBagHint("日志目录里没有可用的 bag 文件", 6000);
      return;
    }
    setBagHint("正在解析日志…（100 MB 级 bag 约 0.5 s）", 6000);
    try {
      const data = await request("/api/bag/load", { name: key });
      bagCurrentKey = key;
      renderBagState(data.state);
      renderBagQueue(data.queue);
      bagPath = data.path || [];
      renderBagTopics(data.topics);
      bagVideoClosed = false;
      refreshCostmap(true);
      refreshBagImage(true);
      const summary = data.summary || {};
      setBagHint(
        "已载入 " + key.split("/").pop() + "：" + (summary.poses || 0) + " 帧位姿，时长 " +
          (data.state.duration || 0).toFixed(1) + " s，轨迹 " +
          (summary.path_length || 0).toFixed(1) + " m",
        6000
      );
    } catch (error) {
      setBagHint("载入失败：" + error.message, 6000);
    }
  }

  async function startBagQueue() {
    // 优先按用户勾选的顺序（可用 ▲▼ 调整），没勾选则按当前列表顺序
    const known = new Set(bagEntries.map((bag) => bag.key));
    const names = bagOrder.filter((key) => known.has(key));
    const queueNames = names.length ? names : bagVisible.map((bag) => bag.key);
    if (!queueNames.length) {
      setBagHint("当前文件夹里没有可播放的日志", 6000);
      return;
    }
    setBagHint("连续播放 " + queueNames.length + " 个日志…", 6000);
    try {
      const data = await request("/api/bag/playlist", { names: queueNames });
      renderBagQueue(data.queue);
      renderBagState(data.state);
    } catch (error) {
      setBagHint("连续播放失败：" + error.message, 6000);
    }
  }

  async function bagControl(command, extra) {
    try {
      const data = await request("/api/bag/control", Object.assign({ command }, extra || {}));
      renderBagState(data.state);
      if (data.path) bagPath = data.path;
    } catch (error) {
      setBagHint("操作失败：" + error.message, 6000);
    }
  }

  if (els.bagSelectAll) {
    els.bagSelectAll.addEventListener("change", () => {
      if (els.bagSelectAll.checked) selectAllVisible();
      else clearBagSelection();
    });
  }

  if (els.bagVideoToggleBtn) {
    els.bagVideoToggleBtn.addEventListener("click", () => {
      if (!bagState || !bagState.loaded) {
        setBagHint("先载入日志，图像窗口只在回放日志时显示", 6000);
        return;
      }
      bagVideoClosed = !bagVideoClosed;
      if (!bagVideoClosed) {
        refreshBagImage(true);
      }
      if (bagVideoClosed && popups.video && !popups.video.win.closed) {
        popups.video.win.close();
      }
      updateBagVideo(bagState);
      els.bagVideoToggleBtn.setAttribute("aria-pressed", bagVideoClosed ? "false" : "true");
    });
  }

  if (els.bagVideoLargeBtn) {
    els.bagVideoLargeBtn.addEventListener("click", () => {
      const large = els.bagVideo.classList.toggle("is-large");
      els.bagVideoLargeBtn.setAttribute("aria-pressed", large ? "true" : "false");
      els.bagVideoLargeBtn.textContent = large ? "⤡" : "⤢";
      els.bagVideoLargeBtn.title = large ? "还原大小" : "放大";
    });
    els.bagVideoPopBtn.addEventListener("click", () => {
      popupWindow("video", els.bagVideo, {
        title: (bagState && bagState.image_topic) || "日志图像",
        width: 620,
        height: 540,
      });
    });
    els.bagVideoCloseBtn.addEventListener("click", () => {
      bagVideoClosed = true;
      els.bagVideo.hidden = true;
      if (popups.video && !popups.video.win.closed) popups.video.win.close();
    });
  }

  if (els.bagPopBtn) {
    els.bagPopBtn.addEventListener("click", () => {
      popupWindow("player", bagPlayerEl, {
        title: "日志播放 · 机器人控制台",
        width: 1020,
        height: 480,
      });
    });
  }

  if (els.bagLoadBtn) {
    els.bagRefreshBtn.addEventListener("click", refreshBags);
    els.bagLoadBtn.addEventListener("click", loadBag);
    els.bagQueueBtn.addEventListener("click", startBagQueue);
    els.bagFolder.addEventListener("change", renderBagList);
    els.bagPlayBtn.addEventListener("click", () => {
      // 暂停/拖动后点"继续"是从当前位置接着放，不是从头重播
      const paused = bagState && bagState.active && !bagState.playing && !bagState.finished;
      bagControl(paused ? "resume" : "play");
    });
    els.bagPauseBtn.addEventListener("click", () => bagControl("pause"));
    els.bagStopBtn.addEventListener("click", () => bagControl("stop"));
    els.bagSpeed.addEventListener("change", () => {
      // 回放倍速叠加在仿真倍速上：直接改仿真速度
      const factor = Number(els.bagSpeed.value) || 1;
      send("/api/speed", { speed: factor >= 4 ? 4 : factor });
    });
    els.bagProgress.addEventListener("pointerdown", () => {
      bagDragging = true;
    });
    els.bagProgress.addEventListener("pointerup", () => {
      bagDragging = false;
      const duration = bagState && bagState.duration ? bagState.duration : 0;
      const ratio = Number(els.bagProgress.value) / 1000;
      bagControl("seek", { time: ratio * duration });
    });
    els.bagProgress.addEventListener("change", () => {
      if (bagDragging) return;
      const duration = bagState && bagState.duration ? bagState.duration : 0;
      const ratio = Number(els.bagProgress.value) / 1000;
      bagControl("seek", { time: ratio * duration });
    });
    refreshBags();
  }

  // ------------------------------------------------------------- 模块显示 / 拖动
  const MODULE_KEY = "mowerModulesV1";
  const OFFSET_KEY = "mowerOffsetsV1";
  const modulePanels = {};
  document.querySelectorAll("[data-module]").forEach((panel) => {
    modulePanels[panel.dataset.module] = panel;
  });
  const moduleDefaults = {
    telemetry: true,
    manual: true,
    zones: true,
    events: true,
    params: true,
  };
  let moduleState = Object.assign({}, moduleDefaults);
  try {
    const saved = JSON.parse(localStorage.getItem(MODULE_KEY) || "null");
    if (saved && typeof saved === "object") {
      moduleState = Object.assign(moduleState, saved);
    }
  } catch (error) {
    /* ignore */
  }
  let offsets = {};
  try {
    const saved = JSON.parse(localStorage.getItem(OFFSET_KEY) || "null");
    if (saved && typeof saved === "object") offsets = saved;
  } catch (error) {
    /* ignore */
  }

  function applyModuleState() {
    Object.keys(modulePanels).forEach((key) => {
      const panel = modulePanels[key];
      const visible = moduleState[key] !== false;
      panel.classList.toggle("is-hidden", !visible);
      const toggle = document.querySelector('[data-module-toggle="' + key + '"]');
      if (toggle) toggle.checked = visible;
    });
  }

  function applyOffsets() {
    Object.keys(modulePanels).forEach((key) => {
      const panel = modulePanels[key];
      const offset = offsets[key];
      panel.style.transform =
        offset && panel.querySelector("[data-drag-handle]")
          ? "translate(" + Number(offset.x || 0) + "px, " + Number(offset.y || 0) + "px)"
          : "";
    });
  }

  function initDrag(panel) {
    const handle = panel.querySelector("[data-drag-handle]");
    if (!handle) return;
    let drag = null;
    handle.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      const key = panel.dataset.dragPanel;
      const base = offsets[key] || { x: 0, y: 0 };
      drag = { x: event.clientX, y: event.clientY, baseX: base.x, baseY: base.y, key };
      panel.classList.add("dragging");
      handle.setPointerCapture(event.pointerId);
    });
    handle.addEventListener("pointermove", (event) => {
      if (!drag) return;
      const dx = drag.baseX + event.clientX - drag.x;
      const dy = drag.baseY + event.clientY - drag.y;
      offsets[drag.key] = { x: dx, y: dy };
      panel.style.transform = "translate(" + dx + "px, " + dy + "px)";
    });
    const finish = (event) => {
      if (!drag) return;
      drag = null;
      panel.classList.remove("dragging");
      localStorage.setItem(OFFSET_KEY, JSON.stringify(offsets));
      if (event && handle.hasPointerCapture(event.pointerId)) {
        handle.releasePointerCapture(event.pointerId);
      }
    };
    handle.addEventListener("pointerup", finish);
    handle.addEventListener("pointercancel", finish);
  }

  Object.keys(modulePanels).forEach((key) => initDrag(modulePanels[key]));

  // 底部面板页签：事件日志 / 日志播放 切换显示
  const DOCK_TAB_KEY = "mowerDockTabV1";
  let dockTab = "events";
  try {
    dockTab = localStorage.getItem(DOCK_TAB_KEY) || "events";
  } catch (error) {
    dockTab = "events";
  }

  function selectDockTab(name, persist) {
    dockTab = name === "bag" ? "bag" : "events";
    els.dockTabs.forEach((tab) => {
      const active = tab.dataset.dockTab === dockTab;
      tab.classList.toggle("is-active", active);
      tab.setAttribute("aria-selected", String(active));
    });
    els.dockPanes.forEach((pane) => {
      pane.classList.toggle("is-active", pane.dataset.dockPane === dockTab);
    });
    if (els.logCount) {
      els.logCount.style.display = dockTab === "events" ? "" : "none";
    }
    if (persist !== false) {
      try {
        localStorage.setItem(DOCK_TAB_KEY, dockTab);
      } catch (error) {
        /* ignore */
      }
    }
  }

  els.dockTabs.forEach((tab) => {
    tab.addEventListener("click", () => selectDockTab(tab.dataset.dockTab));
  });
  selectDockTab(dockTab, false);
  document.querySelectorAll("[data-module-toggle]").forEach((checkbox) => {
    checkbox.addEventListener("change", () => {
      const key = checkbox.dataset.moduleToggle;
      moduleState[key] = checkbox.checked;
      localStorage.setItem(MODULE_KEY, JSON.stringify(moduleState));
      applyModuleState();
    });
  });
  els.moduleBtn.addEventListener("click", (event) => {
    event.stopPropagation();
    const willOpen = els.moduleMenu.hidden;
    els.moduleMenu.hidden = !willOpen;
    els.moduleBtn.setAttribute("aria-expanded", String(willOpen));
  });
  els.moduleMenu.addEventListener("click", (event) => event.stopPropagation());
  document.addEventListener("click", () => {
    if (!els.moduleMenu.hidden) {
      els.moduleMenu.hidden = true;
      els.moduleBtn.setAttribute("aria-expanded", "false");
    }
  });
  els.resetLayoutBtn.addEventListener("click", () => {
    offsets = {};
    localStorage.removeItem(OFFSET_KEY);
    applyOffsets();
  });
  // ------------------------------------------------------------- 参数设置
  function loadConfig(config) {
    configInputs.forEach((input) => {
      const value = config[input.dataset.config];
      if (value === undefined) return;
      input.value =
        input.dataset.type === "int" ? String(parseInt(value, 10)) : String(Number(value));
    });
  }

  configInputs.forEach((input) => {
    input.addEventListener("input", () => {
      paramsDirty = true;
    });
  });

  async function applyParams() {
    const payload = { mowing_mode: els.mowMode.value };
    configInputs.forEach((input) => {
      const raw = input.value.trim();
      if (raw === "") return;
      payload[input.dataset.config] =
        input.dataset.type === "int" ? parseInt(raw, 10) : Number(raw);
    });
    try {
      const state = await request("/api/config", payload);
      paramsDirty = false;
      ingest(state);
      els.paramMessage.textContent = "参数已应用，下一次规划生效";
    } catch (error) {
      els.paramMessage.textContent = error.message;
    }
  }

  async function resetParams() {
    try {
      const state = await request("/api/config/reset", {});
      paramsDirty = false;
      ingest(state);
      els.paramMessage.textContent = "已恢复默认参数";
    } catch (error) {
      els.paramMessage.textContent = error.message;
    }
  }

  els.applyParamsBtn.addEventListener("click", applyParams);
  els.resetParamsBtn.addEventListener("click", resetParams);

  // ------------------------------------------------------------- 参数分类页签
  const PARAM_TAB_KEY = "mowerParamTabV1";

  function applyParamTab(name) {
    const valid = els.paramTabs.some((tab) => tab.dataset.paramTab === name);
    const active = valid ? name : "robot";
    els.paramTabs.forEach((tab) => {
      tab.classList.toggle("is-active", tab.dataset.paramTab === active);
      tab.setAttribute("aria-selected", String(tab.dataset.paramTab === active));
    });
    els.paramPanes.forEach((pane) => {
      pane.classList.toggle("is-active", pane.dataset.paramPane === active);
    });
  }

  els.paramTabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      applyParamTab(tab.dataset.paramTab);
      localStorage.setItem(PARAM_TAB_KEY, tab.dataset.paramTab);
    });
  });

  applyParamTab(localStorage.getItem(PARAM_TAB_KEY) || "robot");

  // ------------------------------------------------------------- 面板折叠
  const COLLAPSE_KEY = "mowerCollapsedV1";
  let collapsed = {};
  try {
    collapsed = JSON.parse(localStorage.getItem(COLLAPSE_KEY) || "null") || {};
  } catch (error) {
    collapsed = {};
  }

  function applyCollapsed() {
    Object.keys(modulePanels).forEach((key) => {
      modulePanels[key].classList.toggle("is-collapsed", collapsed[key] === true);
    });
    const logHidden = collapsed.events === true;
    if (els.logDock) els.logDock.classList.toggle("is-collapsed", logHidden);
    if (els.logToggleBtn) {
      els.logToggleBtn.textContent = logHidden ? "展开" : "收起";
      els.logToggleBtn.setAttribute("aria-expanded", String(!logHidden));
    }
  }

  function toggleCollapsed(panel) {
    const key = panel.dataset.dragPanel;
    collapsed[key] = !collapsed[key];
    localStorage.setItem(COLLAPSE_KEY, JSON.stringify(collapsed));
    applyCollapsed();
  }

  Object.keys(modulePanels).forEach((key) => {
    const panel = modulePanels[key];
    const heading = panel.querySelector("h2");
    if (!heading) return;
    heading.addEventListener("click", (event) => {
      if (event.target.closest("[data-drag-handle]")) return;
      if (panel.classList.contains("dragging")) return;
      toggleCollapsed(panel);
    });
  });
  els.collapseAllBtn.addEventListener("click", () => {
    Object.keys(modulePanels).forEach((key) => {
      collapsed[key] = true;
    });
    localStorage.setItem(COLLAPSE_KEY, JSON.stringify(collapsed));
    applyCollapsed();
  });
  els.expandAllBtn.addEventListener("click", () => {
    collapsed = {};
    localStorage.setItem(COLLAPSE_KEY, JSON.stringify(collapsed));
    applyCollapsed();
  });
  if (els.logToggleBtn) {
    els.logToggleBtn.addEventListener("click", (event) => {
      event.stopPropagation();
      collapsed.events = collapsed.events !== true;
      localStorage.setItem(COLLAPSE_KEY, JSON.stringify(collapsed));
      applyCollapsed();
    });
  }
  applyCollapsed();

  // ------------------------------------------------------------- 地图缩放/坐标
  function canvasPoint(event) {
    const rect = canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  function updateMapCursor(event) {
    if (!lastGeometry) return;
    const point = canvasPoint(event);
    const worldPoint = lastGeometry.toWorld(point.x, point.y);
    hoverWorld = worldPoint;
    els.mapCursor.textContent =
      "X " + worldPoint.x.toFixed(2) + " m · Y " + worldPoint.y.toFixed(2) + " m";
    els.mapCursor.style.display = "block";
    const maxLeft = Math.max(4, canvas.clientWidth - els.mapCursor.offsetWidth - 20);
    const maxTop = Math.max(4, canvas.clientHeight - els.mapCursor.offsetHeight - 20);
    els.mapCursor.style.left = clamp(point.x, 4, maxLeft) + "px";
    els.mapCursor.style.top = clamp(point.y, 4, maxTop) + "px";
  }

  canvas.addEventListener("pointerdown", (event) => {
    if (event.button !== 0) return;
    const point = canvasPoint(event);
    if (pathEditId && lastGeometry) {
      commitPathEdit(lastGeometry.toWorld(point.x, point.y));
      return;
    }
    if (editKind && lastGeometry) {
      const worldPoint = lastGeometry.toWorld(point.x, point.y);
      if (editKind === "no_go") {
        rectStart = worldPoint;
        rectDraft = {
          x0: worldPoint.x,
          y0: worldPoint.y,
          x1: worldPoint.x,
          y1: worldPoint.y,
        };
        canvas.setPointerCapture(event.pointerId);
        return;
      }
      placeEdit(worldPoint);
      return;
    }
    panState = {
      screenX: point.x,
      screenY: point.y,
      centerX: viewCenterX,
      centerY: viewCenterY,
      scale: lastGeometry ? lastGeometry.scale : 1,
    };
    canvas.classList.add("is-panning");
    canvas.setPointerCapture(event.pointerId);
  });

  canvas.addEventListener("pointermove", (event) => {
    updateMapCursor(event);
    if (rectStart && rectDraft && lastGeometry) {
      const point = canvasPoint(event);
      const worldPoint = lastGeometry.toWorld(point.x, point.y);
      rectDraft = {
        x0: Math.min(rectStart.x, worldPoint.x),
        y0: Math.min(rectStart.y, worldPoint.y),
        x1: Math.max(rectStart.x, worldPoint.x),
        y1: Math.max(rectStart.y, worldPoint.y),
      };
      return;
    }
    if (!panState) return;
    const point = canvasPoint(event);
    viewCenterX = panState.centerX - (point.x - panState.screenX) / panState.scale;
    viewCenterY = panState.centerY + (point.y - panState.screenY) / panState.scale;
  });

  function finishPan(event) {
    if (rectStart) {
      commitRect();
      if (event && canvas.hasPointerCapture(event.pointerId)) {
        canvas.releasePointerCapture(event.pointerId);
      }
      return;
    }
    if (!panState) return;
    panState = null;
    canvas.classList.remove("is-panning");
    if (event && canvas.hasPointerCapture(event.pointerId)) {
      canvas.releasePointerCapture(event.pointerId);
    }
  }

  canvas.addEventListener("pointerup", finishPan);
  canvas.addEventListener("pointercancel", (event) => {
    if (rectStart) {
      rectStart = null;
      rectDraft = null;
      if (event && canvas.hasPointerCapture(event.pointerId)) {
        canvas.releasePointerCapture(event.pointerId);
      }
      return;
    }
    finishPan(event);
  });
  canvas.addEventListener("pointerleave", (event) => {
    if (rectStart) {
      updateMapCursor(event);
      return;
    }
    if (!panState) els.mapCursor.style.display = "none";
    else updateMapCursor(event);
  });

  canvas.addEventListener(
    "wheel",
    (event) => {
      event.preventDefault();
      const point = canvasPoint(event);
      zoomAt(point.x, point.y, Math.exp(-event.deltaY * 0.0012));
    },
    { passive: false }
  );

  canvas.addEventListener("dblclick", () => {
    if (lastGeometry) resetView(lastGeometry.world);
  });

  function zoomFromCenter(factor) {
    zoomAt(canvas.clientWidth / 2, canvas.clientHeight / 2, factor);
  }

  els.zoomInBtn.addEventListener("click", () => zoomFromCenter(1.25));
  els.zoomOutBtn.addEventListener("click", () => zoomFromCenter(0.8));
  els.zoomResetBtn.addEventListener("click", () => {
    if (lastGeometry) resetView(lastGeometry.world);
  });

  // ------------------------------------------------------------- 启动
  if (els.editButtons.length) {
    els.editButtons.forEach((button) => {
      button.addEventListener("click", () => setEditKind(button.dataset.editKind));
    });
  }
  if (els.editRadius) {
    els.editRadius.addEventListener("input", updateEditHint);
  }
  if (els.editClearBtn) {
    els.editClearBtn.addEventListener("click", async () => {
      try {
        const state = await request("/api/map/edit", { action: "clear" });
        ingest(state);
        if (els.editMapMessage) els.editMapMessage.textContent = "已清空当前地图的编辑对象";
      } catch (error) {
        if (els.editMapMessage) els.editMapMessage.textContent = "清空失败：" + error.message;
      }
    });
  }
  if (els.themeSelect) {
    els.themeSelect.addEventListener("change", () => {
      applyTheme(els.themeSelect.value);
    });
  }
  let savedTheme = "dark";
  try {
    savedTheme = localStorage.getItem(THEME_KEY) || "dark";
  } catch (error) {
    savedTheme = "dark";
  }
  applyTheme(savedTheme);

  // 高倍速时不必 60 fps 重绘：前端渲染和同进程的仿真线程抢 CPU，
  // 把帧率降到 30 fps 能让仿真多吃一半的 CPU 时间。
  let lastFrameAt = 0;

  function frame(now) {
    const speed = current && current.speed ? current.speed : 1;
    const minGap = speed >= 8 ? 1000 / 30 : 0;
    if (!minGap || now - lastFrameAt >= minGap) {
      lastFrameAt = now;
      draw(now);
    }
    requestAnimationFrame(frame);
  }

  requestAnimationFrame(frame);
  poll();
})();
