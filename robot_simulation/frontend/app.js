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

    log: document.getElementById("log"),
    phaseChip: document.getElementById("phaseChip"),
    zoneChip: document.getElementById("zoneChip"),

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
    reportBtn: document.getElementById("reportBtn"),
    resetBtn: document.getElementById("resetBtn"),
    estopBtn: document.getElementById("estopBtn"),
    manualLinear: document.getElementById("manualLinear"),
    manualAngular: document.getElementById("manualAngular"),
    manualMessage: document.getElementById("manualMessage"),
    speedButtons: Array.from(document.querySelectorAll(".speed-btn")),
    mapSelect: document.getElementById("mapSelect"),
    deleteMapBtn: document.getElementById("deleteMapBtn"),
    applyParamsBtn: document.getElementById("applyParamsBtn"),
    resetParamsBtn: document.getElementById("resetParamsBtn"),
    paramMessage: document.getElementById("paramMessage"),
    importFileBtn: document.getElementById("importFileBtn"),
    importAllMapsBtn: document.getElementById("importAllMapsBtn"),
    importFileInput: document.getElementById("importFileInput"),
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
    dockTab: document.getElementById("dockTab"),
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
    bagProgress: document.getElementById("bagProgress"),
    bagTime: document.getElementById("bagTime"),
    bagBar: document.getElementById("bagBar"),
    bagWorld: document.getElementById("bagWorld"),
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
    editorUndoBtn: document.getElementById("editorUndoBtn"),
    editorBlankBtn: document.getElementById("editorBlankBtn"),
    editorSaveBtn: document.getElementById("editorSaveBtn"),
    editorDeleteBtn: document.getElementById("editorDeleteBtn"),
    editorExitBtn: document.getElementById("editorExitBtn"),
    editorRegionStartBtn: document.getElementById("editorRegionStartBtn"),
    editorRegionSaveBtn: document.getElementById("editorRegionSaveBtn"),
    editorRegionCancelBtn: document.getElementById("editorRegionCancelBtn"),
    editorPassageStartBtn: document.getElementById("editorPassageStartBtn"),
    editorPassageSaveBtn: document.getElementById("editorPassageSaveBtn"),
    editorPassageCancelBtn: document.getElementById("editorPassageCancelBtn"),
    editorNoGoBtn: document.getElementById("editorNoGoBtn"),
    editorDockBtn: document.getElementById("editorDockBtn"),
    editorStaticBtn: document.getElementById("editorStaticBtn"),
    editorDynamicBtn: document.getElementById("editorDynamicBtn"),
    editorRadius: document.getElementById("editorRadius"),
    editorSpan: document.getElementById("editorSpan"),
    editorSpeed: document.getElementById("editorSpeed"),
    editorDockRadius: document.getElementById("editorDockRadius"),
    editorSourceSelect: document.getElementById("editorSourceSelect"),
    editorLoadBtn: document.getElementById("editorLoadBtn"),
    editorName: document.getElementById("editorName"),
    editorMessage: document.getElementById("editorMessage"),
    editorProblems: document.getElementById("editorProblems"),
    editorList: document.getElementById("editorList"),
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
      cloud: "255, 122, 220",
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
      cloud: "176, 38, 150",
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
      cloud: "255, 150, 220",
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
  let strokeBreakPending = false;   // 换日志后下一点不跟上一段连线
  // 话题图层开关：日志里的 grid_costmap（红色代价方块）、轨迹、图像窗口都可单独屏蔽
  const LAYER_KEY = "mowerTopicLayersV1";
  let layerVisible = { pose: true, costmap: true, cloud: true, image: true, map: true, static_costmap: true };
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

  // 世界时间 = 日志当时的墙钟时间（消息头里的 Unix 时间戳），不是仿真时钟
  function wallClockText(epochSeconds) {
    if (!epochSeconds) return "--";
    const date = new Date(Number(epochSeconds) * 1000);
    if (Number.isNaN(date.getTime())) return "--";
    const pad = (v) => String(v).padStart(2, "0");
    // 用 UTC 显示：日志里的时间戳就是 UTC，这样和包名（zmqbag_x_YYYY-MM-DD-HH-MM-SS）对得上；
    // 带毫秒（回放采样率 10~50 Hz，秒级看不出来）
    const ms = date.getUTCMilliseconds();
    return (
      pad(date.getUTCMonth() + 1) + "-" + pad(date.getUTCDate()) + " " +
      pad(date.getUTCHours()) + ":" + pad(date.getUTCMinutes()) + ":" + pad(date.getUTCSeconds()) +
      "." + String(ms).padStart(3, "0")
    );
  }

  let lastEditorLock = "";

  function ingest(state) {
    previous = current || state;
    current = state;
    currentAt = performance.now();
    if (state.queue) renderBagQueue(state.queue);
    if (state.bag) renderBagBar(state.bag);
    // 编辑按钮的可用性跟着任务/回放状态变：状态轮询里必须刷新一次，
    // 否则"忙的时候禁用 → 停止后按钮永远灰着"（地图编辑点了没反应）。
    const editing = (state.draft && state.draft.active) || false;
    const editorLock = (editorTaskBusy(state) ? "busy" : "idle") + "|" + (editing ? "on" : "off");
    if (editorLock !== lastEditorLock) {
      lastEditorLock = editorLock;
      updateEditorUI();
    }

    const replaying = !!(state.bag && state.bag.loaded);
    // 连续播放会自动换日志、重建场地、时间轴归零：这些都不能把上一段的路径抹掉，
    // 只有手动载入日志 / 复位才清除轨迹。
    const queueRunning = !!(bagQueue && bagQueue.active);
    if (lastMapId !== null && state.map_id !== lastMapId) {
      // 手动载入日志/换地图会整体换场地并挪车位，这不是回放里的"跳变"：
      // 直接重新起笔，否则会从上一张图的旧位置拉一条初始跳线过来。
      // 只有连续播放（自动换日志换场地）才接着画，段间用跳线连。
      if (!queueRunning) trail = [];
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
      updateEditorSourceOptions(state.world.maps);
      updateDeleteButton();
    }
    editorApplyDraft(state.draft);
    editorSampleTrace(state);

    if (state.time + 1e-6 < lastSampleTime && !replaying && !queueRunning) {
      trail = [];
      previous = state;
      mapVersion = -1;
    }
    lastSampleTime = state.time;

    const bag = state.bag || {};
    const teleports = Number(bag.teleports || 0);
    const restarts = Number(bag.restarts || 0);
    const logName = bag.name || "";
    const queuePlaying = !!(bagQueue && bagQueue.active);
    const nameChanged = !!(logName && lastLogName && logName !== lastLogName);
    if (nameChanged) {
      // 连续播放会自动换日志，等于重新播一份日志：上一段路径保留，
      // 但这里重新起笔——不画跳线，也不跟上一段连线。
      if (!queuePlaying) trail = [];
      else strokeBreakPending = true;
      lastRestarts = restarts;
      lastBagActive = !!bag.active;
    } else if (restarts > lastRestarts) {
      // 拖了进度条：从当前位置重新起笔画，不画"初始跳线"
      trail = [];
    } else if (!!bag.active && !lastBagActive && !queuePlaying) {
      trail = [];
    }
    lastRestarts = restarts;
    lastBagActive = !!bag.active;
    if (logName) lastLogName = logName;

    let tail = trail[trail.length - 1];
    let step = tail ? Math.hypot(state.pose.x - tail.x, state.pose.y - tail.y) : 0.0;
    if (tail && step > 2.0 && !bag.active && !queuePlaying) {
      // 载入日志会换场地并把车整体挪过去，这不是回放里的"跳变"：
      // 重新起笔，否则会从旧位置拉一条初始跳线过来。
      trail = [];
      tail = null;
      step = 0.0;
    }
    if (!tail || step > 0.02) {
      // 回放中发生重定位、或位置突然跳很远：这一段画成红色跳线（换包不算，重新起笔）
      const jump = !!tail && !strokeBreakPending && (teleports > lastTeleportsMark || step > 2.0);
      lastTeleportsMark = teleports;
      trail.push({
        x: state.pose.x,
        y: state.pose.y,
        jump,
        gap: strokeBreakPending,
      });
      strokeBreakPending = false;
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

  function updateEditorSourceOptions(maps) {
    if (!els.editorSourceSelect) return;
    const signature = maps.map((item) => item.id + ":" + item.name).join("|");
    if (signature === editor.sourceSignature) return;
    editor.sourceSignature = signature;
    const previous = els.editorSourceSelect.value;
    els.editorSourceSelect.textContent = "";
    maps.forEach((item) => {
      const option = document.createElement("option");
      option.value = String(item.id);
      option.textContent =
        item.id + " · " + item.name + (item.editor ? " · 自建" : item.imported ? " · 导入" : "");
      els.editorSourceSelect.appendChild(option);
    });
    const values = maps.map((item) => String(item.id));
    if (previous && values.includes(previous)) els.editorSourceSelect.value = previous;
    else if (values.length) els.editorSourceSelect.value = values[0];
  }

  function updateMapOptions(maps) {
    const signature = maps
      .map(
        (item) =>
          item.id +
          ":" +
          item.name +
          ":" +
          (item.imported ? 1 : 0) +
          (item.editor ? "e" : "")
      )
      .join("|");
    if (signature === mapOptionsSignature) return;
    mapOptionsSignature = signature;
    els.mapSelect.textContent = "";
    maps.forEach((item) => {
      const option = document.createElement("option");
      option.value = String(item.id);
      option.textContent =
        item.id +
        " · " +
        item.name +
        (item.editor ? " · 自建" : item.imported ? " · 导入" : "");
      els.mapSelect.append(option);
    });
    if (current && current.map_id != null) {
      els.mapSelect.value = String(current.map_id);
    }
  }

  let hoverWorld = null;

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

  // ------------------------------------------------------------- 地图编辑器（草稿）
  const EDITOR_IDLE_HINT =
    "点“空白地图”开始编辑，或选一张已有地图点“载入编辑”：画完区域与通道、放好充电桩后点“保存地图”。编辑期间不能开始建图或割草。";
  const EDITOR_MODES = {
    idle: "",
    delete: "元素删除：在地图上点击区域 / 通道 / 充电桩 / 禁区 / 障碍物即可删除",
    region: "添加区域：手动控制小车沿边界绕一圈（轨迹自动采样）后点“保存区域”",
    passage: "添加通道：手动控制小车从区域 A 开到区域 B 后点“保存通道”",
    no_go: "添加禁区：在地图上按住左键拖出矩形，松开即生效",
    dock: "添加充电桩：在地图上点击放置（全场只能有一个，必须在区域内部）",
    static: "添加障碍物：按住左键在地图上涂抹，松开即生效",
    dynamic: "添加动态障碍：在地图上点击放置（半径/速度取左侧数值）",
    dynamic_path: "设置路径：先点动态障碍物，再在地图上点起点 A 与终点 B",
  };
  const EDITOR_TOOL_BUTTONS = {
    delete: "editorDeleteBtn",
    no_go: "editorNoGoBtn",
    dock: "editorDockBtn",
    static: "editorStaticBtn",
    dynamic: "editorDynamicBtn",
  };
  const EDITOR_TRACE_STEP = 0.12;

  const editor = {
    active: false,
    busy: false,
    mode: "idle",
    trace: [],
    painting: [],
    rect: null,
    draft: null,
    signature: "",
    message: "",
    tone: "",
    // 动态障碍物路径编辑：选中对象 + 已点的起点
    pathTarget: null,
    pathA: null,
    sourceSignature: "",
  };

  function editorNumber(input, fallback, low, high) {
    const value = parseFloat(input ? input.value : "");
    if (!isFinite(value)) return fallback;
    return Math.min(high, Math.max(low, value));
  }

  function editorBrushRadius() {
    return editorNumber(els.editorRadius, 0.35, 0.10, 1.50);
  }

  function editorObstacleRadius() {
    return editorNumber(els.editorSpan, 0.30, 0.12, 1.00);
  }

  function editorPatrolSpeed() {
    return editorNumber(els.editorSpeed, 0.15, 0.05, 0.60);
  }

  function editorDockRadius() {
    return editorNumber(els.editorDockRadius, 0.45, 0.30, 1.20);
  }

  function editorRound(value) {
    return Number(Number(value).toFixed(3));
  }

  function editorTaskBusy(state) {
    const snapshot = state || current || {};
    const task = snapshot.task || {};
    const phase = task.phase || "idle";
    if (!!task.type && !["idle", "done", "stopped"].includes(phase)) return true;
    // 日志回放还在跑也算"任务进行中"：这时改地图会被仿真覆盖
    return !!((snapshot.bag || {}).active);
  }

  function editorSay(text, tone) {
    editor.message = text || "";
    editor.tone = tone || "";
    applyEditorMessage();
  }

  function applyEditorMessage() {
    if (!els.editorMessage) return;
    const busy = editorTaskBusy(current);
    let text = editor.message;
    let tone = editor.tone;
    if (!text) {
      if (busy) {
        text = "任务进行中不能编辑地图，请先停止任务";
        tone = "bad";
      } else if (!editor.active) {
        text = EDITOR_IDLE_HINT;
        tone = "";
      } else if (editor.mode === "dynamic_path" && editor.pathTarget) {
        text = editor.pathA
          ? "动态障碍 #" + editor.pathTarget + "：已记下起点 A，请点终点 B"
          : "动态障碍 #" + editor.pathTarget + "：请点巡逻起点 A";
      } else if (editor.mode === "dynamic_path" && editor.pathA) {
        text = "已记下起点 A，请点终点 B（至少 0.4 m）";
      } else {
        text = EDITOR_MODES[editor.mode] || "编辑中：请选择左侧任一编辑工具";
        tone = "";
      }
    }
    els.editorMessage.textContent = text;
    els.editorMessage.classList.toggle("is-bad", tone === "bad");
    els.editorMessage.classList.toggle("is-ok", tone === "ok");
  }

  function editorTraceStats() {
    let length = 0;
    for (let i = 1; i < editor.trace.length; i += 1) {
      length += Math.hypot(
        editor.trace[i][0] - editor.trace[i - 1][0],
        editor.trace[i][1] - editor.trace[i - 1][1]
      );
    }
    return { points: editor.trace.length, length };
  }

  function editorSampleTrace(state) {
    if (!editor.active) return;
    if (editor.mode !== "region" && editor.mode !== "passage") return;
    const pose = state && state.pose;
    if (!pose) return;
    const last = editor.trace[editor.trace.length - 1];
    if (
      last &&
      Math.hypot(pose.x - last[0], pose.y - last[1]) < EDITOR_TRACE_STEP
    ) {
      return;
    }
    editor.trace.push([editorRound(pose.x), editorRound(pose.y)]);
    if (editor.trace.length > 5000) editor.trace.shift();
    const stats = editorTraceStats();
    applyEditorMessage();
    if (els.editorMessage) {
      els.editorMessage.textContent =
        (editor.mode === "region" ? "区域" : "通道") +
        "轨迹采样中：已记录 " +
        stats.points +
        " 点 / " +
        stats.length.toFixed(1) +
        " m（用左侧手动控制把小车开到位）";
    }
  }

  function editorSetMode(mode) {
    if (!editor.active && mode !== "idle") {
      editorSay("请先点“空白地图”进入编辑", "bad");
      return;
    }
    const next = editor.mode === mode ? "idle" : mode;
    editor.mode = next;
    editor.trace = [];
    editor.painting = [];
    editor.rect = null;
    editorSay("");
    updateEditorUI();
  }

  function editorStartTrace(mode) {
    if (!editor.active) {
      editorSay("请先点“空白地图”进入编辑", "bad");
      return;
    }
    editor.mode = mode;
    editor.trace = [];
    editor.painting = [];
    editor.rect = null;
    const stats = editorTraceStats();
    editorSay(
      (mode === "region" ? "区域" : "通道") +
        "起点已记录，请用左侧手动控制移动小车；采样点数 " +
        stats.points,
      "ok"
    );
    updateEditorUI();
  }

  function editorCancelTrace() {
    if (editor.mode === "region" || editor.mode === "passage") {
      const label = editor.mode === "region" ? "区域" : "通道";
      editor.mode = "idle";
      editor.trace = [];
      editorSay("已取消" + label + "生成", "ok");
      updateEditorUI();
    } else {
      editorSay("当前没有正在生成的区域或通道", "bad");
    }
  }

  async function editorCall(action, body, onOk) {
    if (editor.busy) return;
    editor.busy = true;
    updateEditorUI();
    try {
      const response = await request(
        "/api/editor/" + action,
        Object.assign({}, body || {})
      );
      if (response && response.draft) editorApplyDraft(response.draft);
      if (action === "blank" || action === "discard" || action === "save") {
        // 这三个动作会换掉当前地图，重新拉一次完整状态（含栅格与机器人）
        const state = await request("/api/state");
        ingest(state);
      }
      if (onOk) onOk(response || {});
    } catch (error) {
      editorSay(error.message, "bad");
    } finally {
      editor.busy = false;
      updateEditorUI();
    }
  }

  async function editorReload() {
    try {
      const state = await request("/api/state");
      ingest(state);
    } catch (error) {
      /* 轮询会补上 */
    }
  }

  function editorApplyDraft(draft) {
    const snapshot = draft || { active: false };
    const signature = JSON.stringify(snapshot);
    editor.active = !!snapshot.active;
    editor.draft = snapshot;
    if (!editor.active) {
      editor.mode = "idle";
      editor.trace = [];
      editor.painting = [];
      editor.rect = null;
    }
    if (signature === editor.signature) return;
    editor.signature = signature;
    renderEditorDraftInfo(snapshot);
    updateEditorUI();
  }

  // 正在画区域 / 通道（还没点保存）时，草稿本身虽然"合格"，但不能说校验通过
  function editorPendingProblems() {
    if (editor.mode === "region") {
      return [
        "区域正在生成（未保存）：绕完一圈后点“保存区域”，或点“取消区域”放弃这次区域",
      ];
    }
    if (editor.mode === "passage") {
      return [
        "通道正在生成（未保存）：开到另一个区域后点“保存通道”，或点“取消通道”放弃这次通道",
      ];
    }
    return [];
  }

  function renderEditorProblems(draft) {
    if (!els.editorProblems) return;
    els.editorProblems.textContent = "";
    if (!draft || !draft.active) return;
    const problems = editorPendingProblems().concat(draft.problems || []);
    if (!problems.length) {
      const ok = document.createElement("li");
      ok.className = "is-ok";
      ok.textContent = "校验通过，可以保存地图";
      els.editorProblems.appendChild(ok);
      return;
    }
    problems.forEach((text) => {
      const item = document.createElement("li");
      item.textContent = text;
      els.editorProblems.appendChild(item);
    });
  }

  function renderEditorDraftInfo(draft) {
    if (!els.editorProblems || !els.editorList) return;
    renderEditorProblems(draft);

    els.editorList.textContent = "";
    if (!draft || !draft.active) {
      const empty = document.createElement("p");
      empty.className = "hint";
      empty.textContent = "尚未进入地图编辑";
      els.editorList.appendChild(empty);
      return;
    }
    const rows = [];
    (draft.regions || []).forEach((item) => {
      rows.push({
        kind: "region",
        id: item.id,
        label: "区域 #" + item.id,
        meta: "面积 " + item.area.toFixed(1) + " m² · " + item.points.length + " 顶点",
      });
    });
    (draft.passages || []).forEach((item) => {
      rows.push({
        kind: "passage",
        id: item.id,
        label:
          item.id == null || (draft.passages || []).length < 2
            ? "通道"
            : "通道 #" + item.id,
        meta:
          "连接 区域 #" + item.from + " ↔ #" + item.to + " · 宽 " +
          (item.half * 2).toFixed(1) + " m · 只通行不割草",
      });
    });
    (draft.docks || []).forEach((item) => {
      rows.push({
        kind: "dock",
        id: item.id,
        label: "充电桩 #" + item.id,
        meta: "(" + item.x.toFixed(2) + ", " + item.y.toFixed(2) + ") · r " + item.r.toFixed(2) + " m",
      });
    });
    (draft.no_gos || []).forEach((item) => {
      rows.push({
        kind: "no_go",
        id: item.id,
        label: "禁区 #" + item.id,
        meta:
          "(" + item.x0.toFixed(2) + ", " + item.y0.toFixed(2) + ") → (" +
          item.x1.toFixed(2) + ", " + item.y1.toFixed(2) + ")",
      });
    });
    (draft.static || []).forEach((item) => {
      rows.push({
        kind: "static",
        id: item.id,
        label: "笔迹 #" + item.id,
        meta:
          "实心画笔 · 长 " +
          (item.length != null ? item.length.toFixed(1) : "0.0") +
          " m · 宽 " +
          (item.r * 2).toFixed(2) +
          " m（保存时合并为外围轮廓）",
      });
    });
    (draft.poly_obstacles || []).forEach((item) => {
      rows.push({
        kind: "obstacle",
        id: item.id,
        label: "障碍物 #" + item.id,
        meta: "多边形 · 面积 " + item.area.toFixed(1) + " m²",
      });
    });
    (draft.dynamic || []).forEach((item) => {
      rows.push({
        kind: "dynamic",
        id: item.id,
        label: "动态障碍 #" + item.id,
        meta:
          "A(" + item.ax.toFixed(2) + ", " + item.ay.toFixed(2) + ") → B(" +
          item.bx.toFixed(2) + ", " + item.by.toFixed(2) + ") · r " +
          item.r.toFixed(2) + " m · " +
          (item.speed != null ? item.speed.toFixed(2) : "0.15") + " m/s",
      });
    });
    if (!rows.length) {
      const empty = document.createElement("p");
      empty.className = "hint";
      empty.textContent = "还没有任何元素：先画区域，再放充电桩、用通道连通各区域";
      els.editorList.appendChild(empty);
      return;
    }
    rows.forEach((row) => {
      const node = document.createElement("div");
      node.className = "edit-item";
      const title = document.createElement("strong");
      title.textContent = row.label + " ";
      const meta = document.createElement("em");
      meta.textContent = row.meta;
      node.appendChild(title);
      node.appendChild(meta);
      if (row.kind === "dynamic") {
        const pathBtn = document.createElement("button");
        pathBtn.type = "button";
        pathBtn.className = "edit-path";
        pathBtn.textContent = "设置路径";
        pathBtn.addEventListener("click", () => editorStartPath(row.id));
        node.appendChild(pathBtn);
      }
      const remove = document.createElement("button");
      remove.type = "button";
      remove.textContent = "删除";
      remove.addEventListener("click", () => {
        editorCall(
          "delete",
          { kind: row.kind, id: row.id },
          () => editorSay("已删除" + row.label, "ok")
        );
      });
      node.appendChild(remove);
      els.editorList.appendChild(node);
    });
  }

  const EDITOR_TASK_BUTTONS = ["mapBtn", "mowBtn", "chargeBtn", "resumeBtn"];

  function lockMissionDuringEditing(active) {
    EDITOR_TASK_BUTTONS.forEach((key) => {
      const button = els[key];
      if (!button) return;
      button.disabled = active;
      if (active) button.title = "地图编辑中：保存或退出编辑后才能开始任务";
      else button.removeAttribute("title");
    });
  }

  function updateEditorUI() {
    if (!els.editorBlankBtn) return;
    const busy = editorTaskBusy(current);
    const active = editor.active;
    const locked = busy || editor.busy;
    els.editorBlankBtn.disabled = locked;
    els.editorSaveBtn.disabled = locked || !active;
    els.editorExitBtn.disabled = locked || !active;
    [
      "editorDeleteBtn",
      "editorRegionStartBtn",
      "editorRegionSaveBtn",
      "editorRegionCancelBtn",
      "editorPassageStartBtn",
      "editorPassageSaveBtn",
      "editorPassageCancelBtn",
      "editorNoGoBtn",
      "editorDockBtn",
      "editorStaticBtn",
      "editorDynamicBtn",
    ].forEach((key) => {
      if (els[key]) els[key].disabled = locked || !active;
    });
    // “载入编辑”只有在没进编辑时才点得动
    if (els.editorLoadBtn) els.editorLoadBtn.disabled = locked || active;
    if (els.editorUndoBtn) {
      els.editorUndoBtn.disabled = locked || !active || !(editor.draft && editor.draft.can_undo);
    }
    Object.keys(EDITOR_TOOL_BUTTONS).forEach((mode) => {
      const button = els[EDITOR_TOOL_BUTTONS[mode]];
      if (button) button.classList.toggle("is-active", editor.mode === mode);
    });
    if (els.editorRegionSaveBtn) {
      els.editorRegionSaveBtn.classList.toggle("is-active", editor.mode === "region");
    }
    if (els.editorPassageSaveBtn) {
      els.editorPassageSaveBtn.classList.toggle("is-active", editor.mode === "passage");
    }
    if (els.editorName && document.activeElement !== els.editorName) {
      els.editorName.value = (editor.draft && editor.draft.active && editor.draft.name) || "";
      els.editorName.disabled = locked || !active;
    }
    lockMissionDuringEditing(active);
    canvas.style.cursor = editorCursor();
    // 进入 / 退出"区域/通道生成"也要立刻刷新校验结论（否则会一直显示校验通过）
    renderEditorProblems(editor.draft);
    applyEditorMessage();
  }

  function editorCursor() {
    if (!editor.active) return "";
    if (
      editor.mode === "no_go" ||
      editor.mode === "dock" ||
      editor.mode === "static" ||
      editor.mode === "dynamic" ||
      editor.mode === "delete"
    ) {
      return "crosshair";
    }
    return "";
  }

  function editorPointerMode() {
    if (!editor.active || editor.busy) return false;
    return ["no_go", "dock", "static", "dynamic", "delete", "dynamic_path"].includes(
      editor.mode
    );
  }

  function editorStartPath(id) {
    if (!editor.active) {
      editorSay("请先进入编辑（空白地图 / 载入编辑）", "bad");
      return;
    }
    const list = (editor.draft && editor.draft.dynamic) || [];
    if (!list.length) {
      editorSay("还没有动态障碍物：先点“添加动态障碍”在地图上放一个", "bad");
      return;
    }
    editor.mode = "dynamic_path";
    editor.pathTarget = id == null ? null : Number(id);
    editor.pathA = null;
    if (!editor.pathTarget && list.length === 1) editor.pathTarget = list[0].id;
    editorSay(
      editor.pathTarget
        ? "已选中动态障碍 #" + editor.pathTarget + "：请在地图上点巡逻起点 A"
        : "请先点一下地图上的动态障碍物，再点巡逻起点 A",
      "ok"
    );
    updateEditorUI();
  }

  function editorDynamicPathClick(point) {
    const list = (editor.draft && editor.draft.dynamic) || [];
    if (!list.length) {
      editorSay("还没有动态障碍物，请先添加", "bad");
      return;
    }
    if (!editor.pathTarget) {
      let best = null;
      let bestDistance = Infinity;
      list.forEach((item) => {
        const distance = Math.min(
          Math.hypot(point.x - item.x, point.y - item.y),
          Math.hypot(point.x - item.ax, point.y - item.ay),
          Math.hypot(point.x - item.bx, point.y - item.by)
        );
        if (distance < bestDistance) {
          bestDistance = distance;
          best = item;
        }
      });
      if (!best || bestDistance > Math.max(0.8, editorObstacleRadius() + 0.5)) {
        editorSay("没点中动态障碍物，请点在它的圆点上", "bad");
        return;
      }
      editor.pathTarget = best.id;
      editor.pathA = null;
      editorSay("已选中动态障碍 #" + best.id + "：请点巡逻起点 A", "ok");
      return;
    }
    if (!editor.pathA) {
      editor.pathA = [editorRound(point.x), editorRound(point.y)];
      editorSay("起点 A 已记录，请点终点 B（至少 0.4 m）", "ok");
      return;
    }
    const start = editor.pathA;
    const id = editor.pathTarget;
    editor.pathA = null;
    editorCall(
      "dynamic",
      {
        id: id,
        ax: start[0],
        ay: start[1],
        bx: editorRound(point.x),
        by: editorRound(point.y),
        speed: editorPatrolSpeed(),
      },
      () => {
        editor.mode = "idle";
        editor.pathTarget = null;
        editorSay(
          "动态障碍 #" +
            id +
            " 巡逻路径已更新：A(" +
            start[0].toFixed(2) +
            ", " +
            start[1].toFixed(2) +
            ") → B(" +
            point.x.toFixed(2) +
            ", " +
            point.y.toFixed(2) +
            ")，速度 " +
            editorPatrolSpeed().toFixed(2) +
            " m/s",
          "ok"
        );
      }
    );
  }

  function editorPointerDown(worldPoint, event) {
    if (editor.mode === "no_go") {
      editor.rect = {
        x0: worldPoint.x,
        y0: worldPoint.y,
        x1: worldPoint.x,
        y1: worldPoint.y,
      };
      if (event && event.pointerId != null) canvas.setPointerCapture(event.pointerId);
      return;
    }
    if (editor.mode === "static") {
      editor.painting = [[editorRound(worldPoint.x), editorRound(worldPoint.y)]];
      if (event && event.pointerId != null) canvas.setPointerCapture(event.pointerId);
      return;
    }
    if (editor.mode === "dock") {
      editorCall(
        "dock",
        {
          x: editorRound(worldPoint.x),
          y: editorRound(worldPoint.y),
          r: editorDockRadius(),
        },
        (response) =>
          editorSay(
            "充电桩 #" +
              response.dock.id +
              " 已放置 (" +
              response.dock.x.toFixed(2) +
              ", " +
              response.dock.y.toFixed(2) +
              ")",
            "ok"
          )
      );
      return;
    }
    if (editor.mode === "dynamic") {
      editorCall(
        "obstacle",
        {
          kind: "dynamic",
          x: editorRound(worldPoint.x),
          y: editorRound(worldPoint.y),
          r: editorObstacleRadius(),
          span: Math.max(0.4, editorObstacleRadius() * 4),
          heading: 0,
          speed: editorPatrolSpeed(),
        },
        (response) =>
          editorSay(
            "动态障碍 #" +
              response.added.id +
              " 已添加（巡逻速度 " +
              editorPatrolSpeed().toFixed(2) +
              " m/s），可点“设置路径”改 A→B",
            "ok"
          )
      );
      return;
    }
    if (editor.mode === "dynamic_path") {
      editorDynamicPathClick(worldPoint);
      return;
    }
    if (editor.mode === "delete") {
      editorCall(
        "delete",
        { x: editorRound(worldPoint.x), y: editorRound(worldPoint.y) },
        (response) => {
          const removed = (response.result && response.result.removed_passages) || 0;
          editorSay(
            "已删除元素" + (removed ? "（同时移除 " + removed + " 条通道）" : ""),
            "ok"
          );
        }
      );
    }
  }

  function editorPointerMove(worldPoint) {
    if (editor.rect) {
      editor.rect = {
        x0: Math.min(editor.rect.x0, worldPoint.x),
        y0: Math.min(editor.rect.y0, worldPoint.y),
        x1: Math.max(editor.rect.x0, worldPoint.x),
        y1: Math.max(editor.rect.y0, worldPoint.y),
      };
      return;
    }
    if (editor.painting.length) {
      const last = editor.painting[editor.painting.length - 1];
      if (
        Math.hypot(worldPoint.x - last[0], worldPoint.y - last[1]) >=
        Math.max(0.05, editorBrushRadius() * 0.4)
      ) {
        editor.painting.push([
          editorRound(worldPoint.x),
          editorRound(worldPoint.y),
        ]);
      }
    }
  }

  function editorCommitRect() {
    const rect = editor.rect;
    editor.rect = null;
    if (!rect) return;
    if (rect.x1 - rect.x0 < 0.4 || rect.y1 - rect.y0 < 0.4) {
      editorSay("禁区太小（至少 0.4 m × 0.4 m），请重新框选", "bad");
      return;
    }
    editorSay("正在添加禁区…");
    editorCall(
      "no_go",
      {
        x0: editorRound(rect.x0),
        y0: editorRound(rect.y0),
        x1: editorRound(rect.x1),
        y1: editorRound(rect.y1),
      },
      (response) => editorSay("禁区 #" + response.no_go.id + " 已添加", "ok")
    );
  }

  function editorCommitPainting() {
    const points = editor.painting;
    editor.painting = [];
    if (!points.length) return;
    const radius = editorBrushRadius();
    editorSay("正在添加障碍物…");
    editorCall(
      "obstacle",
      { kind: "static", points, r: radius },
      (response) => {
        const added = response.added || {};
        const length = added.length != null ? added.length.toFixed(1) : "0.0";
        editorSay(
          "已添加实心障碍 #" +
            added.id +
            "（长 " +
            length +
            " m，宽 " +
            (radius * 2).toFixed(2) +
            " m）",
          "ok"
        );
      }
    );
  }

  function editorCommitRegion() {
    if (editor.mode !== "region") {
      editorSay("请先点“选择区域起点”，再用手动控制画出区域边界", "bad");
      return;
    }
    if (editor.trace.length < 4) {
      editorSay("轨迹太短：请用手动控制让小车沿区域边界绕一整圈再保存", "bad");
      return;
    }
    editorCall("region", { points: editor.trace }, (response) => {
      const area = response.region && response.region.area ? response.region.area : 0;
      editor.mode = "idle";
      editor.trace = [];
      editorSay(
        "区域 #" + response.region.id + " 保存成功，面积 " + area.toFixed(1) + " m²",
        "ok"
      );
    });
  }

  function editorCommitPassage() {
    if (editor.mode !== "passage") {
      editorSay("请先点“选择通道起点”，再把小车从区域 A 开到区域 B", "bad");
      return;
    }
    if (editor.trace.length < 2) {
      editorSay("轨迹太短：通道需要连接两个不相交的区域", "bad");
      return;
    }
    editorCall("passage", { points: editor.trace }, (response) => {
      editor.mode = "idle";
      editor.trace = [];
      editorSay(
        (response.passage.id == null ? "通道" : "通道 #" + response.passage.id) +
          " 已保存：区域 #" +
          response.passage.from +
          " ↔ 区域 #" +
          response.passage.to,
        "ok"
      );
    });
  }

  function editorEnter(mapId) {
    editor.mode = "idle";
    editor.trace = [];
    editor.painting = [];
    editor.rect = null;
    editor.pathA = null;
    editor.pathTarget = null;
    const payload = {};
    if (mapId != null) payload.map = Number(mapId);
    const name = els.editorName ? els.editorName.value.trim() : "";
    if (name) payload.name = name;
    editorSay(mapId == null ? "正在创建空白地图…" : "正在载入地图…");
    editorCall("blank", payload, (response) => {
      editorApplyDraft(response.draft);
      editorZoomToDraft(response.draft);
      editorSay(
        mapId == null
          ? "已进入编辑：空白场地 " +
              response.draft.width.toFixed(0) +
              " × " +
              response.draft.height.toFixed(0) +
              " m，保存时按画出的范围裁剪"
          : "已载入地图「" + response.draft.name + "」，改完点“保存地图”另存为新地图",
        "ok"
      );
    });
  }

  function editorBlank() {
    editorEnter(null);
  }

  function editorLoadSelected() {
    if (!els.editorSourceSelect) return;
    const value = els.editorSourceSelect.value;
    if (!value) {
      editorSay("请先在下拉框里选一张已有地图（要画新图请点“空白地图”）", "bad");
      return;
    }
    if (editor.active) {
      editorSay("正在编辑中：请先保存或退出编辑，再载入其它地图", "bad");
      return;
    }
    editorEnter(value);
  }

  function editorZoomToDraft(draft) {
    const emptyCanvas = !draft || !(draft.regions || []).length;
    if (emptyCanvas) {
      // 空白场地有 100 × 100 m，整张铺满屏幕上什么都看不清：
      // 默认只给机器人周围 32 m 的取景框，画出来的东西一眼能看见
      const pose = (current && current.pose) || { x: draft ? draft.width / 2 : 50, y: draft ? draft.height / 2 : 50 };
      const worldWidth = (current && current.world && current.world.width) || 100;
      const worldHeight = (current && current.world && current.world.height) || 100;
      const rect = canvas.getBoundingClientRect();
      const pad = 70;
      const viewWidth = Math.max(80, rect.width - pad);
      const viewHeight = Math.max(80, rect.height - pad);
      const desired = Math.min(viewWidth / 32, viewHeight / 32);
      const base = Math.min(viewWidth / worldWidth, viewHeight / worldHeight);
      viewZoom = Math.max(0.4, Math.min(24, desired / base));
      viewCenterX = pose.x;
      viewCenterY = pose.y;
      viewReady = true;
      viewWorldKey =
        (current ? current.map_id : "none") +
        ":" +
        worldWidth.toFixed(2) +
        "x" +
        worldHeight.toFixed(2);
      return;
    }
    const bounds = draft && draft.bounds;
    if (!bounds || bounds.length < 4) return;
    const [x0, y0, x1, y1] = bounds;
    const worldWidth = (current && current.world && current.world.width) || 100;
    const worldHeight = (current && current.world && current.world.height) || 100;
    const spanX = Math.max(1.0, x1 - x0);
    const spanY = Math.max(1.0, y1 - y0);
    // 让草稿外接框铺满画布：先算"铺满画布"和"铺满整张地图"的像素比例，再取比值
    const rect = canvas.getBoundingClientRect();
    const pad = 70;
    const viewWidth = Math.max(80, rect.width - pad);
    const viewHeight = Math.max(80, rect.height - pad);
    const desired = Math.min(viewWidth / spanX, viewHeight / spanY);
    const base = Math.min(viewWidth / worldWidth, viewHeight / worldHeight);
    viewZoom = Math.max(0.4, Math.min(20, (desired / base) * 0.92));
    viewCenterX = (x0 + x1) / 2;
    viewCenterY = (y0 + y1) / 2;
    viewReady = true;
    // 记住当前的 worldKey，否则下一帧会因为"换了场地"重新 resetView，把缩放冲掉
    viewWorldKey =
      (current ? current.map_id : "none") +
      ":" +
      worldWidth.toFixed(2) +
      "x" +
      worldHeight.toFixed(2);
  }

  function editorSave() {
    const draft = editor.draft;
    if (!draft || !draft.active) {
      editorSay("当前没有正在编辑的地图", "bad");
      return;
    }
    const problems = editorPendingProblems().concat((draft && draft.problems) || []);
    if (problems.length) {
      editorSay("还不能保存：" + problems.join("；"), "bad");
      return;
    }
    const typed = els.editorName ? els.editorName.value.trim() : "";
    editorSay("正在保存地图…");
    editorCall("save", { name: typed || draft.name }, (response) => {
      editor.mode = "idle";
      editorSay(
        "地图 #" + response.map_id + "「" + response.name + "」保存成功，已自动切换过去",
        "ok"
      );
    });
  }

  function editorExit() {
    if (!editor.active) return;
    editor.mode = "idle";
    editor.trace = [];
    editor.painting = [];
    editor.rect = null;
    editorCall("discard", {}, () => editorSay("已退出编辑，未保存的编辑内容已丢弃", "ok"));
  }

  function bindEditor() {
    if (!els.editorBlankBtn) return;
    els.editorBlankBtn.addEventListener("click", editorBlank);
    els.editorSaveBtn.addEventListener("click", editorSave);
    els.editorExitBtn.addEventListener("click", editorExit);
    els.editorDeleteBtn.addEventListener("click", () => editorSetMode("delete"));
    els.editorNoGoBtn.addEventListener("click", () => editorSetMode("no_go"));
    els.editorDockBtn.addEventListener("click", () => editorSetMode("dock"));
    els.editorStaticBtn.addEventListener("click", () => editorSetMode("static"));
    els.editorDynamicBtn.addEventListener("click", () => editorSetMode("dynamic"));
    els.editorRegionStartBtn.addEventListener("click", () => editorStartTrace("region"));
    els.editorRegionSaveBtn.addEventListener("click", editorCommitRegion);
    els.editorRegionCancelBtn.addEventListener("click", editorCancelTrace);
    els.editorPassageStartBtn.addEventListener("click", () => editorStartTrace("passage"));
    els.editorPassageSaveBtn.addEventListener("click", editorCommitPassage);
    els.editorPassageCancelBtn.addEventListener("click", editorCancelTrace);
    if (els.editorLoadBtn) els.editorLoadBtn.addEventListener("click", editorLoadSelected);
    if (els.editorUndoBtn) {
      els.editorUndoBtn.addEventListener("click", () => {
        if (!editor.active) {
          editorSay("请先进入编辑（空白地图 / 载入编辑）", "bad");
          return;
        }
        editor.mode = "idle";
        editor.trace = [];
        editor.painting = [];
        editor.rect = null;
        editor.pathA = null;
        editorCall("undo", {}, (response) =>
          editorSay(
            "已撤销上一步编辑" +
              (response.remaining > 0 ? "（还可再撤销 " + response.remaining + " 步）" : ""),
            "ok"
          )
        );
      });
    }
    renderEditorDraftInfo(editor.draft);
    updateEditorUI();
  }

  function editorPointInPolygon(x, y, points) {
    if (!points || points.length < 3) return false;
    let inside = false;
    const count = points.length;
    let j = count - 1;
    for (let i = 0; i < count; i += 1) {
      const xi = points[i][0];
      const yi = points[i][1];
      const xj = points[j][0];
      const yj = points[j][1];
      if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi + 1e-12) + xi) {
        inside = !inside;
      }
      j = i;
    }
    return inside;
  }

  function drawEditorDraft(geom, draft) {
    if (!draft || !draft.active) return;
    ctx.save();
    (draft.regions || []).forEach((region) => {
      const pts = region.points || [];
      if (pts.length < 3) return;
      ctx.beginPath();
      pts.forEach((point, index) => {
        const x = geom.x(point[0]);
        const y = geom.y(point[1]);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.closePath();
      ctx.fillStyle = "rgba(" + palette.green + ", 0.10)";
      ctx.fill();
      ctx.setLineDash([7, 5]);
      ctx.lineWidth = 1.6;
      ctx.strokeStyle = "rgba(" + palette.green + ", 0.85)";
      ctx.stroke();
      ctx.setLineDash([]);
      const cx = pts.reduce((sum, point) => sum + point[0], 0) / pts.length;
      const cy = pts.reduce((sum, point) => sum + point[1], 0) / pts.length;
      ctx.font = "600 11px 'Segoe UI', sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.lineWidth = 3;
      ctx.strokeStyle = palette.outlineHalo;
      ctx.strokeText("区域 #" + region.id, geom.x(cx), geom.y(cy));
      ctx.fillStyle = "rgba(" + palette.green + ", 0.95)";
      ctx.fillText("区域 #" + region.id, geom.x(cx), geom.y(cy));
    });

    (draft.passages || []).forEach((passage) => {
      const pts = passage.points || [];
      if (pts.length < 2) return;
      const regions = (draft.regions || []).map((region) => region.points || []);
      // 把通道折线切成"压在区域上"和"纯通道"两种段：区域内的照常割（用区域色），
      // 纯通道段用琥珀色标出来
      const runs = [];
      let current = null;
      const pushPoint = (point, inside) => {
        if (!current || current.inside !== inside) {
          if (current) current.points.push(point);
          current = { inside, points: [point] };
          runs.push(current);
        } else {
          current.points.push(point);
        }
      };
      for (let index = 0; index < pts.length - 1; index += 1) {
        const ax = pts[index][0];
        const ay = pts[index][1];
        const bx = pts[index + 1][0];
        const by = pts[index + 1][1];
        const steps = Math.max(1, Math.round(Math.hypot(bx - ax, by - ay) / 0.25));
        for (let k = 0; k < steps; k += 1) {
          const x = ax + (bx - ax) * (k / steps);
          const y = ay + (by - ay) * (k / steps);
          const inside = regions.some((poly) => editorPointInPolygon(x, y, poly));
          pushPoint([x, y], inside);
        }
      }
      pushPoint(pts[pts.length - 1], regions.some((poly) => editorPointInPolygon(pts[pts.length - 1][0], pts[pts.length - 1][1], poly)));
      const width = Math.max(3, passage.half * 2 * geom.scale);
      ctx.lineCap = "butt";
      runs.forEach((run) => {
        if (run.points.length < 2) return;
        ctx.beginPath();
        run.points.forEach((point, index) => {
          const x = geom.x(point[0]);
          const y = geom.y(point[1]);
          if (index === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.lineWidth = width;
        ctx.strokeStyle = run.inside
          ? "rgba(" + palette.green + ", 0.30)"
          : "rgba(255, 196, 92, 0.55)";
        ctx.stroke();
        if (!run.inside) {
          ctx.setLineDash([4, 4]);
          ctx.lineWidth = 1;
          ctx.strokeStyle = "rgba(255, 196, 92, 0.95)";
          ctx.stroke();
          ctx.setLineDash([]);
        }
      });
      // 纯通道段的中点写文字说明
      let best = null;
      runs.forEach((run) => {
        if (run.inside || run.points.length < 2) return;
        if (!best || run.points.length > best.points.length) best = run;
      });
      if (best) {
        const mid = best.points[Math.floor(best.points.length / 2)];
        const label =
          passage.id == null || (draft.passages || []).length < 2
            ? "通道"
            : "通道 #" + passage.id;
        ctx.font = "600 10px 'Segoe UI', sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.lineWidth = 3;
        ctx.strokeStyle = palette.outlineHalo;
        ctx.strokeText(label, geom.x(mid[0]), geom.y(mid[1]) - 10);
        ctx.fillStyle = "rgba(255, 209, 102, 0.98)";
        ctx.fillText(label, geom.x(mid[0]), geom.y(mid[1]) - 10);
      }
    });

    (draft.no_gos || []).forEach((zone) => {
      const center = strokeNoGoRect(geom, noGoRect(zone), true, 0.14);
      ctx.font = "600 10px 'Segoe UI', sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      const label = "禁区 #" + zone.id;
      ctx.lineWidth = 3;
      ctx.strokeStyle = palette.outlineHalo;
      ctx.strokeText(label, center.x, center.y);
      ctx.fillStyle = "rgba(" + palette.red + ", 0.95)";
      ctx.fillText(label, center.x, center.y);
    });

    (draft.poly_obstacles || []).forEach((item) => {
      const pts = item.points || [];
      if (pts.length < 3) return;
      ctx.beginPath();
      pts.forEach((point, index) => {
        const x = geom.x(point[0]);
        const y = geom.y(point[1]);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.closePath();
      ctx.fillStyle = "rgba(" + palette.obstacleLarge + ", 0.30)";
      ctx.fill();
      ctx.lineWidth = 1.4;
      ctx.strokeStyle = "rgba(" + palette.obstacleLarge + ", 0.9)";
      ctx.stroke();
    });

    const brushOutlines = draft.brush_outlines || [];
    brushOutlines.forEach((loop) => {
      if (!loop || loop.length < 3) return;
      ctx.beginPath();
      loop.forEach((point, index) => {
        const x = geom.x(point[0]);
        const y = geom.y(point[1]);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.closePath();
      ctx.fillStyle = "rgba(" + palette.obstacleLarge + ", 0.55)";
      ctx.fill();
      ctx.lineWidth = 1.4;
      ctx.strokeStyle = "rgba(" + palette.obstacleLarge + ", 0.95)";
      ctx.stroke();
    });

    (brushOutlines.length ? [] : draft.static || []).forEach((item) => {
      const pts = item.points || [];
      if (!pts.length) return;
      const width = Math.max(3, item.r * 2 * geom.scale);
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      if (pts.length === 1) {
        ctx.beginPath();
        ctx.arc(geom.x(pts[0][0]), geom.y(pts[0][1]), width / 2, 0, Math.PI * 2);
        ctx.fillStyle = "rgba(" + palette.obstacleLarge + ", 0.55)";
        ctx.fill();
      } else {
        ctx.beginPath();
        pts.forEach((point, index) => {
          const x = geom.x(point[0]);
          const y = geom.y(point[1]);
          if (index === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.strokeStyle = "rgba(" + palette.obstacleLarge + ", 0.62)";
        ctx.lineWidth = width;
        ctx.stroke();
      }
      ctx.lineCap = "butt";
      ctx.lineJoin = "miter";
    });

    (draft.dynamic || []).forEach((item) => {
      if (item.ax != null) {
        ctx.beginPath();
        ctx.moveTo(geom.x(item.ax), geom.y(item.ay));
        ctx.lineTo(geom.x(item.bx), geom.y(item.by));
        ctx.setLineDash([3, 5]);
        ctx.lineWidth = 1.2;
        ctx.strokeStyle = "rgba(" + palette.cyan + ", 0.55)";
        ctx.stroke();
        ctx.setLineDash([]);
      }
      ctx.beginPath();
      ctx.arc(geom.x(item.x), geom.y(item.y), Math.max(2, item.r * geom.scale), 0, Math.PI * 2);
      ctx.fillStyle = "rgba(" + palette.cyan + ", 0.30)";
      ctx.fill();
      ctx.lineWidth = 1.4;
      ctx.strokeStyle = "rgba(" + palette.cyan + ", 0.9)";
      ctx.stroke();
    });

    (draft.docks || []).forEach((dock) => {
      ctx.beginPath();
      ctx.arc(geom.x(dock.x), geom.y(dock.y), Math.max(3, dock.r * geom.scale), 0, Math.PI * 2);
      ctx.fillStyle = "rgba(" + palette.green + ", 0.22)";
      ctx.fill();
      ctx.lineWidth = 1.8;
      ctx.strokeStyle = "rgba(" + palette.green + ", 0.95)";
      ctx.stroke();
      ctx.font = "600 10px 'Segoe UI', sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.lineWidth = 3;
      ctx.strokeStyle = palette.outlineHalo;
      const labelX = geom.x(dock.x);
      const labelY = geom.y(dock.y) - Math.max(6, dock.r * geom.scale) - 7;
      ctx.strokeText("充电桩", labelX, labelY);
      ctx.fillStyle = palette.dockIcon;
      ctx.fillText("充电桩", labelX, labelY);
    });

    if (editor.trace.length > 1) {
      ctx.beginPath();
      editor.trace.forEach((point, index) => {
        const x = geom.x(point[0]);
        const y = geom.y(point[1]);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.setLineDash([6, 4]);
      ctx.lineWidth = 2;
      ctx.strokeStyle = "rgba(" + palette.cyan + ", 0.95)";
      ctx.stroke();
      ctx.setLineDash([]);
      const first = editor.trace[0];
      ctx.beginPath();
      ctx.arc(geom.x(first[0]), geom.y(first[1]), 4, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(" + palette.cyan + ", 0.95)";
      ctx.fill();
    }

    if (editor.painting.length) {
      // 落笔过程中同样画成实心带（不是一个接一个的圆点）
      const width = Math.max(3, editorBrushRadius() * 2 * geom.scale);
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.beginPath();
      editor.painting.forEach((point, index) => {
        const x = geom.x(point[0]);
        const y = geom.y(point[1]);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      if (editor.painting.length === 1) {
        const only = editor.painting[0];
        ctx.arc(geom.x(only[0]), geom.y(only[1]), width / 2, 0, Math.PI * 2);
        ctx.fillStyle = "rgba(" + palette.obstacleLarge + ", 0.5)";
        ctx.fill();
      } else {
        ctx.strokeStyle = "rgba(" + palette.obstacleLarge + ", 0.5)";
        ctx.lineWidth = width;
        ctx.stroke();
      }
      ctx.lineCap = "butt";
      ctx.lineJoin = "miter";
    }

    if (editor.rect) {
      strokeNoGoRect(geom, editor.rect, false, 0.16);
    }

    if (editor.pathA) {
      const startX = geom.x(editor.pathA[0]);
      const startY = geom.y(editor.pathA[1]);
      ctx.beginPath();
      ctx.arc(startX, startY, 4, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(" + palette.cyan + ", 0.95)";
      ctx.fill();
      if (hoverWorld) {
        ctx.beginPath();
        ctx.moveTo(startX, startY);
        ctx.lineTo(geom.x(hoverWorld.x), geom.y(hoverWorld.y));
        ctx.setLineDash([5, 4]);
        ctx.lineWidth = 1.6;
        ctx.strokeStyle = "rgba(" + palette.cyan + ", 0.9)";
        ctx.stroke();
        ctx.setLineDash([]);
      }
      ctx.font = "600 10px 'Segoe UI', sans-serif";
      ctx.textAlign = "center";
      ctx.fillStyle = "rgba(" + palette.cyan + ", 0.95)";
      ctx.fillText("A", startX, startY - 8);
    }
    ctx.restore();
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
    const passage = map.passage || "";
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
      if (passage && passage.charCodeAt(index) === 49 && occ !== 50) {
        // 纯通道（不在任何区域里）：只通行不割草，用琥珀色标出来
        data[pixel] = 255;
        data[pixel + 1] = 196;
        data[pixel + 2] = 92;
        data[pixel + 3] = mowed.charCodeAt(index) === 49 ? 70 : 58;
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

  // “区域覆盖”面板已取消：这里只保留"割草区域"输入框的自动占位提示
  function updateZones(zones) {
    const joined = zones.map((zone) => zone.id).join(",");
    if (zones.length && joined !== zoneAutoValue) {
      zoneAutoValue = joined;
      // 只在占位符里提示"自动"会用哪些区域，绝不往输入框里写值：
      // 一旦自动填成 "1,2,3"，后端就会当成用户指定顺序，按路程就近排序永远不生效。
      if (!zoneInputDirty) {
        els.zoneOrder.value = "";
        els.zoneOrder.placeholder = "自动（" + joined + "）";
      }
    }
  }

  function updateMission(task, state) {
    // 回放任务的提示（"开始回放日志 xxx：… 时长 149.6 s"）在任务栏里太长，
    // 回放信息已经由右边的回放条显示，这里不再重复（空消息整块隐藏，给回放条腾地方）
    const message = task && task.type === "replay" ? "" : task.message || "";
    els.missionMessage.textContent = message;
    els.missionMessage.hidden = !message;
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
      drawEditorDraft(geom, state.draft);
      drawDynamicPaths(geom, (state.world && state.world.dynamic) || []);
      drawObstacles(geom, state.obstacles || []);
      drawPaths(geom, state);
      drawCostmap(geom);
      drawCloud(geom);
      drawTrail(geom);
      // 编辑草稿自带充电桩，临时场地的占位桩不画，避免出现两个桩
      drawPassageLabels(geom, state.map || lastMapPayload, world);
      if (!(state.draft && state.draft.active)) drawDock(geom, world.dock, state.charging);
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
      if (item.points && item.points.length) {
        const tail = item.points[item.points.length - 1];
        items.push({ text: "#" + item.id, x: tail[0], y: tail[1] + item.r + 0.20 });
      } else {
        items.push({ text: "#" + item.id, x: item.x, y: item.y + item.r + 0.20 });
      }
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

  function mapPassageAt(map, x, y) {
    if (!map || !map.passage) return false;
    const cell = map.cell || 0.1;
    const col = Math.floor(x / cell);
    const row = Math.floor(y / cell);
    if (col < 0 || col >= map.cols || row < 0 || row >= map.rows) return false;
    return map.passage.charCodeAt(row * map.cols + col) === 49;
  }

  function drawPassageLabels(geom, map, world) {
    const passages = (world && world.passages) || [];
    if (!map || !map.passage || !passages.length) return;
    ctx.save();
    ctx.font = "600 10px 'Segoe UI', sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    passages.forEach((passage) => {
      const points = passage.points || [];
      // 沿折线找一个"纯通道"点，在那里标字（压在区域上的那段是照常割的，不标）
      let spot = null;
      for (let index = 0; index < points.length - 1 && !spot; index += 1) {
        const ax = points[index][0];
        const ay = points[index][1];
        const bx = points[index + 1][0];
        const by = points[index + 1][1];
        const steps = Math.max(1, Math.round(Math.hypot(bx - ax, by - ay) / 0.4));
        for (let k = 0; k <= steps; k += 1) {
          const x = ax + (bx - ax) * (k / steps);
          const y = ay + (by - ay) * (k / steps);
          if (mapPassageAt(map, x, y)) {
            spot = { x, y };
            break;
          }
        }
      }
      if (!spot) return;
      // 只有一条通道时就写"通道"；多条才带编号，且编号缺失时不拼 #undefined
      const label =
        passage.id == null || passages.length < 2 ? "通道" : "通道 #" + passage.id;
      const x = geom.x(spot.x);
      const y = geom.y(spot.y) - 10;
      ctx.lineWidth = 3;
      ctx.strokeStyle = palette.outlineHalo;
      ctx.strokeText(label, x, y);
      ctx.fillStyle = "rgba(255, 209, 102, 0.95)";
      ctx.fillText(label, x, y);
    });
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
        // 栅格行 0 是最小 y，而画布第 0 行在最上面：写图时翻转过来，
        // 这样按 origin 直接叠上去才和真实地图重合（之前漏了这一步，
        // 整块代价地图上下镜像，看着像"转了一个角度"）。
        const offset = ((height - 1 - row) * width + col) * 4;
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
    layer.flipRows = true;  // 已在上面的 offset 里翻转过
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
    // 图像已经翻成"第 0 行在最上"，按 origin 左上角直接贴即可
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
      // 连续播放换日志处重新起笔：不跟上一段连线
      if (point.gap) continue;
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
      send("/api/command", manualCommandPayload(command));
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
    send("/api/command", manualCommandPayload(command));
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
    if (!zones.length) {
      // 留空 = 让后端按"离车最近"自动排区域顺序（省路程）
      els.zoneOrder.placeholder = zoneAutoValue
        ? "自动（" + zoneAutoValue + "）"
        : "自动";
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
    [els.importFileBtn, els.importAllMapsBtn].forEach((button) => {
      if (button) button.disabled = busy;
    });
    if (message) els.importMapMessage.textContent = message;
  }

  async function importAllMaps() {
    setImportBusy(true, "正在扫描 data/map 并导入…");
    try {
      const result = await request("/api/map/scan", {});
      const imported = result.imported || [];
      const failures = result.failures || [];
      trail = [];
      mapVersion = -1;
      viewWorldKey = "";
      zoneAutoValue = "";
      zoneInputDirty = false;
      els.zoneOrder.value = "";
      const state = await request("/api/state", null);
      ingest(state);
      if (!imported.length) {
        els.importMapMessage.textContent =
          "没找到可导入的地图（" + (result.directory || "data/map") + "）";
        return;
      }
      const names = imported.map((item) => item.id + " · " + item.name).join("，");
      els.importMapMessage.textContent =
        "已导入 " + imported.length + " 张地图：" + names +
        (failures.length ? "（" + failures.length + " 张失败）" : "") +
        " · 当前使用 " + state.map_name;
    } catch (error) {
      els.importMapMessage.textContent = "自动导入失败：" + error.message;
    } finally {
      setImportBusy(false);
    }
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
  els.importFileInput.addEventListener("change", () => {
    const files = Array.from(els.importFileInput.files || []);
    els.importFileInput.value = "";
    chooseMapFiles(files);
  });
  if (els.importAllMapsBtn) {
    els.importAllMapsBtn.addEventListener("click", importAllMaps);
  }

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

  // 割草报告：任务结束会在 data/data 下写一份文档，这里直接打开最新一份。
  // 必须同步 window.open（放在 await 之后会被浏览器的弹窗拦截器当广告拦掉），
  // 所以这里直接打开服务端的 /reports/latest，由它跳到最新报告。
  if (els.reportBtn) {
    els.reportBtn.addEventListener("click", () => {
      window.open("/reports/latest", "_blank");
    });
  }

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

  // 手动控制的两个参数只是"键盘档位"：按方向键 / 点方向键时才按这两个值下发，
  // 不会自己让机器动起来。
  const MANUAL_SPEED_KEY = "mowerManualSpeedV1";
  const manualSpeed = { linear: 0.55, angular: 1.80 };

  function applyManualSpeedValues() {
    // 手动档位线速度上限 3 m/s（自主任务仍按后端的 MAX_LINEAR 限速）
    manualSpeed.linear = clamp(Math.abs(parseFloat(els.manualLinear.value)) || 0.55, 0.10, 3.00);
    manualSpeed.angular = clamp(Math.abs(parseFloat(els.manualAngular.value)) || 1.80, 0.20, 1.80);
    els.manualLinear.value = manualSpeed.linear.toFixed(2);
    els.manualAngular.value = manualSpeed.angular.toFixed(2);
    els.manualMessage.textContent =
      "键盘速度：前进/后退 " + manualSpeed.linear.toFixed(2) + " m/s · 转向 " +
      manualSpeed.angular.toFixed(2) + " rad/s（按方向键生效）";
    try {
      localStorage.setItem(MANUAL_SPEED_KEY, JSON.stringify(manualSpeed));
    } catch (error) {
      /* ignore */
    }
  }

  function restoreManualSpeed() {
    try {
      const saved = JSON.parse(localStorage.getItem(MANUAL_SPEED_KEY) || "null");
      if (saved && saved.linear) els.manualLinear.value = saved.linear;
      if (saved && saved.angular) els.manualAngular.value = saved.angular;
    } catch (error) {
      /* ignore */
    }
    applyManualSpeedValues();
  }

  function manualCommandPayload(command) {
    if (command === "stop") return { command: "stop" };
    return { command, linear: manualSpeed.linear, angular: manualSpeed.angular };
  }

  if (els.manualLinear && els.manualAngular) {
    restoreManualSpeed();
    [els.manualLinear, els.manualAngular].forEach((input) => {
      input.addEventListener("input", applyManualSpeedValues);
      input.addEventListener("change", applyManualSpeedValues);
      input.addEventListener("keydown", (event) => {
        if (event.key === "Enter") applyManualSpeedValues();
      });
    });
  }

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
  let missionFolded = true;
  try {
    const savedMission = localStorage.getItem(MISSION_KEY);
    // 第一次进界面：任务操作栏也默认收起
    missionFolded = savedMission === null ? true : savedMission === "1";
  } catch (error) {
    missionFolded = true;
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

  // ------------------------------------------------------- 独立窗口（页面内浮层）
  // 以前用 window.open 开浏览器弹窗：一操作主界面/地图，弹窗就被主窗口压到后面看不见了。
  // 现在改成**页面内浮层**（position: fixed + 高 z-index），永远压在界面和地图上层；
  // 可拖动、可拉伸，位置和大小记在 localStorage 里。
  const floatKey = "mower.floatPanels";
  const floatPanels = {};
  const bagPlayerEl = document.querySelector(".bag-player");

  function loadFloatLayout() {
    try {
      return JSON.parse(localStorage.getItem(floatKey) || "{}") || {};
    } catch (error) {
      return {};
    }
  }

  function saveFloatLayout(key, panel) {
    const layout = loadFloatLayout();
    layout[key] = {
      left: panel.offsetLeft,
      top: panel.offsetTop,
      width: panel.offsetWidth,
      height: panel.offsetHeight,
    };
    try {
      localStorage.setItem(floatKey, JSON.stringify(layout));
    } catch (error) {
      /* ignore */
    }
  }

  function popupWindow(key, node, options) {
    const opts = options || {};
    const known = floatPanels[key];
    if (known && known.panel.isConnected) {
      known.panel.style.zIndex = String(nextFloatZ());
      return known.panel;
    }
    const panel = document.createElement("div");
    panel.className = "float-panel";
    panel.dataset.key = key;
    panel.style.zIndex = String(nextFloatZ());

    const bar = document.createElement("div");
    bar.className = "float-panel__bar";
    const title = document.createElement("span");
    title.className = "float-panel__title";
    title.textContent = opts.title || "机器人控制台";
    const back = document.createElement("button");
    back.type = "button";
    back.className = "float-panel__back";
    back.textContent = "收回";
    back.title = "收回到主界面";
    bar.appendChild(title);
    bar.appendChild(back);

    const body = document.createElement("div");
    body.className = "float-panel__body";
    panel.appendChild(bar);
    panel.appendChild(body);
    document.body.appendChild(panel);

    const layout = loadFloatLayout()[key] || {};
    const width = Math.min(layout.width || opts.width || 560, window.innerWidth - 40);
    const height = Math.min(layout.height || opts.height || 420, window.innerHeight - 40);
    panel.style.width = width + "px";
    panel.style.height = height + "px";
    panel.style.left =
      Math.max(0, Math.min(layout.left, window.innerWidth - width - 8) || window.innerWidth - width - 24) + "px";
    panel.style.top = Math.max(0, layout.top || 64) + "px";

    const parent = node.parentNode;
    const next = node.nextSibling;
    node.classList.add("is-popped");
    body.appendChild(node);
    floatPanels[key] = { panel, node, parent, next };

    back.addEventListener("click", () => restorePopup(key));
    panel.addEventListener("pointerdown", () => {
      panel.style.zIndex = String(nextFloatZ());
    });
    makeFloatDraggable(panel, bar);
    refreshPopupBack();
    return panel;
  }

  let floatZ = 9000;
  function nextFloatZ() {
    floatZ += 1;
    return floatZ;
  }

  function makeFloatDraggable(panel, handle) {
    let startX = 0;
    let startY = 0;
    let baseLeft = 0;
    let baseTop = 0;
    let dragging = false;
    handle.addEventListener("pointerdown", (event) => {
      if (event.target.closest("button")) return;
      dragging = true;
      startX = event.clientX;
      startY = event.clientY;
      baseLeft = panel.offsetLeft;
      baseTop = panel.offsetTop;
      handle.setPointerCapture(event.pointerId);
      event.preventDefault();
    });
    handle.addEventListener("pointermove", (event) => {
      if (!dragging) return;
      const left = baseLeft + (event.clientX - startX);
      const top = baseTop + (event.clientY - startY);
      panel.style.left = Math.max(0, Math.min(left, window.innerWidth - 80)) + "px";
      panel.style.top = Math.max(0, Math.min(top, window.innerHeight - 40)) + "px";
    });
    const finish = (event) => {
      if (!dragging) return;
      dragging = false;
      try {
        handle.releasePointerCapture(event.pointerId);
      } catch (error) {
        /* ignore */
      }
      saveFloatLayout(panel.dataset.key, panel);
    };
    handle.addEventListener("pointerup", finish);
    handle.addEventListener("pointercancel", finish);
  }

  function restorePopup(key) {
    const record = floatPanels[key];
    if (!record) return;
    delete floatPanels[key];
    saveFloatLayout(key, record.panel);
    record.node.classList.remove("is-popped");
    if (record.parent && record.parent.isConnected && record.node.parentNode !== record.parent) {
      const anchor = record.next && record.next.parentNode === record.parent ? record.next : null;
      record.parent.insertBefore(record.node, anchor);
    }
    if (record.panel.isConnected) record.panel.remove();
    refreshPopupBack();
  }

  function refreshPopupBack() {
    if (!els.popupBackBtn) return;
    const open = Object.keys(floatPanels).some((key) => floatPanels[key].panel.isConnected);
    els.popupBackBtn.hidden = !open;
  }

  function recallPopups() {
    Object.keys(floatPanels).forEach((key) => restorePopup(key));
  }

  function syncPopups() {
    Object.keys(floatPanels).forEach((key) => {
      if (!floatPanels[key].panel.isConnected) restorePopup(key);
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
    costmap: "日志局部代价地图（窗口随车滚动时机器人会落在窗口里；否则按日志自己的坐标摆放，和真实地图逐格对齐）",
    cloud: "障碍物点云（深度相机 obstaclecloud，已换算到地图坐标，随回放累积）",
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
    // 队列跑完时 index 会等于 total，序号夹住免得显示成 "3/2"
    const index = total ? Math.min(queue.index + 1, total) : 0;
    if (!total) {
      els.bagQueueInfo.textContent = bagOrder.length
        ? "顺序 " + bagOrder.length + " 个"
        : "队列 --";
      return;
    }
    els.bagQueueInfo.textContent = (queue.active ? "连续播放 " : "队列 ") + index + "/" + total;
  }

  // 任务操作栏里的回放条：平时禁用，载入日志后才可用。
  // 直接吃 /api/state 里的 bag 状态，这样即使不打开日志播放面板也会实时更新。
  function renderBagBar(state) {
    const bag = state || {};
    const loaded = !!bag.loaded;
    if (els.bagBar) els.bagBar.classList.toggle("is-idle", !loaded);
    if (els.bagCurrentName) {
      els.bagCurrentName.textContent = loaded && bag.name ? bag.name : "未载入日志";
      els.bagCurrentName.title = loaded && bag.name ? bag.name : "";
    }
    if (els.bagTime) {
      els.bagTime.textContent = loaded
        ? Number(bag.time || 0).toFixed(1) + " / " + Number(bag.duration || 0).toFixed(1) + " s"
        : "--";
    }
    if (els.bagWorld) {
      els.bagWorld.textContent = loaded ? wallClockText(bag.wall_time) : "--";
    }
    const paused = loaded && bag.active && !bag.playing && !bag.finished;
    if (els.bagPlayBtn) {
      els.bagPlayBtn.disabled = !loaded;
      els.bagPlayBtn.textContent = paused ? "继续" : "播放";
    }
    if (els.bagPauseBtn) els.bagPauseBtn.disabled = !loaded || !bag.playing;
    // 停止放在暂停后面：直接退出这次回放（日志仍在，可再点"播放"）
    if (els.bagStopBtn) els.bagStopBtn.disabled = !loaded || (!bag.active && !bag.playing);
    if (els.bagProgress) els.bagProgress.disabled = !loaded;
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
    renderBagBar(state);
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
      // 只有日志里的窗口本身是"随车滚动"发布时，叠加层才会跟着车走
      const follow =
        state.alignment === "costmap" || state.alignment === "map" || state.pose_frame === "map"
          ? " · 随车"
          : " · 按日志坐标";
      els.bagCostInfo.textContent = cost
        ? cost.size[0] + "×" + cost.size[1] + " m · " + (cost.frames || 1) + " 帧" + follow
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
      if (els.bagHint) els.bagHint.classList.toggle("is-paused", paused);
      // 任务栏空间有限：提示只留短句，完整说明放进 title（悬停可见）
      const hint = state.playing
        ? ["回放中", "青色线是仿真实际轨迹，红色细线是跳变（重定位/换日志）"]
        : state.finished
        ? ["回放结束", "可拖动进度条重播或重新载入其他日志"]
        : paused
        ? ["已暂停，点“继续”接着回放", "已暂停，点“继续”从当前位置接着回放"]
        : ["已载入，点“播放”", "已载入，点“播放”开始把日志轨迹映射到仿真机器人上"];
      setBagHint(hint[0] + noMap, 0);
      if (els.bagHint) els.bagHint.title = hint[1] + noMap;
    } else {
      if (els.bagHint) els.bagHint.classList.remove("is-paused");
      setBagHint("选择日志后点“载入”，再点“播放”；勾选多个可“连续播放”。", 0);
    }
    if (els.bagPlayBtn) {
      const paused = state.loaded && state.active && !state.playing && !state.finished;
      els.bagPlayBtn.title = paused
        ? "从暂停位置继续回放"
        : "从头播放（已暂停时可直接继续）";
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

  let cloudPoints = null;        // 截至当前时刻的障碍物点云（地图坐标，扁平的 [x,y,...]）
  let cloudPollAt = 0;
  let cloudTime = -1;

  async function refreshCloud(force) {
    const state = bagState;
    if (!state || !state.loaded) {
      cloudPoints = null;
      return;
    }
    const time = state.time || 0;
    if (!force && !state.playing && Math.abs(time - cloudTime) < 0.3) return;
    const now = performance.now();
    if (!force && now - cloudPollAt < 500) return;
    cloudPollAt = now;
    cloudTime = time;
    try {
      const data = await request("/api/bag/cloud?t=" + time);
      cloudPoints = data.cloud && data.cloud.points ? data.cloud.points : null;
    } catch (error) {
      /* 回放切换/结束期间的请求失败忽略 */
    }
  }

  function drawCloud(geom) {
    if (layerVisible.cloud === false) return;
    if (!cloudPoints || !cloudPoints.length) return;
    ctx.save();
    ctx.fillStyle = "rgba(" + palette.cloud + ", 0.8)";
    const size = Math.max(1.5, geom.scale * 0.02);
    for (let index = 0; index < cloudPoints.length; index += 2) {
      ctx.fillRect(
        geom.x(cloudPoints[index]) - size / 2,
        geom.y(cloudPoints[index + 1]) - size / 2,
        size,
        size
      );
    }
    ctx.restore();
  }

  async function refreshCostmap(force) {
    const state = bagState;
    if (!state || !state.loaded) {
      costmapLayer = null;
      cloudPoints = null;
      return;
    }
    refreshCloud(force);
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
      if (floatPanels.video) restorePopup("video");
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
      if (bagVideoClosed && floatPanels.video) {
        restorePopup("video");
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
      if (floatPanels.video) restorePopup("video");
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
    events: true,
    params: true,
    map: true,
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
  let collapsed = null;
  try {
    collapsed = JSON.parse(localStorage.getItem(COLLAPSE_KEY) || "null");
  } catch (error) {
    collapsed = null;
  }
  if (!collapsed || typeof collapsed !== "object") {
    // 第一次进界面：所有功能栏默认收起，需要时点标题展开，先把地图留给画布
    collapsed = {};
    Object.keys(modulePanels).forEach((key) => {
      collapsed[key] = true;
    });
    collapsed.events = true;
    try {
      localStorage.setItem(COLLAPSE_KEY, JSON.stringify(collapsed));
    } catch (error) {
      /* ignore */
    }
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
    // 完全收起后整块 dock 不占地方，底部留一个小提手再展开（和左侧栏一致）
    if (els.dockTab) els.dockTab.hidden = !logHidden;
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
  function toggleDock() {
    collapsed.events = collapsed.events !== true;
    localStorage.setItem(COLLAPSE_KEY, JSON.stringify(collapsed));
    applyCollapsed();
  }
  if (els.logToggleBtn) {
    els.logToggleBtn.addEventListener("click", (event) => {
      event.stopPropagation();
      toggleDock();
    });
  }
  if (els.dockTab) els.dockTab.addEventListener("click", toggleDock);
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
    if (editorPointerMode() && lastGeometry) {
      editorPointerDown(lastGeometry.toWorld(point.x, point.y), event);
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
    if ((editor.rect || editor.painting.length) && lastGeometry) {
      const point = canvasPoint(event);
      editorPointerMove(lastGeometry.toWorld(point.x, point.y));
      return;
    }
    if (!panState) return;
    const point = canvasPoint(event);
    viewCenterX = panState.centerX - (point.x - panState.screenX) / panState.scale;
    viewCenterY = panState.centerY + (point.y - panState.screenY) / panState.scale;
  });

  function finishPan(event) {
    if (editor.rect || editor.painting.length) {
      if (event && event.pointerId != null && canvas.hasPointerCapture(event.pointerId)) {
        canvas.releasePointerCapture(event.pointerId);
      }
      if (editor.rect) editorCommitRect();
      else editorCommitPainting();
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
    if (editor.rect || editor.painting.length) {
      editor.rect = null;
      editor.painting = [];
      if (event && canvas.hasPointerCapture(event.pointerId)) {
        canvas.releasePointerCapture(event.pointerId);
      }
      return;
    }
    finishPan(event);
  });
  canvas.addEventListener("pointerleave", (event) => {
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
  bindEditor();
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
