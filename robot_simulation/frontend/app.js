(() => {
  "use strict";

  const POLL_MS = 100;
  const POLL_MS_FAST = 50;
  let interpMs = POLL_MS;
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
    batteryText: document.getElementById("batteryText"),
    batteryFill: document.getElementById("batteryFill"),
    chargeRow: document.getElementById("chargeRow"),
    linearSpeed: document.getElementById("linearSpeed"),
    angularSpeed: document.getElementById("angularSpeed"),
    heading: document.getElementById("heading"),
    distance: document.getElementById("distance"),
    collisions: document.getElementById("collisions"),
    simTime: document.getElementById("simTime"),
    leftWheel: document.getElementById("leftWheel"),
    rightWheel: document.getElementById("rightWheel"),
    zoneList: document.getElementById("zoneList"),
    navModeToggle: document.getElementById("navModeToggle"),
    navModeHint: document.getElementById("navModeHint"),
    navNodeList: document.getElementById("navNodeList"),
    navStaticBtn: document.getElementById("navStaticBtn"),
    navStaticHint: document.getElementById("navStaticHint"),
    navStaticList: document.getElementById("navStaticList"),
    navTopicList: document.getElementById("navTopicList"),
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
  let lastSampleTime = -1;
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

    if (lastMapId !== null && state.map_id !== lastMapId) {
      trail = [];
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

    if (state.time + 1e-6 < lastSampleTime) {
      trail = [];
      previous = state;
      mapVersion = -1;
    }
    lastSampleTime = state.time;

    const tail = trail[trail.length - 1];
    if (!tail || Math.hypot(state.pose.x - tail.x, state.pose.y - tail.y) > 0.02) {
      trail.push({ x: state.pose.x, y: state.pose.y });
      if (trail.length > TRAIL_LIMIT) trail.shift();
    }

    if (state.collisions > lastBump) {
      bumpFlashUntil = performance.now() + 650;
    }
    lastBump = state.collisions;

    updateMapImage(state.map);
    updatePanels(state);
  }

  function setConnected(connected) {
    els.linkChip.textContent = connected ? "已连接" : "连接中断";
    els.linkChip.className = "chip " + (connected ? "chip--ok" : "chip--down");
  }

  async function poll() {
    try {
      const state = await request("/api/state");
      ingest(state);
      setConnected(true);
    } catch (error) {
      setConnected(false);
    }
    const speed = current && current.speed ? current.speed : 1;
    const fast = speed >= 4;
    interpMs = fast ? POLL_MS_FAST : POLL_MS;
    window.setTimeout(poll, fast ? POLL_MS_FAST : POLL_MS);
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
    els.simTime.textContent = Math.round(state.time);
    els.leftWheel.textContent = (state.velocity.left_wheel || 0).toFixed(2);
    els.rightWheel.textContent = (state.velocity.right_wheel || 0).toFixed(2);

    updateVision(state.vision || {});
    updateZones(state.zones || []);
    renderNav(state.nav);
    updateMission(task, state);

    renderLog(state.log);
  }

  const NAV_LABELS = { planner: "规划节点", dock: "沿边回充节点" };
  const NAV_STATE_LABELS = { RUNNING: "运行", PAUSE: "暂停", FAIL: "失败", NONE: "空闲" };
  const ACTION_LABELS = {
    nav_task_action: "导航任务",
    autodock_action: "自动回充",
    leave_dock_action: "离桩",
    visual_egway_action: "视觉沿边",
    egway_coverage_action: "沿边覆盖",
  };
  let navSignature = "";

  let navModeTouched = false;

  function syncNavMode(nav) {
    if (!els.navModeToggle || !nav) return;
    const enabled = nav.enabled !== false;
    if (!navModeTouched) els.navModeToggle.checked = enabled;
    els.navModeHint.textContent = enabled
      ? "规划与控制由节点处理"
      : "使用系统内置算法";
    els.navModeHint.classList.toggle("is-off", !enabled);
  }

  function renderNav(nav) {
    if (!els.navNodeList) return;
    syncNavMode(nav);
    if (!nav) {
      if (navSignature === "off") return;
      navSignature = "off";
      els.navNodeList.textContent = "导航节点未启用";
      els.navTopicList.textContent = "";
      return;
    }
    const nodes = nav.nodes || [];
    const topics = nav.topics || [];
    const signature = JSON.stringify([
      nodes.map((node) => [
        node.name,
        node.main_state_name,
        node.sub_state_name,
        (node.actions || []).map((action) => [action.name, action.state, action.last_result]),
      ]),
      topics.map((topic) => [topic.name, topic.messages, topic.last]),
    ]);
    if (signature === navSignature) return;
    navSignature = signature;

    els.navNodeList.textContent = "";
    nodes.forEach((node) => {
      const row = document.createElement("div");
      row.className = "nav-node";
      const head = document.createElement("div");
      head.className = "nav-node-head";
      const title = document.createElement("strong");
      title.textContent =
        (NAV_LABELS[node.name] || node.name) + " · " + String(node.heartbeat_id);
      const state = document.createElement("span");
      const mainState = node.main_state_name || "NONE";
      state.className = "nav-state nav-state--" + mainState.toLowerCase();
      state.textContent =
        (NAV_STATE_LABELS[mainState] || mainState) +
        (node.sub_state_name && node.sub_state_name !== "NONE" ? " / " + node.sub_state_name : "");
      head.append(title, state);
      row.append(head);
      (node.actions || []).forEach((action) => {
        const line = document.createElement("div");
        line.className = "nav-action";
        const label = ACTION_LABELS[action.name] || action.name;
        line.textContent =
          label +
          " @" +
          action.port +
          " · " +
          action.state +
          (action.last_result ? " · " + action.last_result : "");
        row.append(line);
      });
      els.navNodeList.append(row);
    });

    renderNavStatic(nav.static_config);

    els.navTopicList.textContent = "";
    const title = document.createElement("div");
    title.className = "nav-topic-title";
    title.textContent = "话题 " + (nav.topic_total || topics.length) + " 路";
    els.navTopicList.append(title);
    topics.slice(0, 18).forEach((topic) => {
      const line = document.createElement("div");
      line.className = "nav-topic";
      const name = document.createElement("span");
      name.className = "nav-topic-name";
      name.textContent = topic.name;
      const meta = document.createElement("em");
      meta.textContent = topic.messages + " · " + topic.last;
      line.append(name, meta);
      els.navTopicList.append(line);
    });
  }

  const STATIC_LABELS = {
    nav_speed: "导航速度",
    mow_speed: "弓字速度",
    edge_speed: "沿边速度",
    max_linear: "最大线速度",
    max_angular: "最大角速度",
    coverage_speed: "覆盖速度",
    record_interval: "记录间距",
    xy_goal_tolerance: "到点容差",
    region_dilate_size: "区域膨胀",
    ipa_robot_radius: "规划半径",
  };
  let navStaticSignature = "";

  function renderNavStatic(staticConfig) {
    if (!els.navStaticList) return;
    const loaded = !!(staticConfig && staticConfig.loaded);
    const params = (staticConfig && staticConfig.params) || {};
    const signature = JSON.stringify([loaded, staticConfig && staticConfig.source, params]);
    if (signature === navStaticSignature) return;
    navStaticSignature = signature;

    if (els.navStaticHint) {
      els.navStaticHint.textContent = loaded
        ? "已导入 " + (staticConfig.topic_count || 0) + " 话题 / " +
          (staticConfig.service_count || 0) + " 服务"
        : "未导入";
      els.navStaticHint.classList.toggle("is-off", !loaded);
    }
    els.navStaticList.textContent = "";
    if (!loaded) return;
    const title = document.createElement("div");
    title.className = "nav-topic-title";
    title.textContent = "static_config 参数";
    els.navStaticList.append(title);
    Object.keys(params)
      .filter((key) => STATIC_LABELS[key])
      .forEach((key) => {
        const line = document.createElement("div");
        line.className = "nav-topic";
        const name = document.createElement("span");
        name.className = "nav-topic-name";
        name.textContent = STATIC_LABELS[key];
        const meta = document.createElement("em");
        meta.textContent = String(params[key]);
        line.append(name, meta);
        els.navStaticList.append(line);
      });
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
      drawMap(geom, state.map);
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

  function drawTrail(geom) {
    if (trail.length < 2) return;
    ctx.save();
    ctx.lineWidth = Math.max(1, geom.scale * 0.025);
    ctx.lineCap = "round";
    for (let index = 1; index < trail.length; index += 1) {
      const alpha = 0.03 + 0.22 * (index / trail.length);
      ctx.strokeStyle = "rgba(" + palette.cyan + ", " + alpha.toFixed(3) + ")";
      ctx.beginPath();
      ctx.moveTo(geom.x(trail[index - 1].x), geom.y(trail[index - 1].y));
      ctx.lineTo(geom.x(trail[index].x), geom.y(trail[index].y));
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
  els.estopBtn.addEventListener("click", () => send("/api/command", { command: "stop" }));

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
    nodes: true,
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

  // 底部面板页签：事件日志 / 导航节点 切换显示
  const DOCK_TAB_KEY = "mowerDockTabV1";
  let dockTab = "events";
  try {
    dockTab = localStorage.getItem(DOCK_TAB_KEY) || "events";
  } catch (error) {
    dockTab = "events";
  }

  function selectDockTab(name, persist) {
    dockTab = name === "nodes" ? "nodes" : "events";
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
      if (key === "nodes") {
        moduleState.events = true;
        const eventsToggle = document.querySelector('[data-module-toggle="events"]');
        if (eventsToggle) eventsToggle.checked = true;
        selectDockTab(checkbox.checked ? "nodes" : "events");
      }
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
  if (els.navStaticBtn) {
    els.navStaticBtn.addEventListener("click", async () => {
      els.navStaticBtn.disabled = true;
      try {
        const summary = await request("/api/nav/config", {});
        navStaticSignature = "";
        if (els.navStaticHint) {
          els.navStaticHint.textContent = summary.loaded
            ? "已导入 " + (summary.source || "")
            : "未找到 static_config";
        }
        const state = await request("/api/state", null);
        ingest(state);
      } catch (error) {
        if (els.navStaticHint) els.navStaticHint.textContent = "导入失败：" + error.message;
      } finally {
        els.navStaticBtn.disabled = false;
      }
    });
  }
  if (els.navModeToggle) {
    els.navModeToggle.addEventListener("change", async () => {
      navModeTouched = true;
      const enabled = els.navModeToggle.checked;
      if (els.navModeHint) {
        els.navModeHint.textContent = enabled ? "规划与控制由节点处理" : "使用系统内置算法";
      }
      try {
        const state = await request("/api/nav/mode", { enabled });
        ingest(state);
      } catch (error) {
        els.navModeHint.textContent = "切换失败：" + error.message;
      } finally {
        navModeTouched = false;
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

  function frame(now) {
    draw(now);
    requestAnimationFrame(frame);
  }

  requestAnimationFrame(frame);
  poll();
})();
