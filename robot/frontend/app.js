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
    log: document.getElementById("log"),
    phaseChip: document.getElementById("phaseChip"),
    zoneChip: document.getElementById("zoneChip"),
    mapFill: document.getElementById("mapFill"),
    mapText: document.getElementById("mapText"),
    mowFill: document.getElementById("mowFill"),
    mowText: document.getElementById("mowText"),
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
    applyParamsBtn: document.getElementById("applyParamsBtn"),
    resetParamsBtn: document.getElementById("resetParamsBtn"),
    paramMessage: document.getElementById("paramMessage"),
    collapseAllBtn: document.getElementById("collapseAllBtn"),
    expandAllBtn: document.getElementById("expandAllBtn"),
    visionSpec: document.getElementById("visionSpec"),
  };

  const visionEls = {};
  document.querySelectorAll("[data-vision]").forEach((row) => {
    visionEls[row.dataset.vision] = {
      bar: row.querySelector(".sensor-bar i"),
      value: row.querySelector(".sensor-value"),
    };
  });

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
  let mapOptionsBuilt = false;
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
    if (mapOptionsBuilt) return;
    els.mapSelect.textContent = "";
    maps.forEach((item) => {
      const option = document.createElement("option");
      option.value = String(item.id);
      option.textContent = item.id + " · " + item.name;
      els.mapSelect.append(option);
    });
    mapOptionsBuilt = true;
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
    for (let index = 0; index < map.cols * map.rows; index += 1) {
      const pixel = index * 4;
      const occ = occupancy.charCodeAt(index);
      if (mowed.charCodeAt(index) === 49) {
        data[pixel] = 55;
        data[pixel + 1] = 224;
        data[pixel + 2] = 160;
        data[pixel + 3] = 105;
      } else if (occ === 50) {
        data[pixel] = 255;
        data[pixel + 1] = 132;
        data[pixel + 2] = 86;
        data[pixel + 3] = 200;
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
    els.distance.textContent = state.distance.toFixed(2);
    els.collisions.textContent = String(state.collisions);
    els.simTime.textContent = state.time.toFixed(1);
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
    els.mowFill.style.width = (coverage * 100).toFixed(2) + "%";
    els.mowText.textContent = (coverage * 100).toFixed(2) + "%";
    els.overallFill.style.width = (overall * 100).toFixed(2) + "%";
    els.overallText.textContent = (overall * 100).toFixed(2) + "%";
    els.taskDistance.textContent = Number(task.distance || state.distance || 0).toFixed(2);
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
  function makeGeometry(world, width, height) {
    const padding = 30;
    const scale = Math.min(
      (width - padding * 2) / world.width,
      (height - padding * 2) / world.height
    );
    const ox = (width - world.width * scale) / 2;
    const oy = (height - world.height * scale) / 2;
    return {
      width,
      height,
      scale,
      world,
      x: (value) => ox + value * scale,
      y: (value) => height - oy - value * scale,
    };
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
    const geom = makeGeometry(world, width, height);
    drawBackground(geom);

    if (state) {
      drawMap(geom, state.map);
      drawZones(geom, state.zones || []);
      drawGroundObstacles(
        geom,
        (state.world && state.world.obstacle_shapes) || []
      );
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
    ctx.fillStyle = "#070b16";
    ctx.fillRect(0, 0, width, height);

    const xMin = -geom.ox / geom.scale;
    const xMax = (width - geom.ox) / geom.scale;
    const yMin = -geom.oy / geom.scale;
    const yMax = (height - geom.oy) / geom.scale;
    ctx.strokeStyle = "rgba(70, 110, 175, 0.09)";
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
    gradient.addColorStop(0, "#101f38");
    gradient.addColorStop(1, "#0a1324");
    ctx.fillStyle = gradient;
    ctx.fillRect(x0, y0, x1 - x0, y1 - y0);

    ctx.strokeStyle = "rgba(84, 128, 196, 0.14)";
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

    ctx.strokeStyle = "rgba(110, 190, 255, 0.35)";
    ctx.lineWidth = 2;
    ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
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
      const x = geom.x(zone.x0);
      const y = geom.y(zone.y1);
      const width = (zone.x1 - zone.x0) * geom.scale;
      const height = (zone.y1 - zone.y0) * geom.scale;
      if (zone.current) {
        ctx.fillStyle = "rgba(55, 224, 160, 0.05)";
        ctx.fillRect(x, y, width, height);
      }
      ctx.setLineDash([4, 5]);
      ctx.lineWidth = zone.current ? 1.8 : 1;
      ctx.strokeStyle = zone.current
        ? "rgba(55, 224, 160, 0.85)"
        : zone.selected
        ? "rgba(79, 195, 255, 0.35)"
        : "rgba(139, 163, 199, 0.16)";
      ctx.strokeRect(x, y, width, height);
      ctx.setLineDash([]);
      const labelX = x + width / 2;
      const labelY = y + height / 2;
      ctx.font = "600 " + Math.max(11, geom.scale * 0.18).toFixed(0) + "px 'Segoe UI', sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillStyle = zone.current
        ? "rgba(190, 255, 230, 0.9)"
        : zone.selected
        ? "rgba(160, 215, 255, 0.6)"
        : "rgba(139, 163, 199, 0.35)";
      ctx.fillText(String(zone.id), labelX, labelY - geom.scale * 0.12);
      ctx.font = Math.max(8, geom.scale * 0.10).toFixed(0) + "px 'Segoe UI', sans-serif";
      ctx.fillStyle = zone.selected ? "rgba(55, 224, 160, 0.8)" : "rgba(139, 163, 199, 0.35)";
      ctx.fillText(Math.round(zone.coverage * 100) + "%", labelX, labelY + geom.scale * 0.14);
    });
    ctx.restore();
  }

  function drawGroundObstacles(geom, shapes) {
    ctx.save();
    shapes.forEach((shape) => {
      const outline = shape.outline || [];
      if (outline.length < 3) return;
      const large = shape.kind === "large";
      const color = large ? "255, 143, 90" : "255, 209, 102";
      ctx.beginPath();
      outline.forEach((point, index) => {
        const x = geom.x(point[0]);
        const y = geom.y(point[1]);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.closePath();
      ctx.fillStyle = "rgba(" + color + ", 0.13)";
      ctx.fill();
      ctx.lineWidth = 1.4;
      ctx.strokeStyle = "rgba(" + color + ", 0.75)";
      ctx.stroke();
    });
    ctx.restore();
  }

  function drawObstacles(geom, obstacles) {
    ctx.save();
    obstacles.forEach((obstacle) => {
      const large = obstacle.kind === "large";
      const color = large ? "255, 143, 90" : "255, 209, 102";
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
      ctx.strokeStyle = "rgba(79, 195, 255, 0.55)";
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
      ctx.strokeStyle = "rgba(255, 209, 102, 0.75)";
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
      ctx.strokeStyle = "rgba(79, 195, 255, 0.9)";
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
      ctx.strokeStyle = "rgba(255, 209, 102, 0.9)";
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
      ctx.strokeStyle = "rgba(79, 195, 255, " + alpha.toFixed(3) + ")";
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
      glow.addColorStop(0, "rgba(55, 224, 160, " + (0.22 + pulse * 0.18).toFixed(3) + ")");
      glow.addColorStop(1, "rgba(55, 224, 160, 0)");
      ctx.fillStyle = glow;
      ctx.beginPath();
      ctx.arc(cx, cy, r * 2.4, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(55, 224, 160, 0.10)";
    ctx.fill();
    ctx.setLineDash([5, 5]);
    ctx.lineDashOffset = -performance.now() / 90;
    ctx.strokeStyle = "rgba(55, 224, 160, " + (charging ? 0.95 : 0.7) + ")";
    ctx.lineWidth = 1.6;
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = charging ? "#d8fff0" : "rgba(55, 224, 160, 0.92)";
    ctx.font = "bold " + Math.max(13, r * 0.8).toFixed(0) + "px 'Segoe UI', sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("⚡", cx, cy);
    ctx.font = Math.max(9, r * 0.3).toFixed(0) + "px 'PingFang SC', sans-serif";
    ctx.fillStyle = "rgba(159, 255, 214, 0.8)";
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
    ctx.fillStyle = "rgba(79, 195, 255, 0.09)";
    ctx.fill();
    ctx.strokeStyle = "rgba(79, 195, 255, 0.30)";
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.strokeStyle = "rgba(79, 195, 255, 0.16)";
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
    glow.addColorStop(0, "rgba(79, 195, 255, 0.30)");
    glow.addColorStop(1, "rgba(79, 195, 255, 0)");
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
      ctx.strokeStyle = driven ? "rgba(79, 195, 255, 0.85)" : "rgba(130, 160, 200, 0.4)";
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
    ctx.strokeStyle = flashing ? "#ff5d6c" : "rgba(190, 245, 255, 0.85)";
    ctx.lineWidth = flashing ? 3 : 1.6;
    ctx.stroke();

    // 割草盘
    ctx.beginPath();
    ctx.arc(0, 0, radius * 0.62, 0, Math.PI * 2);
    ctx.strokeStyle = "rgba(55, 224, 160, 0.55)";
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
      ctx.strokeStyle = "rgba(255, 93, 108, 0.55)";
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
    ctx.strokeStyle = "rgba(139, 163, 199, 0.6)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + length, y);
    ctx.moveTo(x, y - 4);
    ctx.lineTo(x, y + 4);
    ctx.moveTo(x + length, y - 4);
    ctx.lineTo(x + length, y + 4);
    ctx.stroke();
    ctx.fillStyle = "rgba(139, 163, 199, 0.85)";
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

  els.mapSelect.addEventListener("change", () => {
    trail = [];
    mapVersion = -1;
    zoneAutoValue = "";
    zoneInputDirty = false;
    send("/api/map", { map: Number(els.mapSelect.value) });
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
  const moduleDefaults = { telemetry: true, manual: true, zones: true, events: true, params: true };
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
      panel.style.transform = offset
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
  document.querySelectorAll("[data-module-toggle]").forEach((checkbox) => {
    checkbox.addEventListener("change", () => {
      moduleState[checkbox.dataset.moduleToggle] = checkbox.checked;
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
  applyCollapsed();

  // ------------------------------------------------------------- 启动
  function frame(now) {
    draw(now);
    requestAnimationFrame(frame);
  }

  requestAnimationFrame(frame);
  poll();
})();
