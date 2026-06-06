const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");
const notice = document.getElementById("notice");

let latestMap = null;
let latestSemantic = null;
let latestGoals = {};
let latestStatus = null;
let latestMapping = null;
let noticeTimer = null;
let deviceStrip = null;
let passabilityWidget = null;
let bevWidget = null;
let cameraGrid = null;
let mapMode = "goal";
let selectedPoint = null;
let draftPoints = [];

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: {"Content-Type": "application/json"},
    ...options,
  });
  if (!response.ok) {
    let message = `${response.status} ${response.statusText}`;
    const text = await response.text();
    if (text) {
      try {
        const payload = JSON.parse(text);
        message = payload.detail || payload.reason || message;
      } catch (_) {
        message = text;
      }
    }
    throw new Error(message);
  }
  return response.json();
}

function showNotice(message, error = false) {
  notice.textContent = message;
  notice.classList.toggle("error", error);
  notice.hidden = false;
  clearTimeout(noticeTimer);
  noticeTimer = setTimeout(() => {
    notice.hidden = true;
  }, 4200);
}

function setText(id, value) {
  document.getElementById(id).textContent = value;
}

function setPill(id, value, state) {
  const node = document.getElementById(id);
  node.textContent = value;
  node.className = `pill ${state || "neutral"}`;
}

function stateFromText(text) {
  const value = String(text || "").toUpperCase();
  if (value.includes("EMERGENCY") || value.includes("STOP") || value.includes("ERROR") || value.includes("FAIL")) return "bad";
  if (value.includes("WARN") || value.includes("SLOWDOWN") || value.includes("DEGRADED") || value.includes("UNKNOWN")) return "warn";
  if (value.includes("SAFE") || value.includes("CLEAR") || value.includes("GOOD") || value.includes("OK") || value.includes("IDLE") || value.includes("ACTIVE")) return "ok";
  return "neutral";
}

function mappingStateClass(state) {
  const value = String(state || "").toUpperCase();
  if (value === "MAP_READY") return "ok";
  if (value === "MAPPING" || value === "SAVING") return "ok";
  if (value === "PREFLIGHT_FAILED") return "warn";
  if (value === "ERROR") return "bad";
  return "neutral";
}

function formatDistanceMm(item) {
  const mm = Number(item.range_mm);
  if (Number.isFinite(mm)) return `${Math.round(mm)} mm`;
  const meters = Number(item.range_m);
  if (Number.isFinite(meters)) return `${Math.round(meters * 1000)} mm`;
  return "--";
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(320, Math.round(rect.width));
  const height = Math.max(320, Math.round(rect.height));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
    if (latestMap) drawMap(latestMap);
  }
}

function mapLayout() {
  if (!latestMap) return null;
  const scale = Math.min(canvas.width / latestMap.width, canvas.height / latestMap.height);
  return {
    scale,
    ox: (canvas.width - latestMap.width * scale) / 2,
    oy: (canvas.height - latestMap.height * scale) / 2,
  };
}

function worldToCanvas(x, y) {
  const layout = mapLayout();
  if (!layout || !latestMap) return null;
  const mx = (x - latestMap.origin.x) / latestMap.resolution;
  const my = latestMap.height - (y - latestMap.origin.y) / latestMap.resolution;
  return {
    x: layout.ox + mx * layout.scale,
    y: layout.oy + my * layout.scale,
  };
}

function drawMap(map) {
  resizeCanvas();
  latestMap = map;
  document.getElementById("map-empty-state").hidden = true;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#eef1f4";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  const layout = mapLayout();
  if (!layout) return;

  const img = ctx.createImageData(map.width, map.height);
  for (let sourceIndex = 0; sourceIndex < map.width * map.height; sourceIndex += 1) {
    const sourceY = Math.floor(sourceIndex / map.width);
    const sourceX = sourceIndex % map.width;
    const targetIndex = (map.height - 1 - sourceY) * map.width + sourceX;
    const value = map.data[sourceIndex] ?? -1;
    const color = value < 0
      ? [225, 231, 238, 210]
      : value > 50
        ? [29, 78, 121, 255]
        : [250, 251, 252, 255];
    img.data[targetIndex * 4] = color[0];
    img.data[targetIndex * 4 + 1] = color[1];
    img.data[targetIndex * 4 + 2] = color[2];
    img.data[targetIndex * 4 + 3] = color[3];
  }

  const tmp = document.createElement("canvas");
  tmp.width = map.width;
  tmp.height = map.height;
  tmp.getContext("2d").putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(tmp, layout.ox, layout.oy, map.width * layout.scale, map.height * layout.scale);

  ctx.strokeStyle = "#1f2937";
  ctx.lineWidth = 1;
  ctx.strokeRect(layout.ox, layout.oy, map.width * layout.scale, map.height * layout.scale);
  drawGrid(layout);
  drawVectorLayers();
  updateMapMeta(map);
}

function drawGrid(layout) {
  const step = Math.max(24, Math.round(0.5 / latestMap.resolution * layout.scale));
  ctx.save();
  ctx.strokeStyle = "rgba(36, 52, 71, 0.12)";
  ctx.lineWidth = 1;
  for (let x = layout.ox; x <= layout.ox + latestMap.width * layout.scale; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, layout.oy);
    ctx.lineTo(x, layout.oy + latestMap.height * layout.scale);
    ctx.stroke();
  }
  for (let y = layout.oy; y <= layout.oy + latestMap.height * layout.scale; y += step) {
    ctx.beginPath();
    ctx.moveTo(layout.ox, y);
    ctx.lineTo(layout.ox + latestMap.width * layout.scale, y);
    ctx.stroke();
  }
  ctx.restore();
}

function updateMapMeta(map) {
  const metersX = map.width * map.resolution;
  const metersY = map.height * map.resolution;
  const source = map.source_topic || "map";
  setText("map-metadata", `${source} | ${metersX.toFixed(1)}m x ${metersY.toFixed(1)}m | ${map.resolution.toFixed(3)}m/格`);
  setText("map-source", `${source} | ${formatAge(map.age_sec)}`);
  setPill("map-state", source === "/rtabmap/grid_map" ? "RTAB-MAP" : "MAP", "ok");
}

function showMapUnavailable(reason = "map not ready") {
  latestMap = null;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#eef2f6";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const empty = document.getElementById("map-empty-state");
  empty.hidden = false;
  empty.querySelector("span").textContent = reason === "map not ready"
    ? "等待 /rtabmap/grid_map 或 /map"
    : reason;
  setText("map-metadata", "地图未就绪");
  setText("map-source", "未收到 OccupancyGrid");
  setPill("map-state", "NO MAP", "warn");
}

function drawPolygon(points, fill, stroke, label) {
  const pixels = points.map(([x, y]) => worldToCanvas(x, y)).filter(Boolean);
  if (pixels.length < 3) return;
  ctx.beginPath();
  ctx.moveTo(pixels[0].x, pixels[0].y);
  pixels.slice(1).forEach((point) => ctx.lineTo(point.x, point.y));
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 2;
  ctx.fill();
  ctx.stroke();
  if (label) {
    const cx = pixels.reduce((sum, p) => sum + p.x, 0) / pixels.length;
    const cy = pixels.reduce((sum, p) => sum + p.y, 0) / pixels.length;
    ctx.fillStyle = "#101828";
    ctx.font = "13px system-ui";
    ctx.fillText(label, cx + 5, cy - 5);
  }
}

function drawVectorLayers() {
  if (!latestMap) return;
  if (latestSemantic) {
    (latestSemantic.walls || []).forEach((wall) => {
      drawPolyline(wall.polyline || [], "#101828", 2);
    });
    (latestSemantic.rooms || []).forEach((room) => {
      drawPolygon(room.polygon || [], "rgba(47, 128, 237, 0.14)", room.color || "#2f80ed", room.name);
    });
    (latestSemantic.no_go_zones || []).forEach((zone) => {
      drawPolygon(zone.polygon || [], "rgba(192, 57, 43, 0.18)", "#c0392b", zone.name);
    });
    (latestSemantic.points_of_interest || []).forEach((poi) => {
      drawPoint(poi.position?.[0], poi.position?.[1], "#6f42c1", poi.name);
    });
  }
  Object.entries(latestGoals || {}).forEach(([key, goal]) => {
    drawPoint(goal.position?.[0], goal.position?.[1], "#168a56", goal.label || key);
  });
  if (latestStatus?.route_path) {
    drawRoute(latestStatus.route_path);
  }
  if (latestStatus?.current_goal) {
    drawPoint(
      latestStatus.current_goal.x,
      latestStatus.current_goal.y,
      "#168a56",
      "当前目标",
      7,
    );
  }
  drawDraft();
  if (latestStatus?.pose) {
    drawUltrasonicRays(latestStatus.pose, latestStatus.sensors?.ultrasonic || []);
    drawPose(latestStatus.pose);
  }
}

function drawRoute(route) {
  const points = route.points || [];
  if (points.length < 2) return;
  drawPolyline(points, "#168a56", 4);
}

function drawPolyline(points, color, width) {
  const pixels = points.map(([x, y]) => worldToCanvas(x, y)).filter(Boolean);
  if (pixels.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(pixels[0].x, pixels[0].y);
  pixels.slice(1).forEach((point) => ctx.lineTo(point.x, point.y));
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.stroke();
}

function drawPoint(x, y, color, label, radius = 5) {
  if (x === undefined || y === undefined) return;
  const pixel = worldToCanvas(Number(x), Number(y));
  if (!pixel) return;
  ctx.beginPath();
  ctx.arc(pixel.x, pixel.y, radius, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
  ctx.fillStyle = "#101828";
  ctx.font = "13px system-ui";
  ctx.fillText(label, pixel.x + 8, pixel.y - 8);
}

function drawDraft() {
  if (draftPoints.length > 1) {
    drawPolyline(draftPoints, mapMode === "zone" ? "#c0392b" : "#2f80ed", 3);
  }
  draftPoints.forEach(([x, y], index) => {
    drawPoint(x, y, mapMode === "zone" ? "#c0392b" : "#2f80ed", String(index + 1), 4);
  });
  if (selectedPoint) {
    drawPoint(selectedPoint.x, selectedPoint.y, "#168a56", "选点", 6);
  }
}

function drawPose(pose) {
  const pixel = worldToCanvas(pose.x, pose.y);
  if (!pixel) return;
  const yaw = pose.yaw || 0;
  ctx.save();
  ctx.translate(pixel.x, pixel.y);
  ctx.rotate(-yaw);
  ctx.beginPath();
  ctx.moveTo(14, 0);
  ctx.lineTo(-9, -8);
  ctx.lineTo(-9, 8);
  ctx.closePath();
  ctx.fillStyle = "#f2994a";
  ctx.fill();
  ctx.strokeStyle = "#101828";
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.restore();
}

function drawUltrasonicRays(pose, sensors) {
  const origin = worldToCanvas(pose.x, pose.y);
  if (!origin) return;
  const angles = [-70, -35, 0, 35, 70, 110];
  sensors.forEach((sensor, index) => {
    if (!sensor.online || !Number.isFinite(sensor.range_m)) return;
    const angle = (pose.yaw || 0) + (angles[index] || 0) * Math.PI / 180;
    const end = worldToCanvas(
      pose.x + Math.cos(angle) * sensor.range_m,
      pose.y + Math.sin(angle) * sensor.range_m,
    );
    if (!end) return;
    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(end.x, end.y);
    ctx.strokeStyle = sensor.range_m < 0.5 ? "rgba(192, 57, 43, 0.70)" : "rgba(22, 138, 86, 0.42)";
    ctx.lineWidth = 2;
    ctx.stroke();
  });
}

function canvasToMap(event) {
  if (!latestMap) return null;
  const rect = canvas.getBoundingClientRect();
  const px = event.clientX - rect.left;
  const py = event.clientY - rect.top;
  const layout = mapLayout();
  const mx = (px - layout.ox) / layout.scale;
  const my = (py - layout.oy) / layout.scale;
  if (mx < 0 || my < 0 || mx >= latestMap.width || my >= latestMap.height) return null;
  return {
    x: Math.min(
      latestMap.origin.x + latestMap.width * latestMap.resolution - latestMap.resolution * 0.001,
      latestMap.origin.x + mx * latestMap.resolution,
    ),
    y: Math.min(
      latestMap.origin.y + latestMap.height * latestMap.resolution - latestMap.resolution * 0.001,
      latestMap.origin.y + (latestMap.height - my) * latestMap.resolution,
    ),
  };
}

function formatAge(age) {
  if (age === null || age === undefined) return "未收到";
  if (age < 1) return `${Math.round(age * 1000)}ms`;
  return `${age.toFixed(1)}s`;
}

function formatNumber(value, digits = 2) {
  return Number.isFinite(value) ? value.toFixed(digits) : "--";
}

function sensorState(entry) {
  if (entry?.online) return "online";
  if (entry?.messages > 0) return "stale";
  return "offline";
}

function renderSensorCard(title, entry, lines) {
  const state = sensorState(entry);
  const card = document.createElement("div");
  card.className = `sensor-card ${state}`;
  const heading = document.createElement("strong");
  heading.append(document.createTextNode(title));
  const dot = document.createElement("span");
  dot.className = "dot";
  heading.append(dot);
  card.append(heading);
  [...lines, `更新 ${formatAge(entry?.age_sec)}`].forEach((line) => {
    const small = document.createElement("small");
    small.textContent = line;
    card.append(small);
  });
  return card;
}

function renderSensors(status) {
  const sensors = status.sensors || {};
  const mapping = status.mapping_3d || {};
  const grid = document.getElementById("sensor-grid");
  grid.innerHTML = "";
  const scan = sensors.scan || {};
  const imu = sensors.imu || {};
  const front = sensors.camera_front || {};
  const left = sensors.camera_left || {};
  const right = sensors.camera_right || {};
  const rear = sensors.camera_rear || {};
  const base = sensors.base || {};
  const odom = sensors.odom || {};

  const cards = [
    renderSensorCard("左 XT-M60", sensors.xtm60_left || {}, [
      `/xtm60/left/points`,
      `点数 ${sensors.xtm60_left?.points ?? "--"}`,
    ]),
    renderSensorCard("右 XT-M60", sensors.xtm60_right || {}, [
      `/xtm60/right/points`,
      `点数 ${sensors.xtm60_right?.points ?? "--"}`,
    ]),
    renderSensorCard("融合点云", mapping.points_merged || {}, [
      `/points_merged`,
      `点数 ${mapping.points_merged?.points ?? "--"}`,
    ]),
    renderSensorCard("导航扫描", scan, [
      `/scan | ${scan.samples ?? "--"} 样本`,
      `最近障碍 ${formatNumber(scan.nearest_m)} m`,
    ]),
    renderSensorCard("H30 IMU", imu, [
      `Yaw ${formatNumber(imu.yaw)} rad`,
      `加速度 ${formatNumber(imu.accel_norm)} m/s²`,
    ]),
    renderSensorCard("RGB 点云", mapping.rgb_cloud_map || {}, [
      `/rgb_cloud_map`,
      `点数 ${mapping.rgb_cloud_map?.points ?? "--"}`,
    ]),
    renderSensorCard("3D 主地图", mapping.rtabmap_cloud_map || {}, [
      `/rtabmap/cloud_map`,
      `点数 ${mapping.rtabmap_cloud_map?.points ?? "--"}`,
    ]),
    renderSensorCard("2D 投影", mapping.rtabmap_grid_map || {}, [
      `/rtabmap/grid_map`,
      `${mapping.rtabmap_grid_map?.width ?? "--"} x ${mapping.rtabmap_grid_map?.height ?? "--"}`,
    ]),
    renderSensorCard("底盘", base, [
      `/base/status`,
      `写入 ${base.last_command_write_ok === undefined ? "--" : base.last_command_write_ok}`,
      `左/右 ${formatNumber(base.left_rpm)} / ${formatNumber(base.right_rpm)} rpm`,
    ]),
    renderSensorCard("轮速里程计", odom, [
      `/wheel/odom`,
      `线速 ${formatNumber(odom.linear_x)} m/s`,
      `角速 ${formatNumber(odom.angular_z)} rad/s`,
    ]),
  ];

  const cameraCards = [
    ["camera_front", "前摄像头", front],
    ["camera_left", "左摄像头", left],
    ["camera_right", "右摄像头", right],
    ["camera_rear", "后摄像头", rear],
  ];
  const configuredCameraCards = cameraCards
    .filter(([key]) => key in sensors)
    .map(([, label, camera]) => renderSensorCard(label, camera, [
      `${camera.width || "--"} x ${camera.height || "--"}`,
      camera.encoding || "--",
    ]));
  cards.splice(5, 0, ...configuredCameraCards);
  grid.append(...cards);

  const monitoredEntries = [
    sensors.xtm60_left,
    sensors.xtm60_right,
    mapping.points_merged,
    scan,
    imu,
    mapping.rgb_cloud_map,
    mapping.rtabmap_cloud_map,
    mapping.rtabmap_grid_map,
    base,
    odom,
    ...cameraCards.filter(([key]) => key in sensors).map(([, , camera]) => camera.online),
  ];
  const onlineCount = monitoredEntries.filter((entry) => entry?.online || entry === true).length;
  const monitoredCount = monitoredEntries.length;
  const ultrasonicCount = (sensors.ultrasonic || []).filter((item) => item.online).length;
  const ultrasonicTotal = (sensors.ultrasonic || []).length;
  setText("sensor-summary", `${onlineCount}/${monitoredCount} | 超声波 ${ultrasonicCount}/${ultrasonicTotal}`);
  renderUltrasonic(sensors.ultrasonic || []);
  deviceStrip?.update({
    ...sensors,
    points_merged: mapping.points_merged,
    rgb_cloud_map: mapping.rgb_cloud_map,
  });
  cameraGrid?.update(sensors);
}

function renderUltrasonic(items) {
  const grid = document.getElementById("ultrasonic-grid");
  grid.innerHTML = "";
  items.forEach((item) => {
    const index = item.index;
    const positions = ["左前", "左侧", "右前", "右侧"];
    const range = Number(item.range_m);
    const distance = formatDistanceMm(item);
    const maxRange = Number(item.max_range) || 3.0;
    const tile = document.createElement("div");
    const low = Number.isFinite(range) && range < 0.5;
    const warn = Number.isFinite(range) && range < 1.5;
    tile.className = `range-tile ${low ? "bad" : warn ? "warn" : ""}`;
    const label = document.createElement("span");
    label.innerHTML = `<b>U${index} ${positions[index] || ""}</b><em>${distance}</em>`;
    const bar = document.createElement("div");
    bar.className = "range-bar";
    const fill = document.createElement("i");
    fill.style.width = `${Math.max(0, Math.min(100, (range / maxRange) * 100 || 0))}%`;
    bar.append(fill);
    tile.append(label, bar);
    grid.append(tile);
  });
}

async function refreshStatus() {
  try {
    const status = await api("/api/status");
    latestStatus = status;
    setPill("safety-state", status.safety_state, stateFromText(status.safety_state));
    setPill("nav-status", status.navigation_status, stateFromText(status.navigation_status));
    const pose = status.pose || {};
    const localizationText = `${pose.source || "unknown"} / ${pose.confidence || "low"}`;
    setPill(
      "localization-state",
      pose.confidence === "high" ? "定位可信" : "定位低可信",
      pose.confidence === "high" ? "ok" : "warn",
    );
    setText("pose", `${Number(pose.x || 0).toFixed(2)}, ${Number(pose.y || 0).toFixed(2)}, ${Number(pose.yaw || 0).toFixed(2)}`);
    setText("pose-source", `${localizationText} | ${formatAge(pose.age_sec)}`);
    setText("current-goal", status.current_goal ? `${status.current_goal.x.toFixed(2)}, ${status.current_goal.y.toFixed(2)}` : "无");
    setText("hardware-status", `${status.hardware_status.state || "UNKNOWN"} - ${status.hardware_status.reason || ""}`);
    setText("localization-health", `${status.localization_health.state || "UNKNOWN"} - ${status.localization_health.reason || ""}`);
    setText("passability-status", `${status.passability_status.state || "UNKNOWN"} - ${status.passability_status.reason || ""}`);
    setText("last-refresh", new Date().toLocaleTimeString());
    if (status.map_status?.ok) {
      setText("map-source", `${status.map_status.source_topic} | ${formatAge(status.map_status.age_sec)}`);
    }
    renderSensors(status);
    passabilityWidget?.update(status);
    bevWidget?.update(status);
    if (latestMap) drawMap(latestMap);
  } catch (error) {
    showNotice(`状态刷新失败: ${error.message}`, true);
  }
}

async function refreshGoals() {
  try {
    latestGoals = await api("/api/goals");
    const list = document.getElementById("goal-list");
    list.innerHTML = "";
    Object.entries(latestGoals).forEach(([key, goal]) => {
      const item = document.createElement("li");
      const label = document.createElement("span");
      label.textContent = `${goal.label || key} (${goal.position[0].toFixed(2)}, ${goal.position[1].toFixed(2)})`;
      const go = document.createElement("button");
      go.textContent = "导航";
      go.onclick = async () => {
        try {
          await api(`/api/navigate/${encodeURIComponent(key)}`, {method: "POST"});
          refreshStatus();
        } catch (error) {
          showNotice(`目标不可导航: ${error.message}`, true);
        }
      };
      const del = document.createElement("button");
      del.textContent = "删除";
      del.className = "secondary";
      del.onclick = async () => {
        await api(`/api/goals/${encodeURIComponent(key)}`, {method: "DELETE"});
        refreshGoals();
      };
      item.append(label, go, del);
      list.appendChild(item);
    });
    setText("goal-count", String(Object.keys(latestGoals).length));
    if (latestMap) drawMap(latestMap);
  } catch (error) {
    showNotice(`目标点刷新失败: ${error.message}`, true);
  }
}

function parsePolygon(text) {
  return text
    .split(";")
    .map((pair) => pair.trim())
    .filter(Boolean)
    .map((pair) => pair.split(",").map((value) => Number(value.trim())))
    .filter((pair) => pair.length === 2 && pair.every(Number.isFinite));
}

async function refreshSemanticMap() {
  try {
    latestSemantic = await api("/api/semantic-map");
    const roomList = document.getElementById("room-list");
    const zoneList = document.getElementById("zone-list");
    roomList.innerHTML = "";
    zoneList.innerHTML = "";
    (latestSemantic.rooms || []).forEach((room) => {
      const item = document.createElement("li");
      const label = document.createElement("span");
      label.textContent = `${room.name} (${(room.polygon || []).length} 点)`;
      const del = document.createElement("button");
      del.textContent = "删除";
      del.className = "secondary";
      del.onclick = async () => {
        await api(`/api/rooms/${encodeURIComponent(room.name)}`, {method: "DELETE"});
        refreshSemanticMap();
      };
      item.append(label, del);
      roomList.appendChild(item);
    });
    (latestSemantic.no_go_zones || []).forEach((zone) => {
      const item = document.createElement("li");
      const label = document.createElement("span");
      label.textContent = `${zone.name} (${(zone.polygon || []).length} 点)`;
      const del = document.createElement("button");
      del.textContent = "删除";
      del.className = "secondary";
      del.onclick = async () => {
        await api(`/api/no-go-zones/${encodeURIComponent(zone.name)}`, {method: "DELETE"});
        refreshSemanticMap();
      };
      item.append(label, del);
      zoneList.appendChild(item);
    });
    setText("semantic-summary", `房间 ${(latestSemantic.rooms || []).length} | 禁行 ${(latestSemantic.no_go_zones || []).length}`);
    if (latestMap) drawMap(latestMap);
  } catch (error) {
    showNotice(`语义地图刷新失败: ${error.message}`, true);
  }
}

async function refreshMap() {
  try {
    const map = await api("/api/map");
    if (!map.ok) {
      showMapUnavailable(map.reason);
      return;
    }
    drawMap(map);
  } catch (error) {
    showMapUnavailable("地图接口不可用");
    showNotice(`地图刷新失败: ${error.message}`, true);
  }
}

async function refreshMapping() {
  try {
    latestMapping = await api("/api/mapping/status");
    renderMapping(latestMapping);
  } catch (error) {
    showNotice(`建图状态刷新失败: ${error.message}`, true);
  }
}

function renderMapping(mapping) {
  const state = mapping.state || "IDLE";
  const chip = document.getElementById("mapping-state-chip");
  chip.textContent = state;
  chip.className = `mini-chip ${mappingStateClass(state)}`;
  document.getElementById("mapping-reason").textContent = mapping.reason || "等待建图";
  document.getElementById("mapping-progress-fill").style.width = `${Math.max(0, Math.min(100, mapping.progress || 0))}%`;

  const startBtn = document.getElementById("mapping-start-btn");
  const finishBtn = document.getElementById("mapping-finish-btn");
  const cancelBtn = document.getElementById("mapping-cancel-btn");
  const busy = state === "MAPPING" || state === "SAVING";
  startBtn.disabled = busy;
  finishBtn.disabled = state !== "MAPPING";
  cancelBtn.disabled = !(state === "MAPPING" || state === "SAVING" || state === "PREFLIGHT_FAILED" || state === "ERROR");

  const preflight = document.getElementById("mapping-preflight");
  preflight.innerHTML = "";
  (mapping.preflight?.checks || []).forEach((item) => {
    const node = document.createElement("div");
    node.className = `preflight-item ${item.ok ? "ok" : ""} ${item.required ? "required" : "optional"}`;
    node.title = item.detail || "";
    node.innerHTML = `<i></i><span>${item.label}</span>`;
    preflight.append(node);
  });
}

function updateSelectionUi() {
  const value = document.getElementById("selected-point");
  value.textContent = selectedPoint
    ? `${selectedPoint.x.toFixed(2)}, ${selectedPoint.y.toFixed(2)}`
    : draftPoints.length
      ? `${draftPoints.length} 个顶点`
      : "未选点";
  document.getElementById("send-click-goal-btn").disabled = !selectedPoint || mapMode !== "goal";
  document.getElementById("set-initial-pose-btn").disabled = !selectedPoint || mapMode !== "initial";
}

function setMapMode(mode) {
  mapMode = mode;
  selectedPoint = null;
  draftPoints = [];
  const labels = {
    goal: "临时目标",
    poi: "保存 POI",
    initial: "初始位姿",
    room: "房间",
    zone: "禁行区",
  };
  document.querySelectorAll("[data-map-mode]").forEach((button) => {
    const active = button.dataset.mapMode === mode;
    button.classList.toggle("active", active);
    button.classList.toggle("secondary", !active);
  });
  setPill("mode-state", labels[mode], "neutral");
  updateSelectionUi();
  if (latestMap) drawMap(latestMap);
}

function updatePolygonInput() {
  const targetId = mapMode === "room" ? "room-polygon" : "zone-polygon";
  document.getElementById(targetId).value = draftPoints
    .map(([x, y]) => `${x.toFixed(2)},${y.toFixed(2)}`)
    .join("; ");
}

canvas.addEventListener("click", (event) => {
  const point = canvasToMap(event);
  if (!point) return;
  if (mapMode === "room" || mapMode === "zone") {
    draftPoints.push([point.x, point.y]);
    updatePolygonInput();
  } else {
    selectedPoint = point;
    if (mapMode === "goal" || mapMode === "poi") {
      document.getElementById("goal-x").value = point.x.toFixed(2);
      document.getElementById("goal-y").value = point.y.toFixed(2);
    }
  }
  updateSelectionUi();
  drawMap(latestMap);
});

window.addEventListener("resize", () => {
  resizeCanvas();
  if (latestMap) drawMap(latestMap);
});

document.getElementById("goal-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const payload = {
    name: document.getElementById("goal-name").value,
    x: Number(document.getElementById("goal-x").value),
    y: Number(document.getElementById("goal-y").value),
    yaw: Number(document.getElementById("goal-yaw").value || 0),
    frame_id: "map",
  };
  try {
    await api("/api/goals", {method: "POST", body: JSON.stringify(payload)});
    event.target.reset();
    document.getElementById("goal-yaw").value = 0;
    selectedPoint = null;
    updateSelectionUi();
    refreshGoals();
  } catch (error) {
    showNotice(`目标点保存失败: ${error.message}`, true);
  }
});

document.getElementById("room-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const polygon = parsePolygon(document.getElementById("room-polygon").value);
  if (polygon.length < 3) {
    showNotice("房间边界至少需要 3 个点", true);
    return;
  }
  try {
    await api("/api/rooms", {
      method: "POST",
      body: JSON.stringify({
        name: document.getElementById("room-name").value,
        polygon,
        color: document.getElementById("room-color").value,
      }),
    });
    event.target.reset();
    draftPoints = [];
    updateSelectionUi();
    refreshSemanticMap();
  } catch (error) {
    showNotice(`房间保存失败: ${error.message}`, true);
  }
});

document.getElementById("zone-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const polygon = parsePolygon(document.getElementById("zone-polygon").value);
  if (polygon.length < 3) {
    showNotice("禁行区至少需要 3 个点", true);
    return;
  }
  try {
    await api("/api/no-go-zones", {
      method: "POST",
      body: JSON.stringify({
        name: document.getElementById("zone-name").value,
        polygon,
      }),
    });
    event.target.reset();
    draftPoints = [];
    updateSelectionUi();
    refreshSemanticMap();
  } catch (error) {
    showNotice(`禁行区保存失败: ${error.message}`, true);
  }
});

document.querySelectorAll("[data-map-mode]").forEach((button) => {
  button.addEventListener("click", () => setMapMode(button.dataset.mapMode));
});

document.getElementById("send-click-goal-btn").onclick = async () => {
  if (!selectedPoint) return;
  try {
    await api("/api/navigate", {
      method: "POST",
      body: JSON.stringify({
        x: selectedPoint.x,
        y: selectedPoint.y,
        yaw: Number(document.getElementById("goal-yaw").value || 0),
      }),
    });
    showNotice("临时目标已发送到 Nav2");
    refreshStatus();
  } catch (error) {
    showNotice(`目标不可导航: ${error.message}`, true);
  }
};

document.getElementById("set-initial-pose-btn").onclick = async () => {
  if (!selectedPoint) return;
  try {
    await api("/api/initial_pose", {
      method: "POST",
      body: JSON.stringify({
        x: selectedPoint.x,
        y: selectedPoint.y,
        yaw: Number(document.getElementById("goal-yaw").value || 0),
      }),
    });
    showNotice("初始位姿已发布，等待定位确认");
    refreshStatus();
  } catch (error) {
    showNotice(`初始位姿设置失败: ${error.message}`, true);
  }
};

document.getElementById("clear-draft-btn").onclick = () => {
  selectedPoint = null;
  draftPoints = [];
  if (mapMode === "room") document.getElementById("room-polygon").value = "";
  if (mapMode === "zone") document.getElementById("zone-polygon").value = "";
  updateSelectionUi();
  if (latestMap) drawMap(latestMap);
};

document.getElementById("stop-btn").onclick = async () => {
  await api("/api/stop", {method: "POST"});
  refreshStatus();
};
document.getElementById("resume-btn").onclick = async () => {
  await api("/api/release_stop", {method: "POST"});
  refreshStatus();
};
document.getElementById("refresh-map-btn").onclick = refreshMap;
document.getElementById("mapping-start-btn").onclick = async () => {
  const mapName = document.getElementById("mapping-name").value.trim();
  const result = await api("/api/mapping/start", {
    method: "POST",
    body: JSON.stringify({map_name: mapName || null}),
  });
  renderMapping(result);
  showNotice(result.reason || "建图已启动");
};
document.getElementById("mapping-finish-btn").onclick = async () => {
  const mapName = document.getElementById("mapping-name").value.trim();
  const result = await api("/api/mapping/finish", {
    method: "POST",
    body: JSON.stringify({map_name: mapName || null}),
  });
  renderMapping(result);
  await refreshMap();
  if (result.backend === "rtabmap") {
    const m3d = result.saved_map_3d ? `3D 主地图：${result.saved_map_3d}` : "3D 主地图：导出跳过（需移动建图）";
    const m2d = result.saved_map ? `；2D 导航投影：${result.saved_map}` : "";
    showNotice(`${m3d}${m2d}`);
  } else {
    showNotice(result.saved_map ? `2D 地图已保存：${result.saved_map}` : "地图已保存");
  }
};
document.getElementById("mapping-cancel-btn").onclick = async () => {
  const result = await api("/api/mapping/cancel", {method: "POST"});
  renderMapping(result);
  showNotice("建图已取消");
};

function initGuiWidgets() {
  deviceStrip = new DeviceStrip(document.getElementById("device-strip"));
  passabilityWidget = new PassabilityWidget();
  bevWidget = new BevWidget(document.getElementById("bev-canvas"));
  cameraGrid = new CameraStatusGrid(document.getElementById("camera-grid"));
}

initGuiWidgets();
resizeCanvas();
setMapMode("goal");
refreshMap();
refreshMapping();
refreshSemanticMap();
refreshGoals();
refreshStatus();
setInterval(refreshStatus, 1000);
setInterval(refreshMapping, 2000);
setInterval(refreshMap, 5000);
