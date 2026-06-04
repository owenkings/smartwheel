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

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: {"Content-Type": "application/json"},
    ...options,
  });
  if (!response.ok) throw new Error(await response.text());
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
  if (value.includes("EMERGENCY") || value.includes("ERROR") || value.includes("FAIL")) return "bad";
  if (value.includes("WARN") || value.includes("DEGRADED") || value.includes("UNKNOWN")) return "warn";
  if (value.includes("SAFE") || value.includes("OK") || value.includes("IDLE") || value.includes("ACTIVE")) return "ok";
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
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#eef1f4";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  const layout = mapLayout();
  if (!layout) return;

  const img = ctx.createImageData(map.width, map.height);
  for (let i = 0; i < map.data.length; i += 1) {
    const value = map.data[i];
    const color = value < 0 ? 209 : value > 50 ? 34 : 247;
    img.data[i * 4] = color;
    img.data[i * 4 + 1] = color;
    img.data[i * 4 + 2] = color;
    img.data[i * 4 + 3] = 255;
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
  setText("map-metadata", `${map.frame_id || "map"} | ${metersX.toFixed(1)}m x ${metersY.toFixed(1)}m | ${map.resolution.toFixed(3)}m/格`);
  setPill("map-state", "MAP", "ok");
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
  if (latestStatus?.pose) {
    drawUltrasonicRays(latestStatus.pose, latestStatus.sensors?.ultrasonic || []);
    drawPose(latestStatus.pose);
  }
}

function drawRoute(route) {
  const points = route.points || [];
  if (points.length < 2) return;
  drawPolyline(points, "#f2994a", 4);
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

function drawPoint(x, y, color, label) {
  if (x === undefined || y === undefined) return;
  const pixel = worldToCanvas(Number(x), Number(y));
  if (!pixel) return;
  ctx.beginPath();
  ctx.arc(pixel.x, pixel.y, 5, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
  ctx.fillStyle = "#101828";
  ctx.font = "13px system-ui";
  ctx.fillText(label, pixel.x + 8, pixel.y - 8);
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
  const mx = Math.floor((px - layout.ox) / layout.scale);
  const my = Math.floor((py - layout.oy) / layout.scale);
  if (mx < 0 || my < 0 || mx >= latestMap.width || my >= latestMap.height) return null;
  return {
    x: latestMap.origin.x + mx * latestMap.resolution,
    y: latestMap.origin.y + (latestMap.height - my) * latestMap.resolution,
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
  const grid = document.getElementById("sensor-grid");
  grid.innerHTML = "";
  const laser = sensors.laser || {};
  const scan = sensors.scan || {};
  const imu = sensors.imu || {};
  const front = sensors.camera_front || {};
  const left = sensors.camera_left || {};
  const right = sensors.camera_right || {};
  const rear = sensors.camera_rear || {};
  const base = sensors.base || {};
  const odom = sensors.odom || {};

  const cards = [
    renderSensorCard("XT-M60", laser.online ? laser : scan, [
      `点云 ${laser.points ?? "--"} | 扫描 ${scan.samples ?? "--"}`,
      `最近障碍 ${formatNumber(scan.nearest_m)} m`,
    ]),
    renderSensorCard("H30 IMU", imu, [
      `Yaw ${formatNumber(imu.yaw)} rad`,
      `加速度 ${formatNumber(imu.accel_norm)} m/s²`,
    ]),
    renderSensorCard("底盘", base, [
      `写入 ${base.last_command_write_ok === undefined ? "--" : base.last_command_write_ok}`,
      `左/右 ${formatNumber(base.left_rpm)} / ${formatNumber(base.right_rpm)} rpm`,
    ]),
    renderSensorCard("里程计", odom, [
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
  cards.splice(2, 0, ...configuredCameraCards);
  grid.append(...cards);

  const onlineCount = [
    laser.online || scan.online,
    imu.online,
    base.online,
    odom.online,
    ...cameraCards.filter(([key]) => key in sensors).map(([, , camera]) => camera.online),
  ].filter(Boolean).length;
  const monitoredCount = 4 + cameraCards.filter(([key]) => key in sensors).length;
  const ultrasonicCount = (sensors.ultrasonic || []).filter((item) => item.online).length;
  const ultrasonicTotal = (sensors.ultrasonic || []).length;
  setText("sensor-summary", `${onlineCount}/${monitoredCount} | 超声波 ${ultrasonicCount}/${ultrasonicTotal}`);
  renderUltrasonic(sensors.ultrasonic || []);
  deviceStrip?.update(sensors);
  cameraGrid?.update(sensors);
}

function renderUltrasonic(items) {
  const grid = document.getElementById("ultrasonic-grid");
  grid.innerHTML = "";
  items.forEach((item) => {
    const index = item.index;
    const range = Number(item.range_m);
    const distance = formatDistanceMm(item);
    const maxRange = Number(item.max_range) || 3.0;
    const tile = document.createElement("div");
    const low = Number.isFinite(range) && range < 0.5;
    const warn = Number.isFinite(range) && range < 1.5;
    tile.className = `range-tile ${low ? "bad" : warn ? "warn" : ""}`;
    const label = document.createElement("span");
    label.innerHTML = `<b>U${index}</b><em>${distance}</em>`;
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
    setText("pose", `${status.pose.x.toFixed(2)}, ${status.pose.y.toFixed(2)}, ${status.pose.yaw.toFixed(2)}`);
    setText("current-goal", status.current_goal ? `${status.current_goal.x.toFixed(2)}, ${status.current_goal.y.toFixed(2)}` : "无");
    setText("hardware-status", `${status.hardware_status.state || "UNKNOWN"} - ${status.hardware_status.reason || ""}`);
    setText("localization-health", `${status.localization_health.state || "UNKNOWN"} - ${status.localization_health.reason || ""}`);
    setText("passability-status", `${status.passability_status.state || "UNKNOWN"} - ${status.passability_status.reason || ""}`);
    setText("last-refresh", new Date().toLocaleTimeString());
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
    drawMap(await api("/api/map"));
  } catch (error) {
    setPill("map-state", "NO MAP", "warn");
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

canvas.addEventListener("click", (event) => {
  const point = canvasToMap(event);
  if (!point) return;
  document.getElementById("goal-x").value = point.x.toFixed(2);
  document.getElementById("goal-y").value = point.y.toFixed(2);
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
  await api("/api/rooms", {
    method: "POST",
    body: JSON.stringify({
      name: document.getElementById("room-name").value,
      polygon,
      color: document.getElementById("room-color").value,
    }),
  });
  event.target.reset();
  refreshSemanticMap();
});

document.getElementById("zone-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const polygon = parsePolygon(document.getElementById("zone-polygon").value);
  if (polygon.length < 3) {
    showNotice("禁行区至少需要 3 个点", true);
    return;
  }
  await api("/api/no-go-zones", {
    method: "POST",
    body: JSON.stringify({
      name: document.getElementById("zone-name").value,
      polygon,
    }),
  });
  event.target.reset();
  refreshSemanticMap();
});

document.getElementById("stop-btn").onclick = async () => {
  await api("/api/stop", {method: "POST"});
  refreshStatus();
};
document.getElementById("resume-btn").onclick = async () => {
  await api("/api/resume", {method: "POST"});
  refreshStatus();
};
document.getElementById("zero-btn").onclick = async () => {
  await api("/api/hardware/zero", {method: "POST"});
  showNotice("已发布零速命令");
};
document.getElementById("shutdown-btn").onclick = async () => {
  await api("/api/hardware/shutdown", {method: "POST"});
  showNotice("已发布硬件关闭命令");
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
refreshMap();
refreshMapping();
refreshSemanticMap();
refreshGoals();
refreshStatus();
setInterval(refreshStatus, 1000);
setInterval(refreshMapping, 2000);
setInterval(refreshMap, 5000);
