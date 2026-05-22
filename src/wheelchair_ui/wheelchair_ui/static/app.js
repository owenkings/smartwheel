const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");
let latestMap = null;
let latestSemantic = null;
let latestGoals = {};
let latestStatus = null;

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: {"Content-Type": "application/json"},
    ...options,
  });
  if (!response.ok) throw new Error(await response.text());
  return response.json();
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
  latestMap = map;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#f3f5f7";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const layout = mapLayout();
  const img = ctx.createImageData(map.width, map.height);
  for (let i = 0; i < map.data.length; i++) {
    const value = map.data[i];
    const color = value < 0 ? 205 : value > 50 ? 35 : 245;
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
  ctx.strokeRect(layout.ox, layout.oy, map.width * layout.scale, map.height * layout.scale);
  drawVectorLayers();
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
    ctx.fillStyle = stroke;
    ctx.font = "13px system-ui";
    ctx.fillText(label, cx + 4, cy - 4);
  }
}

function drawVectorLayers() {
  if (!latestMap) return;
  if (latestSemantic) {
    (latestSemantic.walls || []).forEach((wall) => {
      drawPolyline(wall.polyline || [], "#111827", 2);
    });
    (latestSemantic.rooms || []).forEach((room) => {
      drawPolygon(room.polygon || [], "rgba(64, 132, 214, 0.16)", room.color || "#1f6feb", room.name);
    });
    (latestSemantic.no_go_zones || []).forEach((zone) => {
      drawPolygon(zone.polygon || [], "rgba(198, 40, 40, 0.20)", "#c62828", zone.name);
    });
    (latestSemantic.points_of_interest || []).forEach((poi) => {
      drawPoint(poi.position?.[0], poi.position?.[1], "#6a4c93", poi.name);
    });
  }
  Object.entries(latestGoals || {}).forEach(([key, goal]) => {
    drawPoint(goal.position?.[0], goal.position?.[1], "#0f8b4c", goal.label || key);
  });
  if (latestStatus?.pose) drawPose(latestStatus.pose);
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
  ctx.fillStyle = "#111827";
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
  ctx.moveTo(12, 0);
  ctx.lineTo(-8, -7);
  ctx.lineTo(-8, 7);
  ctx.closePath();
  ctx.fillStyle = "#ff9f1c";
  ctx.fill();
  ctx.restore();
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

canvas.addEventListener("click", (event) => {
  const point = canvasToMap(event);
  if (!point) return;
  document.getElementById("goal-x").value = point.x.toFixed(2);
  document.getElementById("goal-y").value = point.y.toFixed(2);
});

async function refreshStatus() {
  const status = await api("/api/status");
  latestStatus = status;
  document.getElementById("safety-state").textContent = status.safety_state;
  document.getElementById("nav-status").textContent = status.navigation_status;
  document.getElementById("pose").textContent =
    `${status.pose.x.toFixed(2)}, ${status.pose.y.toFixed(2)}, ${status.pose.yaw.toFixed(2)}`;
  document.getElementById("current-goal").textContent =
    status.current_goal ? `${status.current_goal.x.toFixed(2)}, ${status.current_goal.y.toFixed(2)}` : "无";
  document.getElementById("sensors").textContent =
    `雷达:${status.sensor_status.laser ? "在线" : "离线"} ` +
    `IMU:${status.sensor_status.imu ? "在线" : "离线"} ` +
    `超声波:${status.sensor_status.ultrasonic.filter(Boolean).length}/6 ` +
    `前摄:${status.sensor_status.camera_front ? "在线" : "离线"} ` +
    `左摄:${status.sensor_status.camera_left ? "在线" : "离线"} ` +
    `底盘:${status.sensor_status.base ? "在线" : "离线"}`;
  document.getElementById("hardware-status").textContent =
    `${status.hardware_status.state || "UNKNOWN"} - ${status.hardware_status.reason || ""}`;
  document.getElementById("localization-health").textContent =
    `${status.localization_health.state || "UNKNOWN"} - ${status.localization_health.reason || ""}`;
  document.getElementById("passability-status").textContent =
    `${status.passability_status.state || "UNKNOWN"} - ${status.passability_status.reason || ""}`;
  if (latestMap) drawMap(latestMap);
}

async function refreshGoals() {
  latestGoals = await api("/api/goals");
  const list = document.getElementById("goal-list");
  list.innerHTML = "";
  Object.entries(latestGoals).forEach(([key, goal]) => {
    const item = document.createElement("li");
    const label = document.createElement("span");
    label.textContent = `${goal.label || key} (${goal.position[0].toFixed(2)}, ${goal.position[1].toFixed(2)})`;
    const go = document.createElement("button");
    go.textContent = "导航";
    go.onclick = () => api(`/api/navigate/${encodeURIComponent(key)}`, {method: "POST"});
    const del = document.createElement("button");
    del.textContent = "删除";
    del.onclick = async () => {
      await api(`/api/goals/${encodeURIComponent(key)}`, {method: "DELETE"});
      refreshGoals();
    };
    item.append(label, go, del);
    list.appendChild(item);
  });
  if (latestMap) drawMap(latestMap);
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
  latestSemantic = await api("/api/semantic-map");
  const roomList = document.getElementById("room-list");
  const zoneList = document.getElementById("zone-list");
  roomList.innerHTML = "";
  zoneList.innerHTML = "";
  (latestSemantic.rooms || []).forEach((room) => {
    const item = document.createElement("li");
    item.textContent = `${room.name} (${(room.polygon || []).length} 点)`;
    roomList.appendChild(item);
  });
  (latestSemantic.no_go_zones || []).forEach((zone) => {
    const item = document.createElement("li");
    item.textContent = `${zone.name} (${(zone.polygon || []).length} 点)`;
    zoneList.appendChild(item);
  });
  if (latestMap) drawMap(latestMap);
}

document.getElementById("goal-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const payload = {
    name: document.getElementById("goal-name").value,
    x: Number(document.getElementById("goal-x").value),
    y: Number(document.getElementById("goal-y").value),
    yaw: Number(document.getElementById("goal-yaw").value || 0),
    frame_id: "map",
  };
  await api("/api/goals", {method: "POST", body: JSON.stringify(payload)});
  event.target.reset();
  document.getElementById("goal-yaw").value = 0;
  refreshGoals();
});

document.getElementById("room-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const polygon = parsePolygon(document.getElementById("room-polygon").value);
  if (polygon.length < 3) return alert("房间边界至少需要 3 个点");
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
  if (polygon.length < 3) return alert("禁行区至少需要 3 个点");
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

document.getElementById("stop-btn").onclick = () => api("/api/stop", {method: "POST"});
document.getElementById("resume-btn").onclick = () => api("/api/resume", {method: "POST"});

async function refreshMap() {
  drawMap(await api("/api/map"));
}

refreshMap();
refreshSemanticMap();
refreshGoals();
refreshStatus();
setInterval(refreshStatus, 1000);
setInterval(refreshMap, 5000);
