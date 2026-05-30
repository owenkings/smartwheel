class DeviceStrip {
  constructor(root) {
    this.root = root;
    this.devices = [
      ["XT-M60", "laser"],
      ["Scan", "scan"],
      ["IMU", "imu"],
      ["Base", "base"],
      ["Odom", "odom"],
    ];
    this.cameraLabels = [
      ["Front Cam", "camera_front"],
      ["Left Cam", "camera_left"],
      ["Right Cam", "camera_right"],
      ["Rear Cam", "camera_rear"],
    ];
  }

  update(sensors = {}) {
    this.root.innerHTML = "";
    const devices = [
      ...this.devices.slice(0, 3),
      ...this.cameraLabels.filter(([, key]) => key in sensors),
      ...this.devices.slice(3),
    ];
    devices.forEach(([label, key]) => {
      const item = document.createElement("span");
      const entry = sensors[key] || {};
      item.className = `device-strip-item ${this.state(entry)}`;
      item.innerHTML = `<i></i><b>${label}</b><em>${this.age(entry.age_sec)}</em>`;
      this.root.append(item);
    });
    const ultrasonicOnline = (sensors.ultrasonic || []).filter((entry) => entry.online).length;
    const ultrasonicTotal = (sensors.ultrasonic || []).length;
    const ultra = document.createElement("span");
    ultra.className = `device-strip-item ${ultrasonicOnline ? "online" : "offline"}`;
    ultra.innerHTML = `<i></i><b>Ultrasonic</b><em>${ultrasonicOnline}/${ultrasonicTotal}</em>`;
    this.root.append(ultra);
  }

  state(entry) {
    if (entry.online) return "online";
    if ((entry.messages || 0) > 0) return "stale";
    return "offline";
  }

  age(age) {
    if (age === null || age === undefined) return "--";
    if (age < 1) return `${Math.round(age * 1000)}ms`;
    return `${age.toFixed(1)}s`;
  }
}

class PassabilityWidget {
  constructor() {
    this.chip = document.getElementById("passability-chip");
    this.orb = document.getElementById("passability-orb");
    this.width = document.getElementById("passability-width");
    this.required = document.getElementById("passability-required");
    this.reason = document.getElementById("passability-reason");
  }

  update(status = {}) {
    const data = status.passability_status || {};
    const state = String(data.state || "UNKNOWN").toUpperCase();
    const klass = this.classForState(state);
    this.chip.textContent = state;
    this.chip.className = `mini-chip ${klass}`;
    this.orb.className = `passability-orb ${klass}`;
    this.orb.textContent = this.shortText(state);
    this.width.textContent = this.meters(data.estimated_width_m);
    this.required.textContent = this.meters(data.required_width_m);
    this.reason.textContent = data.reason || "等待通行性分析";
  }

  classForState(state) {
    if (state === "CLEAR") return "ok";
    if (state === "NARROW" || state === "UNKNOWN") return "warn";
    if (state === "BLOCKED" || state === "ERROR") return "bad";
    return "neutral";
  }

  shortText(state) {
    return {
      CLEAR: "通",
      NARROW: "窄",
      BLOCKED: "阻",
      UNKNOWN: "?",
    }[state] || "--";
  }

  meters(value) {
    if (value === null || value === undefined || value === "") return "--";
    return Number.isFinite(Number(value)) ? `${Number(value).toFixed(2)} m` : "--";
  }
}

class BevWidget {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.rangeM = 5.0;
  }

  update(status = {}) {
    this.resize();
    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#111827";
    ctx.fillRect(0, 0, width, height);
    this.drawGrid();
    this.drawSafetyZones();
    this.drawUltrasonic(status.sensors?.ultrasonic || []);
    this.drawWheelchair();
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const width = Math.max(260, Math.round(rect.width));
    const height = Math.max(220, Math.round(rect.height));
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
  }

  toCanvas(x, y) {
    const scale = Math.min(this.canvas.width, this.canvas.height) / (this.rangeM * 2);
    return {
      x: this.canvas.width / 2 + y * scale,
      y: this.canvas.height * 0.78 - x * scale,
    };
  }

  drawGrid() {
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = "rgba(255,255,255,0.10)";
    ctx.lineWidth = 1;
    for (let meter = -this.rangeM; meter <= this.rangeM; meter += 1) {
      const a = this.toCanvas(-this.rangeM, meter);
      const b = this.toCanvas(this.rangeM, meter);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
      const c = this.toCanvas(meter, -this.rangeM);
      const d = this.toCanvas(meter, this.rangeM);
      ctx.beginPath();
      ctx.moveTo(c.x, c.y);
      ctx.lineTo(d.x, d.y);
      ctx.stroke();
    }
    ctx.restore();
  }

  drawSafetyZones() {
    const ctx = this.ctx;
    const corners = [
      this.toCanvas(0.0, -0.45),
      this.toCanvas(1.8, -0.45),
      this.toCanvas(1.8, 0.45),
      this.toCanvas(0.0, 0.45),
    ];
    ctx.beginPath();
    ctx.moveTo(corners[0].x, corners[0].y);
    corners.slice(1).forEach((point) => ctx.lineTo(point.x, point.y));
    ctx.closePath();
    ctx.fillStyle = "rgba(22, 138, 86, 0.22)";
    ctx.strokeStyle = "rgba(22, 138, 86, 0.72)";
    ctx.lineWidth = 1.5;
    ctx.fill();
    ctx.stroke();
  }

  drawWheelchair() {
    const ctx = this.ctx;
    const body = [
      this.toCanvas(-0.35, -0.32),
      this.toCanvas(0.45, -0.32),
      this.toCanvas(0.45, 0.32),
      this.toCanvas(-0.35, 0.32),
    ];
    ctx.beginPath();
    ctx.moveTo(body[0].x, body[0].y);
    body.slice(1).forEach((point) => ctx.lineTo(point.x, point.y));
    ctx.closePath();
    ctx.fillStyle = "rgba(47, 128, 237, 0.72)";
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.fill();
    ctx.stroke();
    const nose = this.toCanvas(0.68, 0);
    const center = this.toCanvas(0.1, 0);
    ctx.beginPath();
    ctx.moveTo(center.x, center.y);
    ctx.lineTo(nose.x, nose.y);
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 3;
    ctx.stroke();
  }

  drawUltrasonic(items) {
    const ctx = this.ctx;
    const angles = [-70, -35, 0, 35, 70, 110];
    items.forEach((item, index) => {
      const range = Number(item.range_m);
      if (!item.online || !Number.isFinite(range)) return;
      const theta = (angles[index] || 0) * Math.PI / 180;
      const x = Math.cos(theta) * range;
      const y = Math.sin(theta) * range;
      const start = this.toCanvas(0, 0);
      const end = this.toCanvas(x, y);
      ctx.beginPath();
      ctx.moveTo(start.x, start.y);
      ctx.lineTo(end.x, end.y);
      ctx.strokeStyle = range < 0.5 ? "#ef4444" : range < 1.5 ? "#f59e0b" : "#22c55e";
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(end.x, end.y, 4, 0, Math.PI * 2);
      ctx.fillStyle = ctx.strokeStyle;
      ctx.fill();
    });
  }
}

class CameraStatusGrid {
  constructor(root) {
    this.root = root;
  }

  update(sensors = {}) {
    const cameras = [
      ["camera_front", "前摄像头", sensors.camera_front || {}],
      ["camera_left", "左摄像头", sensors.camera_left || {}],
      ["camera_right", "右摄像头", sensors.camera_right || {}],
      ["camera_rear", "后摄像头", sensors.camera_rear || {}],
    ].filter(([key]) => key in sensors);
    this.root.innerHTML = "";
    cameras.forEach(([, label, camera]) => {
      const card = document.createElement("div");
      card.className = `camera-card ${camera.online ? "online" : "offline"}`;
      card.innerHTML = `
        <div class="camera-preview">${camera.online ? "LIVE" : "NO SIGNAL"}</div>
        <div class="camera-meta">
          <b>${label}</b>
          <span>${camera.width || "--"} x ${camera.height || "--"} ${camera.encoding || ""}</span>
          <small>${this.age(camera.age_sec)}</small>
        </div>
      `;
      this.root.append(card);
    });
    const online = cameras.filter(([, , camera]) => camera.online).length;
    document.getElementById("camera-summary").textContent = `${online}/${cameras.length}`;
  }

  age(age) {
    if (age === null || age === undefined) return "未收到";
    if (age < 1) return `${Math.round(age * 1000)}ms`;
    return `${age.toFixed(1)}s`;
  }
}
