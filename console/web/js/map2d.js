const STATUS_COLOR = {
  PENDING: "#5d6b78",
  DISPATCHING: "#37c8d4",
  NAVIGATING: "#37c8d4",
  ARRIVED: "#39d98a",
  DWELLING: "#f2a33c",
  DONE: "#39d98a",
  SKIPPED: "#44515d",
  FAILED: "#e5484d",
};

const ROBOT_COLOR = { robot1: "#37c8d4", robot2: "#f2a33c" };

export class MapView {
  constructor(canvas, { onPick } = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.onPick = onPick;
    this.meta = null;
    this.snapshot = null;
    this.waypoints = [];
    this.image = new Image();
    this.image.decoding = "async";
    this.image.onload = () => {
      this.dirty = true;
    };
    this.imageSeq = -1;
    this.center = { x: 0, y: 0 };
    this.scale = 40;
    this.fitted = false;
    this.dirty = true;
    this.dpr = Math.min(2, window.devicePixelRatio || 1);
    this.pointer = null;
    this.showScan = true;

    this.bindEvents();
    this.resize();
    const observer = new ResizeObserver(() => this.resize());
    observer.observe(canvas.parentElement || canvas);
    const loop = () => {
      if (this.dirty) this.draw();
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  resize() {
    const parent = this.canvas.parentElement || this.canvas;
    const width = Math.max(1, parent.clientWidth);
    const height = Math.max(1, parent.clientHeight);
    this.dpr = Math.min(2, window.devicePixelRatio || 1);
    this.canvas.width = Math.round(width * this.dpr);
    this.canvas.height = Math.round(height * this.dpr);
    this.canvas.style.width = `${width}px`;
    this.canvas.style.height = `${height}px`;
    this.width = width;
    this.height = height;
    if (!this.fitted && this.meta) this.fit();
    this.dirty = true;
  }

  setSnapshot(snapshot) {
    this.snapshot = snapshot;
    const meta = snapshot ? snapshot.map : null;
    if (meta) {
      const first = !this.meta;
      this.meta = meta;
      if (meta.seq !== this.imageSeq) {
        this.imageSeq = meta.seq;
        this.image.src = `/api/map.png?v=${meta.seq}`;
      }
      if (first && !this.fitted) this.fit();
    }
    this.dirty = true;
  }

  setWaypoints(waypoints) {
    this.waypoints = waypoints || [];
    this.dirty = true;
  }

  fit() {
    if (!this.meta || !this.width) return;
    const { width, height, resolution, origin } = this.meta;
    const worldW = width * resolution;
    const worldH = height * resolution;
    const padding = 46;
    const scaleX = (this.width - padding * 2) / Math.max(1, worldW);
    const scaleY = (this.height - padding * 2) / Math.max(1, worldH);
    this.scale = Math.max(4, Math.min(scaleX, scaleY));
    this.center = { x: origin[0] + worldW / 2, y: origin[1] + worldH / 2 };
    this.fitted = true;
    this.dirty = true;
  }

  worldToScreen(x, y) {
    return {
      x: (x - this.center.x) * this.scale + this.width / 2,
      y: this.height / 2 - (y - this.center.y) * this.scale,
    };
  }

  screenToWorld(px, py) {
    return {
      x: (px - this.width / 2) / this.scale + this.center.x,
      y: (this.height / 2 - py) / this.scale + this.center.y,
    };
  }

  bindEvents() {
    const canvas = this.canvas;

    canvas.addEventListener("pointerdown", (event) => {
      canvas.setPointerCapture(event.pointerId);
      this.pointer = {
        id: event.pointerId,
        x: event.clientX,
        y: event.clientY,
        startX: event.clientX,
        startY: event.clientY,
        moved: false,
      };
      canvas.classList.add("dragging");
    });

    canvas.addEventListener("pointermove", (event) => {
      if (!this.pointer || this.pointer.id !== event.pointerId) return;
      const dx = event.clientX - this.pointer.x;
      const dy = event.clientY - this.pointer.y;
      if (Math.abs(event.clientX - this.pointer.startX) > 4 ||
          Math.abs(event.clientY - this.pointer.startY) > 4) {
        this.pointer.moved = true;
      }
      this.center.x -= dx / this.scale;
      this.center.y += dy / this.scale;
      this.pointer.x = event.clientX;
      this.pointer.y = event.clientY;
      this.dirty = true;
    });

    const finish = (event) => {
      if (!this.pointer || this.pointer.id !== event.pointerId) return;
      const wasClick = !this.pointer.moved;
      const rect = canvas.getBoundingClientRect();
      const px = event.clientX - rect.left;
      const py = event.clientY - rect.top;
      this.pointer = null;
      canvas.classList.remove("dragging");
      if (wasClick && this.onPick && px >= 0 && py >= 0 && px <= this.width && py <= this.height) {
        this.onPick(this.screenToWorld(px, py));
      }
    };
    canvas.addEventListener("pointerup", finish);
    canvas.addEventListener("pointercancel", () => {
      this.pointer = null;
      canvas.classList.remove("dragging");
    });

    canvas.addEventListener(
      "wheel",
      (event) => {
        event.preventDefault();
        const rect = canvas.getBoundingClientRect();
        const px = event.clientX - rect.left;
        const py = event.clientY - rect.top;
        const before = this.screenToWorld(px, py);
        const factor = event.deltaY < 0 ? 1.12 : 1 / 1.12;
        this.scale = Math.max(3, Math.min(240, this.scale * factor));
        const after = this.screenToWorld(px, py);
        this.center.x += before.x - after.x;
        this.center.y += before.y - after.y;
        this.dirty = true;
      },
      { passive: false }
    );
  }

  draw() {
    this.dirty = false;
    const ctx = this.ctx;
    const width = this.width;
    const height = this.height;
    if (!width || !height) return;

    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);

    this.drawGrid(ctx);
    if (this.meta && this.image.complete && this.image.naturalWidth > 0) {
      const { width: cells, height: rows, resolution, origin } = this.meta;
      const topLeft = this.worldToScreen(origin[0], origin[1] + rows * resolution);
      ctx.imageSmoothingEnabled = false;
      ctx.globalAlpha = 0.95;
      ctx.drawImage(
        this.image,
        topLeft.x,
        topLeft.y,
        cells * resolution * this.scale,
        rows * resolution * this.scale
      );
      ctx.globalAlpha = 1;
    }

    this.drawScan(ctx);
    this.drawGoals(ctx);
    this.drawWaypoints(ctx);
    this.drawRobots(ctx);
    this.drawScale(ctx);
  }

  drawGrid(ctx) {
    const step = this.scale > 70 ? 1 : this.scale > 26 ? 5 : 10;
    const left = this.center.x - this.width / 2 / this.scale;
    const right = this.center.x + this.width / 2 / this.scale;
    const bottom = this.center.y - this.height / 2 / this.scale;
    const top = this.center.y + this.height / 2 / this.scale;
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(120, 160, 180, 0.07)";
    ctx.beginPath();
    for (let x = Math.ceil(left / step) * step; x <= right; x += step) {
      const p = this.worldToScreen(x, 0);
      ctx.moveTo(Math.round(p.x) + 0.5, 0);
      ctx.lineTo(Math.round(p.x) + 0.5, this.height);
    }
    for (let y = Math.ceil(bottom / step) * step; y <= top; y += step) {
      const p = this.worldToScreen(0, y);
      ctx.moveTo(0, Math.round(p.y) + 0.5);
      ctx.lineTo(this.width, Math.round(p.y) + 0.5);
    }
    ctx.stroke();
  }

  drawScan(ctx) {
    if (!this.showScan || !this.snapshot || !this.snapshot.robots) return;
    for (const ns of Object.keys(this.snapshot.robots)) {
      const robot = this.snapshot.robots[ns];
      const color = ROBOT_COLOR[ns] || "#37c8d4";
      if (!robot.scan || !robot.scan.length) continue;
      ctx.fillStyle = color;
      ctx.globalAlpha = 0.55;
      for (const point of robot.scan) {
        const p = this.worldToScreen(point[0], point[1]);
        ctx.fillRect(p.x - 0.9, p.y - 0.9, 1.8, 1.8);
      }
      ctx.globalAlpha = 1;
    }
  }

  drawGoals(ctx) {
    if (!this.snapshot || !this.snapshot.robots) return;
    for (const ns of Object.keys(this.snapshot.robots)) {
      const robot = this.snapshot.robots[ns];
      if (!robot.pose || !robot.goal) continue;
      const from = this.worldToScreen(robot.pose[0], robot.pose[1]);
      const to = this.worldToScreen(robot.goal[0], robot.goal[1]);
      const color = ROBOT_COLOR[ns] || "#37c8d4";
      ctx.save();
      ctx.strokeStyle = color;
      ctx.globalAlpha = 0.4;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(from.x, from.y);
      ctx.lineTo(to.x, to.y);
      ctx.stroke();
      ctx.restore();

      ctx.save();
      ctx.strokeStyle = color;
      ctx.globalAlpha = 0.85;
      ctx.beginPath();
      ctx.arc(to.x, to.y, 7, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(to.x - 3, to.y);
      ctx.lineTo(to.x + 3, to.y);
      ctx.moveTo(to.x, to.y - 3);
      ctx.lineTo(to.x, to.y + 3);
      ctx.stroke();
      ctx.restore();
    }
  }

  drawWaypoints(ctx) {
    this.waypoints.forEach((waypoint, index) => {
      const p = this.worldToScreen(waypoint.x, waypoint.y);
      const color = STATUS_COLOR[waypoint.status] || STATUS_COLOR.PENDING;
      const radius = 10;
      ctx.beginPath();
      ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(8, 13, 18, 0.9)";
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = color;
      ctx.stroke();

      ctx.fillStyle = color;
      ctx.font = "600 9px ui-monospace, monospace";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(String(index + 1), p.x, p.y + 0.5);

      if (waypoint.label) {
        ctx.font = "9px ui-monospace, monospace";
        ctx.fillStyle = "rgba(214, 228, 238, 0.72)";
        ctx.textAlign = "left";
        ctx.fillText(waypoint.label, p.x + 14, p.y + 3);
      }
    });
  }

  drawRobots(ctx) {
    if (!this.snapshot || !this.snapshot.robots) return;
    for (const ns of Object.keys(this.snapshot.robots)) {
      const robot = this.snapshot.robots[ns];
      if (!robot.pose) continue;
      const color = ROBOT_COLOR[ns] || "#37c8d4";
      const p = this.worldToScreen(robot.pose[0], robot.pose[1]);
      const yaw = robot.pose[2] || 0;

      ctx.save();
      ctx.translate(p.x, p.y);

      ctx.beginPath();
      ctx.arc(0, 0, 17, 0, Math.PI * 2);
      ctx.strokeStyle = color;
      ctx.globalAlpha = 0.25;
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.globalAlpha = 1;

      ctx.rotate(-yaw);
      ctx.beginPath();
      ctx.moveTo(13, 0);
      ctx.lineTo(-8, 9);
      ctx.lineTo(-4, 0);
      ctx.lineTo(-8, -9);
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.shadowColor = color;
      ctx.shadowBlur = 12;
      ctx.fill();
      ctx.restore();

      ctx.font = "600 9px ui-monospace, monospace";
      ctx.fillStyle = color;
      ctx.textAlign = "center";
      ctx.fillText(ns.toUpperCase(), p.x, p.y - 23);
    }
  }

  drawScale(ctx) {
    const targetPx = 90;
    const raw = targetPx / this.scale;
    const nice = [0.5, 1, 2, 5, 10, 20, 50].find((value) => value >= raw) || 50;
    const length = nice * this.scale;
    const x = this.width - 18 - length;
    const y = this.height - 20;
    ctx.strokeStyle = "rgba(230, 237, 243, 0.65)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + length, y);
    ctx.moveTo(x, y - 4);
    ctx.lineTo(x, y + 4);
    ctx.moveTo(x + length, y - 4);
    ctx.lineTo(x + length, y + 4);
    ctx.stroke();
    ctx.fillStyle = "rgba(230, 237, 243, 0.75)";
    ctx.font = "9px ui-monospace, monospace";
    ctx.textAlign = "center";
    ctx.fillText(`${nice} m`, x + length / 2, y - 6);
  }
}
