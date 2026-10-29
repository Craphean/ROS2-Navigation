// WebGL view of the accumulated world-frame cloud.
// Drag to orbit, wheel to zoom, right-drag to pan.

const ROBOT_COLOR = {
  robot1: [0.22, 0.78, 0.83],
  robot2: [0.95, 0.64, 0.24],
};

function perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2);
  const nf = 1 / (near - far);
  return new Float32Array([
    f / aspect, 0, 0, 0,
    0, f, 0, 0,
    0, 0, (far + near) * nf, -1,
    0, 0, 2 * far * near * nf, 0,
  ]);
}

function lookAt(eye, center, up) {
  let zx = eye[0] - center[0];
  let zy = eye[1] - center[1];
  let zz = eye[2] - center[2];
  let len = Math.hypot(zx, zy, zz) || 1;
  zx /= len;
  zy /= len;
  zz /= len;
  let xx = up[1] * zz - up[2] * zy;
  let xy = up[2] * zx - up[0] * zz;
  let xz = up[0] * zy - up[1] * zx;
  len = Math.hypot(xx, xy, xz) || 1;
  xx /= len;
  xy /= len;
  xz /= len;
  const yx = zy * xz - zz * xy;
  const yy = zz * xx - zx * xz;
  const yz = zx * xy - zy * xx;
  return new Float32Array([
    xx, yx, zx, 0,
    xy, yy, zy, 0,
    xz, yz, zz, 0,
    -(xx * eye[0] + xy * eye[1] + xz * eye[2]),
    -(yx * eye[0] + yy * eye[1] + yz * eye[2]),
    -(zx * eye[0] + zy * eye[1] + zz * eye[2]),
    1,
  ]);
}

function multiply(a, b) {
  const out = new Float32Array(16);
  for (let col = 0; col < 4; col += 1) {
    for (let row = 0; row < 4; row += 1) {
      let sum = 0;
      for (let k = 0; k < 4; k += 1) sum += a[k * 4 + row] * b[col * 4 + k];
      out[col * 4 + row] = sum;
    }
  }
  return out;
}

const VERT = `#version 300 es
precision highp float;
layout(location=0) in vec3 a_pos;
uniform mat4 u_mvp;
uniform float u_z0;
uniform float u_z1;
uniform float u_size;
out float v_h;
void main() {
  gl_Position = u_mvp * vec4(a_pos, 1.0);
  v_h = clamp((a_pos.z - u_z0) / max(u_z1 - u_z0, 0.05), 0.0, 1.0);
  gl_PointSize = u_size;
}`;

const FRAG = `#version 300 es
precision mediump float;
in float v_h;
uniform vec3 u_color;
uniform float u_use_height;
out vec4 o;
void main() {
  vec2 p = gl_PointCoord * 2.0 - 1.0;
  if (dot(p, p) > 1.0) discard;
  vec3 low = vec3(0.18, 0.32, 0.48);
  vec3 high = vec3(0.45, 0.90, 0.86);
  vec3 color = mix(u_color, mix(low, high, v_h), u_use_height);
  o = vec4(color, 0.92);
}`;

export class CloudView {
  constructor(canvas) {
    this.canvas = canvas;
    this.gl = canvas.getContext("webgl2", { antialias: false, alpha: false, depth: true });
    this.ok = Boolean(this.gl);
    this.count = 0;
    this.z0 = 0;
    this.z1 = 2;
    this.yaw = 0.8;
    this.pitch = 0.55;
    this.distance = 16;
    this.target = [0, 0, 0.6];
    this.poses = [];
    this.visible = false;
    this._seq = -1;
    if (!this.ok) return;
    const gl = this.gl;
    this.program = this._program(gl, VERT, FRAG);
    this.buffer = gl.createBuffer();
    this.robotBuffer = gl.createBuffer();
    this._bindPointer();
    gl.enable(gl.DEPTH_TEST);
    gl.clearColor(0.024, 0.035, 0.047, 1);
    canvas.addEventListener("contextmenu", (event) => event.preventDefault());
    canvas.addEventListener("pointerdown", (event) => this._down(event));
    canvas.addEventListener("pointermove", (event) => this._move(event));
    canvas.addEventListener("pointerup", () => { this.pointer = null; });
    canvas.addEventListener("pointercancel", () => { this.pointer = null; });
    canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      this.distance = Math.max(1.5, Math.min(80, this.distance * (event.deltaY < 0 ? 0.9 : 1.1)));
    }, { passive: false });
    const loop = () => {
      if (this.visible) this.draw();
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  resize() {
    const parent = this.canvas.parentElement || this.canvas;
    const width = Math.max(1, parent.clientWidth);
    const height = Math.max(1, parent.clientHeight);
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    this.canvas.width = Math.round(width * dpr);
    this.canvas.height = Math.round(height * dpr);
    this.canvas.style.width = `${width}px`;
    this.canvas.style.height = `${height}px`;
    this.width = width;
    this.height = height;
    if (this.gl) this.gl.viewport(0, 0, this.canvas.width, this.canvas.height);
  }

  setBuffer(buffer, seq) {
    if (!this.ok || seq === this._seq) return;
    const view = new DataView(buffer);
    const count = view.getUint32(0, true);
    if (count <= 0 || buffer.byteLength < 4 + count * 12) {
      this.count = 0;
      this._seq = seq;
      const gl = this.gl;
      if (gl) {
        gl.bindBuffer(gl.ARRAY_BUFFER, this.buffer);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(0), gl.DYNAMIC_DRAW);
      }
      return;
    }
    const points = new Float32Array(buffer, 4, count * 3);
    this.count = count;
    this._seq = seq;
    let z0 = Infinity;
    let z1 = -Infinity;
    let sx = 0;
    let sy = 0;
    let sz = 0;
    const step = Math.max(1, Math.floor(count / 4000));
    let used = 0;
    for (let i = 0; i < count; i += step) {
      const z = points[i * 3 + 2];
      sx += points[i * 3];
      sy += points[i * 3 + 1];
      sz += z;
      if (z < z0) z0 = z;
      if (z > z1) z1 = z;
      used += 1;
    }
    this.z0 = Number.isFinite(z0) ? z0 : 0;
    this.z1 = Number.isFinite(z1) ? z1 : 2;
    if (!this.fitted && used > 0) {
      this.target = [sx / used, sy / used, sz / used];
      this.distance = Math.max(8, (this.z1 - this.z0) * 8);
      this.fitted = true;
    }
    const gl = this.gl;
    gl.bindBuffer(gl.ARRAY_BUFFER, this.buffer);
    gl.bufferData(gl.ARRAY_BUFFER, points, gl.DYNAMIC_DRAW);
  }

  setPoses(robots) {
    const list = [];
    for (const ns of Object.keys(robots || {})) {
      const pose = robots[ns]?.pose;
      if (!pose) continue;
      list.push({ ns, x: pose[0], y: pose[1], z: 0.2 });
    }
    this.poses = list;
  }

  fit() {
    this.fitted = false;
    this.yaw = 0.8;
    this.pitch = 0.55;
  }

  draw() {
    const gl = this.gl;
    if (!gl || !this.width) return;
    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    if (!this.count) return;
    const aspect = this.width / Math.max(1, this.height);
    const eye = this._eye();
    const view = lookAt(eye, this.target, [0, 0, 1]);
    const proj = perspective(Math.PI / 4, aspect, 0.05, 200);
    const mvp = multiply(proj, view);
    gl.useProgram(this.program);
    gl.uniformMatrix4fv(gl.getUniformLocation(this.program, "u_mvp"), false, mvp);
    gl.uniform1f(gl.getUniformLocation(this.program, "u_z0"), this.z0);
    gl.uniform1f(gl.getUniformLocation(this.program, "u_z1"), this.z1);
    gl.uniform1f(gl.getUniformLocation(this.program, "u_size"), 2.2);
    gl.uniform1f(gl.getUniformLocation(this.program, "u_use_height"), 1);
    gl.uniform3f(gl.getUniformLocation(this.program, "u_color"), 1, 1, 1);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.buffer);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    gl.drawArrays(gl.POINTS, 0, this.count);

    if (!this.poses.length) return;
    const marker = new Float32Array(this.poses.length * 3);
    this.poses.forEach((pose, index) => {
      marker[index * 3] = pose.x;
      marker[index * 3 + 1] = pose.y;
      marker[index * 3 + 2] = pose.z;
    });
    gl.bindBuffer(gl.ARRAY_BUFFER, this.robotBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, marker, gl.DYNAMIC_DRAW);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    gl.uniform1f(gl.getUniformLocation(this.program, "u_size"), 14);
    gl.uniform1f(gl.getUniformLocation(this.program, "u_use_height"), 0);
    this.poses.forEach((pose, index) => {
      const color = ROBOT_COLOR[pose.ns] || [1, 1, 1];
      gl.uniform3fv(gl.getUniformLocation(this.program, "u_color"), color);
      gl.drawArrays(gl.POINTS, index, 1);
    });
  }

  _eye() {
    const cp = Math.cos(this.pitch);
    const sp = Math.sin(this.pitch);
    return [
      this.target[0] + this.distance * cp * Math.cos(this.yaw),
      this.target[1] + this.distance * cp * Math.sin(this.yaw),
      this.target[2] + this.distance * sp,
    ];
  }

  _down(event) {
    this.canvas.setPointerCapture(event.pointerId);
    this.pointer = { id: event.pointerId, x: event.clientX, y: event.clientY, button: event.button };
  }

  _move(event) {
    if (!this.pointer || this.pointer.id !== event.pointerId) return;
    const dx = event.clientX - this.pointer.x;
    const dy = event.clientY - this.pointer.y;
    this.pointer.x = event.clientX;
    this.pointer.y = event.clientY;
    if (this.pointer.button === 2 || event.shiftKey) {
      // Grab-and-drag: the cloud follows the cursor. For lookAt(eye, target,
      // +Z) the screen-up axis is cross(right, forward), i.e.
      // (-cos(yaw)sin(pitch), -sin(yaw)sin(pitch), cos(pitch)); the old code
      // added the horizontal view direction instead, which made vertical
      // drags feel inverted and mostly change depth.
      const scale = this.distance / Math.max(200, this.height);
      const sp = Math.sin(this.pitch);
      const cp = Math.cos(this.pitch);
      const rightx = -Math.sin(this.yaw);
      const righty = Math.cos(this.yaw);
      this.target[0] += (-dx * rightx - dy * Math.cos(this.yaw) * sp) * scale;
      this.target[1] += (-dx * righty - dy * Math.sin(this.yaw) * sp) * scale;
      this.target[2] += dy * cp * scale;
      return;
    }
    this.yaw -= dx * 0.008;
    this.pitch = Math.max(-0.2, Math.min(1.35, this.pitch + dy * 0.006));
  }

  _bindPointer() {
    const gl = this.gl;
    gl.bindBuffer(gl.ARRAY_BUFFER, this.buffer);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
  }

  _program(gl, vs, fs) {
    const compile = (type, source) => {
      const shader = gl.createShader(type);
      gl.shaderSource(shader, source);
      gl.compileShader(shader);
      return shader;
    };
    const program = gl.createProgram();
    gl.attachShader(program, compile(gl.VERTEX_SHADER, vs));
    gl.attachShader(program, compile(gl.FRAGMENT_SHADER, fs));
    gl.bindAttribLocation(program, 0, "a_pos");
    gl.linkProgram(program);
    return program;
  }
}
