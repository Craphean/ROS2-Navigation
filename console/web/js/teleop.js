const MAX_VX = 0.28;
const MAX_WZ = 0.75;

export class Teleop {
  constructor({ onCommand, onState } = {}) {
    this.onCommand = onCommand || (() => {});
    this.onState = onState || (() => {});
    this.active = false;
    this.joystick = { x: 0, y: 0, pointer: null };
    this.keys = new Set();
    this.joyElement = document.getElementById("joystick");
    this.stickElement = document.getElementById("joy-stick");
    this.vxElement = document.getElementById("joy-vx");
    this.wzElement = document.getElementById("joy-wz");
    this.bindJoystick();
    this.bindKeyboard();
    setInterval(() => this.tick(), 100);
  }

  setActive(active) {
    this.active = active;
    if (!active) this.reset();
    this.onState(active);
  }

  reset() {
    this.keys.clear();
    this.joystick.x = 0;
    this.joystick.y = 0;
    this.joystick.pointer = null;
    if (this.stickElement) this.stickElement.style.transform = "translate(0px, 0px)";
    if (this.vxElement) this.vxElement.textContent = "0.00";
    if (this.wzElement) this.wzElement.textContent = "0.00";
  }

  bindJoystick() {
    const element = this.joyElement;
    if (!element) return;
    const radius = 34;

    const update = (event) => {
      const rect = element.getBoundingClientRect();
      const cx = rect.left + rect.width / 2;
      const cy = rect.top + rect.height / 2;
      let dx = (event.clientX - cx) / radius;
      let dy = (event.clientY - cy) / radius;
      const length = Math.hypot(dx, dy);
      if (length > 1) {
        dx /= length;
        dy /= length;
      }
      this.joystick.x = dx;
      this.joystick.y = dy;
      if (this.stickElement) {
        this.stickElement.style.transform = `translate(${dx * radius}px, ${dy * radius}px)`;
      }
    };

    element.addEventListener("pointerdown", (event) => {
      if (!this.active) return;
      element.setPointerCapture(event.pointerId);
      this.joystick.pointer = event.pointerId;
      update(event);
    });
    element.addEventListener("pointermove", (event) => {
      if (this.joystick.pointer !== event.pointerId) return;
      update(event);
    });
    const release = (event) => {
      if (this.joystick.pointer !== event.pointerId) return;
      this.joystick.pointer = null;
      this.joystick.x = 0;
      this.joystick.y = 0;
      if (this.stickElement) this.stickElement.style.transform = "translate(0px, 0px)";
    };
    element.addEventListener("pointerup", release);
    element.addEventListener("pointercancel", release);
  }

  bindKeyboard() {
    window.addEventListener("keydown", (event) => {
      if (event.target instanceof HTMLInputElement) return;
      const code = event.code;
      if (["KeyW", "KeyA", "KeyS", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(code)) {
        event.preventDefault();
        this.keys.add(code);
      }
    });
    window.addEventListener("keyup", (event) => {
      this.keys.delete(event.code);
    });
    window.addEventListener("blur", () => this.keys.clear());
  }

  vector() {
    let vx = -this.joystick.y * MAX_VX;
    let wz = -this.joystick.x * MAX_WZ;

    const forward = this.keys.has("KeyW") || this.keys.has("ArrowUp");
    const backward = this.keys.has("KeyS") || this.keys.has("ArrowDown");
    const left = this.keys.has("KeyA") || this.keys.has("ArrowLeft");
    const right = this.keys.has("KeyD") || this.keys.has("ArrowRight");
    if (forward) vx += MAX_VX;
    if (backward) vx -= MAX_VX;
    if (left) wz += MAX_WZ;
    if (right) wz -= MAX_WZ;

    vx = Math.max(-0.15, Math.min(MAX_VX, vx));
    wz = Math.max(-MAX_WZ, Math.min(MAX_WZ, wz));
    return { vx, wz };
  }

  tick() {
    const { vx, wz } = this.vector();
    if (this.vxElement) this.vxElement.textContent = vx.toFixed(2);
    if (this.wzElement) this.wzElement.textContent = wz.toFixed(2);
    if (!this.active) return;
    if (Math.abs(vx) < 0.01 && Math.abs(wz) < 0.01) return;
    this.onCommand(vx, wz);
  }
}
