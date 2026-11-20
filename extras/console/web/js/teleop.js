const MAX_VX = 0.28;
const MAX_WZ = 0.75;

export class Teleop {
  constructor({ onCommand, onState } = {}) {
    this.onCommand = onCommand || (() => {});
    this.onState = onState || (() => {});
    this.active = false;
    this.keys = new Set();
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
    let vx = 0;
    let wz = 0;

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
    if (!this.active) return;
    if (Math.abs(vx) < 0.01 && Math.abs(wz) < 0.01) return;
    this.onCommand(vx, wz);
  }
}
