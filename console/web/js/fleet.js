const STATE_LABEL = {
  IDLE: "待命",
  PLANNING: "规划中",
  NAVIGATING: "巡航中",
  RECOVERY: "脱困中",
  ESTOPPED: "急停",
};

export function updateFleet(container, snapshot, mission) {
  if (!snapshot || !snapshot.robots) return;
  const robots = snapshot.robots;
  for (const ns of Object.keys(robots)) {
    let card = container.querySelector(`.robot-card[data-ns="${ns}"]`);
    if (!card) {
      card = document.createElement("div");
      card.className = "robot-card";
      card.dataset.ns = ns;
      card.innerHTML = `
        <div class="rc-head">
          <span class="rc-dot"></span>
          <span class="rc-name">${ns.toUpperCase()}</span>
          <span class="rc-badge" data-f="badge">IDLE</span>
        </div>
        <div class="rc-grid">
          <div><label>X</label><span data-f="x">--</span></div>
          <div><label>Y</label><span data-f="y">--</span></div>
          <div><label>YAW</label><span data-f="yaw">--</span></div>
          <div><label>V m/s</label><span data-f="v">--</span></div>
          <div><label>W rad/s</label><span data-f="w">--</span></div>
          <div><label>目标</label><span data-f="goal">--</span></div>
        </div>
        <div class="rc-goal"><span data-f="state">待命</span><span data-f="dist">--</span></div>`;
      container.appendChild(card);
    }

    const robot = robots[ns];
    const set = (field, text) => {
      const element = card.querySelector(`[data-f="${field}"]`);
      if (element) element.textContent = text;
    };
    const pose = robot.pose;
    set("x", pose ? Number(pose[0]).toFixed(2) : "--");
    set("y", pose ? Number(pose[1]).toFixed(2) : "--");
    set("yaw", pose ? `${(Number(pose[2]) * 180 / Math.PI).toFixed(0)}°` : "--");
    set("v", Number(robot.vel?.[0] || 0).toFixed(2));
    set("w", Number(robot.vel?.[1] || 0).toFixed(2));
    set(
      "goal",
      robot.goal ? `${Number(robot.goal[0]).toFixed(1)},${Number(robot.goal[1]).toFixed(1)}` : "—"
    );
    set("badge", robot.state || "IDLE");
    set("state", STATE_LABEL[robot.state] || robot.state || "待命");
    set(
      "dist",
      robot.distance_remaining === null || robot.distance_remaining === undefined
        ? "—"
        : `${Number(robot.distance_remaining).toFixed(1)} m`
    );

    card.classList.toggle("active", Boolean(mission && mission.robot === ns && mission.state === "RUNNING"));
    card.classList.toggle("estopped", robot.state === "ESTOPPED");
  }
}
