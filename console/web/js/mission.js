const STATUS_LABEL = {
  PENDING: "待执行",
  DISPATCHING: "派发中",
  NAVIGATING: "巡航中",
  ARRIVED: "已到达",
  DWELLING: "停留中",
  DONE: "已完成",
  SKIPPED: "已跳过",
  FAILED: "失败",
};

const STATE_LABEL = {
  IDLE: "待命",
  RUNNING: "执行中",
  PAUSED: "已暂停",
  ESTOPPED: "急停",
  COMPLETED: "已完成",
  ABORTED: "已终止",
  INTERRUPTED: "中断待续",
};

export function renderMissionSelect(select, missions, selectedId) {
  const ids = missions.map((mission) => mission.id).join("|");
  if (select.dataset.ids !== ids) {
    select.dataset.ids = ids;
    const keep = selectedId || select.value;
    select.innerHTML = "";
    for (const mission of missions) {
      const option = document.createElement("option");
      option.value = mission.id;
      option.textContent = `${mission.name} · ${mission.total} 点`;
      select.appendChild(option);
    }
    if (keep && missions.some((mission) => mission.id === keep)) select.value = keep;
  }
}

export function renderMission(mission, refs) {
  if (!mission) {
    refs.state.textContent = "无任务";
    refs.progress.style.width = "0%";
    refs.step.textContent = "0 / 0";
    refs.robot.textContent = "—";
    refs.list.innerHTML = "";
    refs.list.dataset.sig = "";
    if (refs.stripName) refs.stripName.textContent = "—";
    if (refs.stripWaypoint) refs.stripWaypoint.textContent = "—";
    if (refs.stripDistance) refs.stripDistance.textContent = "—";
    return;
  }

  refs.state.textContent = STATE_LABEL[mission.state] || mission.state;
  const total = mission.total || mission.waypoints.length || 1;
  const done = mission.waypoints.filter((waypoint) =>
    ["DONE", "SKIPPED"].includes(waypoint.status)
  ).length;
  const percent = Math.round((done / total) * 100);
  refs.progress.style.width = `${percent}%`;
  refs.step.textContent = `${done} / ${total}`;
  refs.robot.textContent = mission.robot;

  if (refs.stripName) refs.stripName.textContent = mission.name;
  const current = mission.waypoints[mission.cursor];
  if (refs.stripWaypoint) {
    refs.stripWaypoint.textContent = current
      ? `${current.index + 1}. ${current.label || "巡检点"}`
      : "—";
  }
  if (refs.stripDistance) {
    refs.stripDistance.textContent =
      current && current.t_dispatched ? "执行中" : "—";
  }

  const signature = mission.waypoints
    .map((waypoint) => `${waypoint.index}:${waypoint.status}`)
    .join("|");
  if (refs.list.dataset.sig === signature && refs.list.dataset.cursor === String(mission.cursor)) {
    return;
  }
  refs.list.dataset.sig = signature;
  refs.list.dataset.cursor = String(mission.cursor);
  refs.list.innerHTML = "";
  mission.waypoints.forEach((waypoint, index) => {
    const item = document.createElement("li");
    item.className = `wp ${waypoint.status}${index === mission.cursor ? " current" : ""}`;
    item.innerHTML = `
      <span class="wp-index">${waypoint.index + 1}</span>
      <span class="wp-label">${escapeHtml(waypoint.label || `巡检点 ${index + 1}`)}</span>
      <span class="wp-meta">${STATUS_LABEL[waypoint.status] || waypoint.status} · ${Number(waypoint.dwell).toFixed(0)}s</span>`;
    refs.list.appendChild(item);
  });
}

export function missionStateLabel(state) {
  return STATE_LABEL[state] || state || "--";
}

function escapeHtml(text) {
  return String(text).replace(/[&<>"']/g, (character) => {
    return {
      "&": "&amp;",
      "<": "&lt;",
      ">": "&gt;",
      '"': "&quot;",
      "'": "&#39;",
    }[character];
  });
}
