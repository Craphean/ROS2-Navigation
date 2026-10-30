import { $, api, toast } from "./utils.js";
import { Link } from "./ws.js?v=3";
import { MapView } from "./map2d.js?v=5";
import { CloudView } from "./cloud3d.js?v=8";
import { updateFleet } from "./fleet.js";
import { renderMission, renderMissionSelect } from "./mission.js";
import { Teleop } from "./teleop.js";

const state = {
  snapshot: null,
  role: "viewer",
  missions: [],
  selectedId: localStorage.getItem("mine_selected_mission") || "",
  missionDetail: null,
  clickMode: "nav",
  target: "robot1",
  leased: false,
  events: [],
  sim: { sim: { running: false }, exploration: { running: false } },
};

const missionRefs = {
  state: $("#mission-state"),
  progress: $("#mission-progress"),
  step: $("#mission-step"),
  robot: $("#mission-robot-label"),
  list: $("#waypoint-list"),
  stripName: $("#strip-name"),
  stripWaypoint: $("#strip-waypoint"),
  stripDistance: $("#strip-distance"),
};

const mapView = new MapView($("#map-canvas"), {
  onPick: (point) => handleMapPick(point),
});
const cloudView = new CloudView($("#cloud-canvas"));
let viewMode = "2d";
let cloudSeq = -1;

const teleop = new Teleop({
  onCommand: (vx, wz) => {
    link.send({ type: "teleop", ns: state.target, vx, wz });
  },
  onState: (active) => {
    $("#btn-lease")?.classList.toggle("on", active);
  },
});

const link = new Link({
  onStatus: (connected) => {
    const pill = $("#pill-link");
    pill.classList.toggle("ok", connected);
    pill.classList.toggle("bad", !connected);
    pill.querySelector("span:last-child").textContent = connected ? "链路正常" : "链路断开";
    if (!connected) {
      state.leased = false;
      teleop.setActive(false);
    }
  },
  onSnapshot: (snapshot) => {
    state.snapshot = snapshot;
    applySnapshot(snapshot);
  },
  onEvent: (event) => appendEvent(event),
  onTelemetry: (message) => applyTelemetry(message),
  onTrail: (message) => mapView.setTrail(message.trail || {}),
  onRole: (role) => applyRole(role),
  onError: (text) => toast(text, "error"),
});

/* ------------------------------------------------------------------ */
/* snapshot                                                            */
/* ------------------------------------------------------------------ */
function applyTelemetry(message) {
  const series = message.coverage || [];
  const last = series.length ? series[series.length - 1] : null;
  const pct = last ? last[1] * 100 : 0;
  const area = last ? last[2] : 0;
  const bar = $("#cov-bar");
  if (bar) bar.style.width = `${Math.min(100, pct).toFixed(1)}%`;
  const pctEl = $("#cov-pct");
  if (pctEl) pctEl.textContent = last ? `${pct.toFixed(1)}%` : "--";
  const areaEl = $("#cov-area");
  const target = Number(message.target_area || 0);
  if (areaEl) {
    areaEl.textContent = last
      ? `已勘探 ${area.toFixed(0)}${target ? ` / ${target.toFixed(0)}` : ""} m²`
      : "等待地图";
  }
  const knownEl = $("#cov-known");
  if (knownEl) knownEl.textContent = `${series.length} 点`;
  mapView.setTrail(message.trail || {});
  drawCoverageChart(series);
}

function drawCoverageChart(series) {
  const canvas = $("#cov-chart");
  if (!canvas) return;
  const dpr = window.devicePixelRatio || 1;
  const width = canvas.clientWidth || 240;
  const height = canvas.clientHeight || 64;
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  if (series.length < 2) return;
  const values = series.map((row) => Number(row[1]) * 100);
  const max = Math.max(...values, 1);
  const min = Math.min(...values, 0);
  const span = Math.max(1e-6, max - min);
  ctx.beginPath();
  values.forEach((value, index) => {
    const x = (index / (values.length - 1)) * (width - 2) + 1;
    const y = height - 2 - ((value - min) / span) * (height - 6);
    if (index) ctx.lineTo(x, y);
    else ctx.moveTo(x, y);
  });
  ctx.strokeStyle = "#37c8d4";
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.lineTo(width - 1, height);
  ctx.lineTo(1, height);
  ctx.closePath();
  ctx.fillStyle = "rgba(55, 200, 212, 0.14)";
  ctx.fill();
}

let missionOptionLock = { id: "", until: 0 };

function syncMissionOptions(mission) {
  // While a click is in flight, do not let the 10 Hz snapshot paint the old
  // robot back over it.
  if (Date.now() < missionOptionLock.until) return;
  if (!mission) return;
  const box = $("#mission-robot");
  if (box && !box.querySelector("button")) ensureMissionPicker();
  if (box) {
    const robot = (mission && mission.robot) || "robot1";
    box.querySelectorAll("button").forEach((button) => {
      button.classList.toggle("active", button.dataset.ns === robot);
    });
  }
  const loop = $("#mission-loop");
  if (loop && document.activeElement !== loop) {
    loop.checked = Boolean(mission && mission.loop);
  }
}

function applySnapshot(snapshot) {
  updateFleet($("#robot-cards"), snapshot, activeMission());
  mapView.setSnapshot(snapshot);
  if (viewMode === "3d") cloudView.setPoses(snapshot.robots || {});

  const mission = displayMission();
  renderMission(mission, missionRefs);
  syncMissionOptions(mission);
  if (mission) mapView.setWaypoints(mission.waypoints || []);

  for (const ns of ["robot1", "robot2"]) {
    const robot = snapshot.robots?.[ns];
    const fallback = $(`#video-fallback-${ns}`);
    const image = $(`#video-${ns}`);
    if (!fallback || !image) continue;
    const alive = robot && robot.video;
    fallback.classList.toggle("hidden", Boolean(alive));
    if (!image.dataset.loaded) {
      image.dataset.loaded = "1";
      followCamera(ns, image);
    }
  }

  const empty = $("#map-empty");
  if (empty) empty.classList.toggle("hidden", Boolean(snapshot.map));
}

function setView(mode) {
  viewMode = mode;
  const wrap = $("#map-wrap");
  const three = mode === "3d";
  const world = mode === "world";
  const flat = !three && !world;
  wrap?.classList.toggle("show-3d", three);
  wrap?.classList.toggle("show-world", world);
  $("#cloud-canvas")?.classList.toggle("visible", three);
  $("#btn-view-2d")?.classList.toggle("active", flat);
  $("#btn-view-3d")?.classList.toggle("active", three);
  $("#btn-view-world")?.classList.toggle("active", world);
  $("#click-hint") && ($("#click-hint").style.display = flat ? "" : "none");
  $("#btn-click-mode") && ($("#btn-click-mode").style.display = flat ? "" : "none");
  if (three) {
    cloudView.resize();
    cloudView.visible = true;
  } else {
    cloudView.visible = false;
  }
  if (world) refreshWorldPreview();
}

/* ------------------------------------------------------------------ */
/* Gazebo world preview                                                */
/* ------------------------------------------------------------------ */
const worldState = {
  active: false,
  ns: "robot1",
  yaw: 0,
  pitch: -12,
  pos: [0, 0, 1.55],
};
let worldViewTimer = 0;
let worldViewQueued = false;

function applyWorldStatus(status) {
  if (!status) return;
  if (status.ns) worldState.ns = status.ns;
  if (status.yaw !== undefined) worldState.yaw = status.yaw;
  if (status.pitch !== undefined) worldState.pitch = status.pitch;
  if (Array.isArray(status.pos)) worldState.pos = status.pos.slice(0, 3);
}

async function refreshWorldPreview() {
  let status = null;
  try {
    status = await api("/api/world/preview/status");
  } catch {
    status = null;
  }
  const active = Boolean(status?.active && status?.bridge);
  if (active) {
    worldState.active = true;
    applyWorldStatus(status);
    const img = $("#world-stream");
    if (img && !img.dataset.streaming) {
      img.dataset.streaming = "1";
      img.src = `/api/world/preview.mjpeg?t=${Date.now()}`;
    }
  } else {
    worldState.active = false;
    const img = $("#world-stream");
    if (img) {
      img.dataset.streaming = "";
      img.removeAttribute("src");
    }
  }
  $("#world-empty")?.classList.toggle("hidden", active);
  return active;
}

async function startWorldPreview() {
  if (!requireOperator()) return;
  toast("正在启动世界预览…");
  const payload = await fetch("/api/world/preview/start", {
    method: "POST",
    credentials: "same-origin",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ ns: worldState.ns }),
  })
    .then((response) => response.json().then((body) => ({ ok: response.ok, body })))
    .catch((error) => ({ ok: false, body: { detail: error.message } }));
  if (!payload.ok || payload.body?.ok === false) {
    toast(payload.body?.detail || payload.body?.message || "启动失败", "error");
    return;
  }
  await refreshWorldPreview();
  toast("世界预览已开启", "ok");
}

function queueWorldView() {
  if (worldViewTimer) {
    worldViewQueued = true;
    return;
  }
  sendWorldView();
  worldViewTimer = window.setTimeout(() => {
    worldViewTimer = 0;
    if (worldViewQueued) {
      worldViewQueued = false;
      queueWorldView();
    }
  }, 40);
}

async function sendWorldView(extra = {}) {
  if (!worldState.active) return;
  try {
    const response = await fetch("/api/world/preview/view", {
      method: "POST",
      credentials: "same-origin",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        ns: worldState.ns,
        yaw: worldState.yaw,
        pitch: worldState.pitch,
        pos: worldState.pos,
        ...extra,
      }),
    });
    const status = await response.json();
    // The server clamps pitch/distance and recenters on ns; mirror it so the
    // next drag starts from what is actually on screen.
    applyWorldStatus(status);
  } catch {
    /* a dropped frame of drag is not worth a toast */
  }
}

function bindWorldPreview() {
  $("#btn-view-world")?.addEventListener("click", () => setView("world"));
  $("#btn-world-start")?.addEventListener("click", startWorldPreview);
  $("#btn-world-stop")?.addEventListener("click", async () => {
    if (!requireOperator()) return;
    await call("/api/world/preview/stop", {}, "世界预览已关闭");
    await refreshWorldPreview();
  });

  const recenter = async (ns) => {
    // Only ns/pitch/distance: sending the old pivot would undo the recentre.
    worldState.ns = ns;
    try {
      const response = await fetch("/api/world/preview/view", {
        method: "POST",
        credentials: "same-origin",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ ns, pitch: -12 }),
      });
      applyWorldStatus(await response.json());
    } catch {
      /* keep the current view */
    }
  };

  $("#btn-world-reset")?.addEventListener("click", () => {
    if (worldState.active) recenter(worldState.ns);
  });

  document.querySelectorAll("#world-target button").forEach((button) => {
    button.addEventListener("click", () => {
      if (!worldState.active) return;
      document
        .querySelectorAll("#world-target button")
        .forEach((other) => other.classList.toggle("active", other === button));
      recenter(button.dataset.ns);
    });
  });

  const img = $("#world-stream");
  let drag = null;
  img?.addEventListener("contextmenu", (event) => event.preventDefault());

  img?.addEventListener("pointerdown", (event) => {
    if (!worldState.active) return;
    event.preventDefault();
    img.setPointerCapture(event.pointerId);
    drag = {
      x: event.clientX,
      y: event.clientY,
      // Right button or Shift = pan, like the Gazebo GUI.
      pan: event.button === 2 || event.shiftKey,
    };
    img.classList.add("dragging");
  });

  img?.addEventListener("pointermove", (event) => {
    if (!drag) return;
    const dx = event.clientX - drag.x;
    const dy = event.clientY - drag.y;
    drag.x = event.clientX;
    drag.y = event.clientY;
    if (drag.pan) {
      // Grab-and-drag: the world follows the cursor, so the eye moves the
      // other way along its own right / up axes.
      const yaw = (worldState.yaw * Math.PI) / 180;
      const pitch = (worldState.pitch * Math.PI) / 180;
      const scale = 0.006;
      const rx = -Math.sin(yaw);
      const ry = Math.cos(yaw);
      const ux = -Math.cos(yaw) * Math.sin(pitch);
      const uy = -Math.sin(yaw) * Math.sin(pitch);
      const uz = Math.cos(pitch);
      worldState.pos = [
        worldState.pos[0] + (-dx * rx + dy * ux) * scale,
        worldState.pos[1] + (-dx * ry + dy * uy) * scale,
        Math.max(0.1, worldState.pos[2] + dy * uz * scale),
      ];
    } else {
      // Look around: dragging down tilts the view down, so the floor is one
      // drag away without having to climb over the ceiling.
      worldState.yaw = (worldState.yaw - dx * 0.3 + 360) % 360;
      worldState.pitch = Math.min(89, Math.max(-89, worldState.pitch - dy * 0.3));
    }
    queueWorldView();
  });

  const drop = () => {
    drag = null;
    img?.classList.remove("dragging");
  };
  window.addEventListener("pointerup", drop);
  window.addEventListener("pointercancel", drop);

  img?.addEventListener(
    "wheel",
    (event) => {
      if (!worldState.active) return;
      event.preventDefault();
      // Wheel = dolly along the view direction, like the Gazebo viewport.
      const yaw = (worldState.yaw * Math.PI) / 180;
      const pitch = (worldState.pitch * Math.PI) / 180;
      const step = -event.deltaY * 0.008;
      worldState.pos = [
        worldState.pos[0] + step * Math.cos(pitch) * Math.cos(yaw),
        worldState.pos[1] + step * Math.cos(pitch) * Math.sin(yaw),
        Math.max(0.1, worldState.pos[2] + step * Math.sin(pitch)),
      ];
      queueWorldView();
    },
    { passive: false }
  );
}

async function pollCloud() {
  if (viewMode === "3d" && cloudView.ok) {
    try {
      const response = await fetch("/api/cloud.bin", { cache: "no-store" });
      if (response.ok) {
        const seq = Number(response.headers.get("X-Cloud-Seq") || "0");
        const buffer = await response.arrayBuffer();
        cloudView.setBuffer(buffer, seq);
        cloudSeq = seq;
        $("#cloud-empty")?.classList.toggle("has-cloud", cloudView.count > 0);
      }
    } catch {
      /* keep the last cloud */
    }
  }
  window.setTimeout(pollCloud, 2000);
}

function followCamera(ns, image) {
  let objectUrl = "";
  const tick = async () => {
    try {
      const response = await fetch(`/api/video/${ns}/snapshot.jpg`, { cache: "no-store" });
      if (response.ok) {
        const blob = await response.blob();
        const next = URL.createObjectURL(blob);
        image.src = next;
        if (objectUrl) URL.revokeObjectURL(objectUrl);
        objectUrl = next;
      }
    } catch {
      /* keep the last frame and try again */
    }
    // 3 fps: each frame costs one HTTP round trip, and the public RTT is
    // ~200 ms.  Ten per second per robot kept all six browser connections busy.
    window.setTimeout(tick, 1000 / 3);
  };
  tick();
}

function activeMission() {
  return state.snapshot?.mission || null;
}

function displayMission() {
  const active = activeMission();
  const selected = selectedMissionId() || state.selectedId;
  if (active && active.id === selected) return active;
  if (
    active &&
    ["RUNNING", "PAUSED", "ESTOP", "ESTOPPED", "INTERRUPTED"].includes(active.state)
  ) {
    return active;
  }
  return state.missionDetail;
}

/* ------------------------------------------------------------------ */
/* events                                                              */
/* ------------------------------------------------------------------ */
function appendEvent(event) {
  state.events.unshift(event);
  state.events = state.events.slice(0, 60);
  const list = $("#event-list");
  if (!list) return;
  const item = document.createElement("li");
  item.className = event.level || "info";
  item.innerHTML = `<time>${new Date(event.ts * 1000).toLocaleTimeString("zh-CN", {
    hour12: false,
  })}</time><span><span class="tag">${event.code}</span> ${escapeHtml(
    event.text
  )}</span>`;
  list.prepend(item);
  while (list.children.length > 60) list.lastChild.remove();
  $("#event-count").textContent = String(state.events.length);
  $("#strip-event").textContent = event.text;
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

/* ------------------------------------------------------------------ */
/* role / auth                                                         */
/* ------------------------------------------------------------------ */
function applyRole(role) {
  state.role = role;
  const button = $("#btn-login");
  if (button) button.textContent = role === "operator" ? "退出登录" : "操作员登录";
  document.body.classList.toggle("is-operator", role === "operator");
}

function closeLoginDialog() {
  const dialog = $("#login-dialog");
  if (dialog?.open) dialog.close();
  const input = $("#login-token");
  if (input) input.value = "";
}

function requireOperator() {
  if (state.role === "operator") return true;
  toast("请先登录操作员", "error");
  $("#login-dialog")?.showModal();
  return false;
}

async function refreshSession() {
  try {
    const payload = await api("/api/session");
    applyRole(payload.role);
  } catch {
    applyRole("viewer");
  }
}

/* ------------------------------------------------------------------ */
/* missions                                                            */
/* ------------------------------------------------------------------ */
async function refreshMissions() {
  try {
    const payload = await api("/api/missions");
    state.missions = payload.missions || [];
    renderMissionSelect($("#mission-select"), state.missions, state.selectedId);
    // A stale id in localStorage (a mission deleted or renamed since) used to
    // leave missionDetail null for the whole session: the panel then showed no
    // mission and the robot selector snapped back to R1 on every snapshot.
    const known = state.missions.some((mission) => mission.id === state.selectedId);
    if (!known && state.missions.length) {
      state.selectedId = state.missions[0].id;
      $("#mission-select").value = state.selectedId;
      localStorage.setItem("mine_selected_mission", state.selectedId);
    }
    await loadMissionDetail(state.selectedId);
  } catch (error) {
    console.warn(error);
  }
}

async function loadMissionDetail(missionId) {
  if (!missionId) return;
  try {
    state.missionDetail = await api(`/api/missions/${missionId}`);
    renderMission(state.missionDetail, missionRefs);
    mapView.setWaypoints(state.missionDetail.waypoints || []);
  } catch (error) {
    console.warn(error);
  }
}

function selectedMissionId() {
  // The <select> can be empty for a moment (or permanently, if the list failed
  // to load) - fall back through the remembered id to the first mission
  // instead of silently doing nothing on every click.
  return (
    $("#mission-select")?.value ||
    state.selectedId ||
    state.missions?.[0]?.id ||
    ""
  );
}

/* ------------------------------------------------------------------ */
/* map click                                                           */
/* ------------------------------------------------------------------ */
async function handleMapPick(point) {
  if (!requireOperator()) return;
  if (state.clickMode === "wp") {
    const missionId = selectedMissionId();
    if (!missionId) {
      toast("请先选择任务", "error");
      return;
    }
    try {
      const mission = await api(`/api/missions/${missionId}/waypoints`, {
        method: "POST",
        body: {
          label: `巡检点 ${(state.missionDetail?.waypoints?.length || 0) + 1}`,
          x: Number(point.x.toFixed(3)),
          y: Number(point.y.toFixed(3)),
          yaw: 0,
          dwell: 5,
        },
      });
      state.missionDetail = mission.mission;
      renderMission(state.missionDetail, missionRefs);
      mapView.setWaypoints(state.missionDetail.waypoints);
      toast("已追加巡检点", "ok");
    } catch (error) {
      toast(error.message, "error");
    }
    return;
  }

  try {
    await api(`/api/robot/${state.target}/goal`, {
      method: "POST",
      body: { x: point.x, y: point.y, yaw: 0 },
    });
    toast(`${state.target.toUpperCase()} 目标已下发`, "ok");
  } catch (error) {
    toast(error.message, "error");
  }
}

/* ------------------------------------------------------------------ */
/* actions                                                             */
/* ------------------------------------------------------------------ */
async function call(path, body, okText) {
  if (!requireOperator()) return null;
  try {
    const payload = await api(path, { method: "POST", body });
    if (okText) toast(okText, "ok");
    return payload;
  } catch (error) {
    toast(error.message, "error");
    return null;
  }
}

const PICKER_HTML =
  '<div class="segmented" id="mission-robot" title="执行巡检的车">' +
  '<button data-ns="robot1" class="active">R1</button>' +
  '<button data-ns="robot2">R2</button></div>' +
  '<label class="check-row tight" for="mission-loop">' +
  '<input type="checkbox" id="mission-loop" /><span>循环执行</span></label>';

function ensureMissionPicker() {
  // The picker must own the id: mission.js used to write the robot name into
  // #mission-robot, which (with a duplicate id on the progress line) wiped the
  // buttons out of the DOM on the next snapshot - the panel looked fine and
  // clicked nowhere.  Rebuild whenever the row exists but has no buttons.
  const existing = $("#mission-robot");
  if (existing && existing.querySelector("button")) return;
  const select = $("#mission-select");
  if (!select) return;
  if (existing) {
    existing.parentElement.classList.add("mission-opts");
    existing.parentElement.innerHTML = PICKER_HTML;
    return;
  }
  const row = document.createElement("div");
  row.className = "mission-opts";
  row.innerHTML = PICKER_HTML;
  select.insertAdjacentElement("afterend", row);
}

function wireActions() {
  ensureMissionPicker();
  $("#btn-login")?.addEventListener("click", async () => {
    if (state.role === "operator") {
      await api("/api/logout", { method: "POST" }).catch(() => {});
      localStorage.removeItem("console_token");
      applyRole("viewer");
      link.reconnect();
      toast("已退出登录");
      return;
    }
    $("#login-dialog").showModal();
    $("#login-token").focus();
  });

  $("#login-form")?.addEventListener("submit", async (event) => {
    event.preventDefault();
    // Both buttons are submit buttons (method="dialog"), so 取消 also lands
    // here; without this branch it would try to log in with an empty token and
    // leave the dialog open.
    if (event.submitter?.value === "cancel") {
      closeLoginDialog();
      return;
    }
    const token = $("#login-token").value.trim();
    try {
      await api("/api/login", { method: "POST", body: { token } });
      localStorage.setItem("console_token", token);
      applyRole("operator");
      $("#login-dialog").close();
      $("#login-token").value = "";
      link.reconnect();
      toast("操作员已登录", "ok");
    } catch (error) {
      toast(error.message, "error");
    }
  });

  $("#mission-select")?.addEventListener("change", (event) => {
    state.selectedId = event.target.value;
    localStorage.setItem("mine_selected_mission", state.selectedId);
    loadMissionDetail(state.selectedId);
  });

  document.querySelectorAll("#mission-robot button").forEach((button) => {
    button.addEventListener("click", async () => {
      const id = selectedMissionId();
      if (!id) {
        toast("请先选择任务", "error");
        return;
      }
      const ns = button.dataset.ns;
      // Paint immediately: the snapshot loop runs at 10 Hz and would otherwise
      // repaint the previous robot before the POST returns.
      document.querySelectorAll("#mission-robot button").forEach((other) => {
        other.classList.toggle("active", other === button);
      });
      missionOptionLock = { id, until: Date.now() + 2500 };
      const payload = await call(
        `/api/missions/${id}/options`,
        { robot: ns },
        `执行车已切换为 ${ns.toUpperCase()}`
      );
      if (payload?.mission) {
        if (state.missionDetail?.id === id) state.missionDetail.robot = payload.mission.robot;
        if (state.snapshot?.mission?.id === id) state.snapshot.mission.robot = payload.mission.robot;
        await loadMissionDetail(id);
      } else {
        missionOptionLock.until = 0;
        syncMissionOptions(displayMission());
      }
    });
  });

  $("#mission-loop")?.addEventListener("change", async (event) => {
    const id = selectedMissionId();
    if (!id) {
      toast("请先选择任务", "error");
      return;
    }
    const enabled = event.target.checked;
    missionOptionLock = { id, until: Date.now() + 2500 };
    const payload = await call(
      `/api/missions/${id}/options`,
      { loop: enabled },
      enabled ? "已开启循环执行" : "已关闭循环执行"
    );
    if (!payload) {
      missionOptionLock.until = 0;
      event.target.checked = !enabled;
    } else if (payload.mission && state.missionDetail?.id === id) {
      state.missionDetail.loop = payload.mission.loop;
    }
  });

  $("#btn-mission-start")?.addEventListener("click", async () => {
    const id = selectedMissionId();
    if (!id) return;
    await call(`/api/missions/${id}/start`, undefined, "任务已开始");
    await refreshMissions();
  });
  $("#btn-mission-pause")?.addEventListener("click", () =>
    call(`/api/missions/${selectedMissionId()}/pause`, undefined, "任务已暂停")
  );
  $("#btn-mission-resume")?.addEventListener("click", () =>
    call(`/api/missions/${selectedMissionId()}/resume`, undefined, "任务继续")
  );
  $("#btn-mission-skip")?.addEventListener("click", () =>
    call(`/api/missions/${selectedMissionId()}/skip`, undefined, "已跳过当前点")
  );
  $("#btn-mission-stop")?.addEventListener("click", () =>
    call(`/api/missions/${selectedMissionId()}/stop`, undefined, "任务已终止")
  );

  $("#btn-click-mode")?.addEventListener("click", () => {
    state.clickMode = state.clickMode === "nav" ? "wp" : "nav";
    $("#click-hint").textContent =
      state.clickMode === "nav" ? "点击地图：单点导航" : "点击地图：追加巡检点";
    $("#btn-click-mode").textContent =
      state.clickMode === "nav" ? "追加航点" : "单点导航";
    $("#btn-click-mode").classList.toggle("on", state.clickMode === "wp");
  });

  $("#btn-fit")?.addEventListener("click", () => {
    if (viewMode === "3d") {
      cloudView.fit();
      return;
    }
    mapView.fitted = false;
    mapView.fit();
  });

  $("#btn-view-2d")?.addEventListener("click", () => setView("2d"));
  $("#btn-view-3d")?.addEventListener("click", () => setView("3d"));
  pollCloud();

  document.querySelectorAll("#teleop-target button").forEach((button) => {
    button.addEventListener("click", async () => {
      const ns = button.dataset.ns;
      if (ns === state.target) return;
      if (state.leased) {
        await api("/api/teleop/release", { method: "POST" }).catch(() => {});
        state.leased = false;
        teleop.setActive(false);
      }
      state.target = ns;
      document
        .querySelectorAll("#teleop-target button")
        .forEach((item) => item.classList.toggle("active", item === button));
    });
  });

  $("#btn-lease")?.addEventListener("click", async () => {
    if (!requireOperator()) return;
    if (state.leased) {
      toast("已处于接管状态");
      return;
    }
    try {
      await api("/api/teleop/lease", {
        method: "POST",
        body: { ns: state.target },
      });
      state.leased = true;
      teleop.setActive(true);
      toast(`${state.target.toUpperCase()} 已接管`, "ok");
    } catch (error) {
      toast(error.message, "error");
    }
  });

  $("#btn-release")?.addEventListener("click", async () => {
    await api("/api/teleop/release", { method: "POST" }).catch(() => {});
    state.leased = false;
    teleop.setActive(false);
    toast("已释放控制权");
  });

  $("#btn-estop-2")?.addEventListener("click", triggerEstop);
  $("#btn-clear-estop")?.addEventListener("click", async () => {
    await call("/api/estop/clear", undefined, "急停已解除，自主勘探需重新启动");
  });

  $("#btn-explore")?.addEventListener("click", async () => {
    const payload = await call("/api/exploration/start", undefined, "自主勘探已启动");
    if (payload) toast(payload.message || "自主勘探已启动", payload.ok ? "ok" : "error");
  });
  $("#btn-stop-explore")?.addEventListener("click", async () => {
    await call("/api/exploration/stop", undefined, "自主勘探已停止");
  });

  $("#btn-recall-1")?.addEventListener("click", () =>
    call("/api/robot/robot1/recall", undefined, "ROBOT 1 召回中")
  );
  $("#btn-recall-2")?.addEventListener("click", () =>
    call("/api/robot/robot2/recall", undefined, "ROBOT 2 召回中")
  );

  $("#btn-save-map")?.addEventListener("click", async () => {
    if (!requireOperator()) return;
    toast("正在保存地图…");
    const payload = await call("/api/map/save");
    if (payload) toast(`已保存 ${payload.path}`, "ok");
  });

  $("#btn-save-cloud")?.addEventListener("click", async () => {
    if (!requireOperator()) return;
    toast("正在导出点云…");
    const payload = await call("/api/cloud/save", { binary: true });
    if (!payload?.ok) return;
    // The export lands on the machine running the console; pull it down so the
    // operator ends up with the file instead of just a path on the server.
    const name = String(payload.path).split("/").pop();
    try {
      await downloadFile(`/api/cloud/file/${encodeURIComponent(name)}`, name);
      toast(`已下载 ${name}（${payload.points.toLocaleString()} 点）`, "ok");
    } catch (error) {
      toast(`导出成功但下载失败：${error.message}`, "error");
    }
  });

  $("#btn-restart-sim")?.addEventListener("click", async () => {
    if (!requireOperator()) return;
    const world = $("#select-world")?.value || "mine";
    const clean = $("#chk-clean")?.checked ?? true;
    const label = $("#select-world")?.selectedOptions?.[0]?.textContent || world;
    if (
      !window.confirm(
        `重启仿真到「${label}」？\n${clean ? "重启前会清除旧缓存（地图、点云、残留 DDS 段）。" : ""}`
      )
    )
      return;
    toast("正在重启仿真（约 60–120 秒）…");
    const payload = await call("/api/sim/restart", {
      world,
      mode: "off",
      headless: true,
      clean,
    });
    if (payload?.ok) {
      const removed = (payload.cleaned || []).length;
      toast(removed ? `仿真已重启，清理 ${removed} 项旧缓存` : "仿真已重启", "ok");
    } else if (payload) {
      toast(payload.message || "重启失败", "error");
    }
    pollSim();
  });

  $("#btn-stop-sim")?.addEventListener("click", async () => {
    if (!requireOperator()) return;
    if (!window.confirm("确定停止仿真？")) return;
    await call("/api/sim/stop", undefined, "仿真已停止");
    pollSim();
  });

  $("#btn-scan-toggle")?.addEventListener("click", (event) => {
    mapView.showScan = !mapView.showScan;
    mapView.dirty = true;
    const button = event.currentTarget;
    button.classList.toggle("on", mapView.showScan);
    button.setAttribute("aria-pressed", mapView.showScan ? "true" : "false");
  });

  window.addEventListener("keydown", (event) => {
    if (event.code === "Escape" && state.leased) {
      api("/api/teleop/release", { method: "POST" }).catch(() => {});
      state.leased = false;
      teleop.setActive(false);
    }
  });
}

async function triggerEstop() {
  if (!requireOperator()) return;
  teleop.setActive(false);
  state.leased = false;
  const ok = await api("/api/estop", { method: "POST" }).catch((error) => {
    toast(error.message, "error");
    return null;
  });
  if (ok) toast("急停已触发，勘探已暂停", "error");
}

/* ------------------------------------------------------------------ */
/* sim polling                                                         */
/* ------------------------------------------------------------------ */
async function downloadFile(url, filename) {
  const response = await fetch(url, { credentials: "same-origin" });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  const blob = await response.blob();
  const href = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = href;
  link.download = filename;
  document.body.appendChild(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(href), 10000);
}

async function loadWorlds() {
  try {
    const payload = await api("/api/sim/worlds");
    const select = $("#select-world");
    if (!select) return;
    select.innerHTML = (payload.worlds || [])
      .map((world) => `<option value="${world.id}">${world.label}</option>`)
      .join("");
    const active = payload.active || payload.default;
    if (active) select.value = active;
  } catch {
    /* console offline — keep the list empty */
  }
}

async function pollSim() {
  try {
    const payload = await api("/api/sim/status");
    state.sim = payload;
    const sim = payload.sim || {};
    const exploration = payload.exploration || {};
    $("#sim-state").textContent = sim.running
      ? `仿真：运行 ${Math.round(sim.uptime || 0)}s${sim.world ? " · " + sim.world : ""}`
      : "仿真：未受控";
    const worldPill = $("#pill-world");
    if (worldPill) {
      worldPill.querySelector("b").textContent = sim.running && sim.world ? sim.world : "--";
    }
    const select = $("#select-world");
    if (sim.running && sim.world && select && select.value !== sim.world) {
      select.value = sim.world;
    }
    $("#pill-mode").querySelector("b").textContent = exploration.running
      ? "勘探"
      : "巡检";
  } catch {
    /* ignore */
  }
}

/* ------------------------------------------------------------------ */
/* boot                                                                */
/* ------------------------------------------------------------------ */
async function boot() {
  wireActions();
  bindWorldPreview();
  await refreshSession();
  await loadWorlds();
  await refreshMissions();
  link.connect();
  // The WebSocket pushes telemetry every second; this covers a reload before
  // the first push arrives.
  api("/api/telemetry").then(applyTelemetry).catch(() => {});
  pollSim();
  setInterval(pollSim, 5000);
  setInterval(refreshMissions, 20000);
  // Cheap: one status GET every 5 s, and only while the tab is open.
  setInterval(() => {
    if (viewMode === "world") refreshWorldPreview();
  }, 5000);
}

boot();
