import { $, api, toast } from "./utils.js";
import { Link } from "./ws.js";
import { MapView } from "./map2d.js?v=4";
import { CloudView } from "./cloud3d.js?v=7";
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
  robot: $("#mission-robot"),
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
    const element = $("#teleop-state");
    if (element) {
      element.textContent = active
        ? `已接管 ${state.target.toUpperCase()} · WASD / 摇杆`
        : "未接管 · WASD / 摇杆";
    }
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
  onRole: (role) => applyRole(role),
  onError: (text) => toast(text, "error"),
});

/* ------------------------------------------------------------------ */
/* snapshot                                                            */
/* ------------------------------------------------------------------ */
function applySnapshot(snapshot) {
  updatePills(snapshot);
  updateFleet($("#robot-cards"), snapshot, activeMission());
  mapView.setSnapshot(snapshot);
  if (viewMode === "3d") cloudView.setPoses(snapshot.robots || {});

  const mission = displayMission();
  renderMission(mission, missionRefs);
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
  wrap?.classList.toggle("show-3d", three);
  $("#cloud-canvas")?.classList.toggle("visible", three);
  $("#btn-view-2d")?.classList.toggle("active", !three);
  $("#btn-view-3d")?.classList.toggle("active", three);
  $("#click-hint") && ($("#click-hint").style.display = three ? "none" : "");
  $("#btn-click-mode") && ($("#btn-click-mode").style.display = three ? "none" : "");
  if (three) {
    cloudView.resize();
    cloudView.visible = true;
  } else {
    cloudView.visible = false;
  }
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

function updatePills(snapshot) {
  const loops = snapshot.loop_closures || {};
  const loopTotal = Object.values(loops).reduce((sum, value) => sum + Number(value || 0), 0);
  const loopEl = $("#loop-status");
  if (loopEl) {
    const parts = Object.entries(loops).map(([ns, count]) => `${ns.slice(-1)}:${count}`);
    loopEl.textContent = loopTotal
      ? `闭环 ${loopTotal}（${parts.join(" ")}）`
      : "闭环 0";
  }

  const clockPill = $("#pill-clock").querySelector("b");
  const clock = snapshot.sim?.clock;
  clockPill.textContent =
    snapshot.sim?.stale || clock === null || clock === undefined
      ? "暂停"
      : `T+${Math.floor(clock)}s`;

  const mapPill = $("#pill-map").querySelector("b");
  const meta = snapshot.map;
  mapPill.textContent = meta
    ? `${meta.width}×${meta.height} @ ${Number(meta.resolution).toFixed(2)}m`
    : "--";
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
    if (!state.selectedId && state.missions.length) {
      state.selectedId = state.missions[0].id;
      $("#mission-select").value = state.selectedId;
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
  return $("#mission-select")?.value || state.selectedId;
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

function wireActions() {
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

  $("#btn-estop")?.addEventListener("click", triggerEstop);
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
  await refreshSession();
  await loadWorlds();
  await refreshMissions();
  link.connect();
  pollSim();
  setInterval(pollSim, 5000);
  setInterval(refreshMissions, 20000);
}

boot();
