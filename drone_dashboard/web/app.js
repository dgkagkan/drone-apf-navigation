"use strict";

const colors = ["#ff6572", "#57df8c", "#62a9ff", "#ffbd52", "#bb77ff", "#49d8d0"];
const canvas = document.getElementById("map");
const context = canvas.getContext("2d");
let state = null;
let view = { x: 0, y: 0, metersPerPixel: 2.2 };
let dragging = false;
let moved = false;
let dragStart = null;
let cameraIds = [];
let droneInteractionUntil = 0;
const speedDrafts = new Map();
const lidarRangeDrafts = new Map();

function droneColor(id) {
  let hash = 2166136261;
  for (const character of id) hash = Math.imul(hash ^ character.charCodeAt(0), 16777619);
  return colors[Math.abs(hash) % colors.length];
}

function batteryPresentation(drone) {
  const labels = ["UNKNOWN", "NORMAL", "LOW", "RETURNING HOME", "EMERGENCY LAND"];
  const stateName = labels[drone.battery_state] || "UNKNOWN";
  const percentage = drone.battery_valid && drone.battery_remaining_pct != null ?
    Math.max(0, Math.min(100, drone.battery_remaining_pct)) : null;
  const time = drone.battery_time_remaining_s;
  const timeText = time == null || time < 0 ? "" : ` · ${Math.ceil(time / 60)} min remaining`;
  const powerText = drone.battery_power_w == null ? "" : ` · ${drone.battery_power_w.toFixed(0)} W`;
  const energyText = drone.battery_remaining_energy_wh == null ? "" :
    ` · ${drone.battery_remaining_energy_wh.toFixed(0)} Wh`;
  const tone = drone.battery_state >= 4 ? "emergency" :
    (drone.battery_state >= 2 ? "warning" : "normal");
  return {
    label: percentage == null ? `BATTERY ${stateName}` :
      `BATTERY ${percentage.toFixed(0)}% · ${stateName}${timeText}${powerText}${energyText}`,
    percentage: percentage == null ? 0 : percentage,
    tone,
    valid: percentage != null,
  };
}

async function api(path, payload) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const result = await response.json();
  toast(result.message || (result.ok ? "Command accepted" : "Command rejected"), !result.ok);
  if (!response.ok) throw new Error(result.message);
  return result;
}

function toast(message, error = false) {
  const element = document.getElementById("toast");
  element.textContent = message;
  element.className = `toast show${error ? " error" : ""}`;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => element.className = "toast", 3600);
}

async function refresh() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    state = await response.json();
    render();
  } catch (error) {
    setConnection(false, "Dashboard offline");
  }
}

function setConnection(online, text) {
  const box = document.querySelector(".connection");
  box.className = `connection ${online ? "online" : "offline"}`;
  document.getElementById("connection-text").textContent = text;
}

function render() {
  const age = Date.now() / 1000 - state.updated_at;
  setConnection(state.connected && age < 3, state.connected ? "Coordinator online" : "Waiting for ROS");
  document.getElementById("status-message").textContent = state.status_message;
  const startMission = document.getElementById("start-mission");
  startMission.disabled = !state.plan_ready || state.dispatch_in_progress;
  startMission.textContent = state.plan_is_recalculation ? "START RECALCULATED MISSION" : "START MISSION";
  renderMetrics();
  renderTargets();
  renderDrones();
  renderCameras();
  drawMap();
}

function renderMetrics() {
  const connected = state.drones.filter(drone => drone.connected).length;
  const busy = state.drones.filter(drone => drone.busy).length;
  const values = [
    [connected, "Connected drones"],
    [state.available_drone_count, "Available"],
    [busy, "Busy"],
    [state.pending_target_count, "Pending targets"],
    [state.active_target_count, "Active targets"],
    [state.planned_drone_count || 0, "Planned drones"],
  ];
  document.getElementById("metrics").innerHTML = values.map(([value, label]) =>
    `<div class="metric"><span>${label}</span><b>${value}</b></div>`
  ).join("");
}

function renderTargets() {
  const list = document.getElementById("target-list");
  if (!state.pending_targets.length) {
    list.className = "target-list empty";
    list.textContent = "No pending targets";
    return;
  }
  list.className = "target-list";
  list.innerHTML = state.pending_targets.map(target => {
    const p = target.position;
    return `<div class="target-item">
      <div class="target-id">${target.target_id}</div>
      <div><b>${p.x.toFixed(1)}, ${p.y.toFixed(1)}, ${p.z.toFixed(1)}</b>
      <small>${target.cruise_speed_m_s.toFixed(1)} m/s · ${target.use_fixed_wing ? "fixed wing" : "multicopter"}</small></div>
      <button class="delete-target" data-remove-target="${target.target_id}" title="Remove">×</button>
    </div>`;
  }).join("");
}

function renderDrones() {
  const grid = document.getElementById("drone-grid");
  if (!state.drones.length) {
    grid.innerHTML = `<div class="camera-empty">Waiting for drones to join the swarm</div>`;
    return;
  }
  const activeEditor = document.activeElement;
  if (Date.now() < droneInteractionUntil ||
      (grid.contains(activeEditor) && activeEditor.matches("input"))) return;
  grid.innerHTML = state.drones.map(drone => {
    const p = drone.position;
    const telemetry = state.telemetry[drone.drone_id];
    const speed = state.motion[drone.drone_id]?.speed_m_s;
    const obstacle = telemetry?.nearest_obstacle_m;
    const enabled = drone.operator_enabled;
    const routingEnabled = enabled && !drone.safety_excluded;
    const battery = batteryPresentation(drone);
    const plannedRoute = (state.planned_assignments || []).filter(
      assignment => assignment.drone_id === drone.drone_id);
    const activeRoute = (state.assignments || []).filter(
      assignment => assignment.drone_id === drone.drone_id);
    const displayedRoute = plannedRoute.length ? plannedRoute : activeRoute;
    const routeKind = plannedRoute.length ? "PREVIEW" : "ACTIVE ROUTE";
    const routeCost = displayedRoute[0]?.route_total_cost;
    const speedCommand = speedDrafts.get(drone.drone_id) ??
      (drone.has_speed_override ? drone.speed_override_m_s.toFixed(1) : "15.0");
    const lidarRange = lidarRangeDrafts.get(drone.drone_id) ?? drone.lidar_range_m.toFixed(0);
    const controlsDisabled = drone.connected && routingEnabled ? "" : "disabled";
    const badgeText = drone.safety_excluded ?
      (drone.return_home_active ? "BATTERY RTH" : "SAFETY EXCLUDED") :
      (!enabled ? "OFF / EXCLUDED" : (drone.connected ? "connected" : "offline"));
    const safetyActive = drone.safety_excluded && drone.battery_state >= 3;
    const membershipDisabled = safetyActive ? "disabled" : "";
    const membershipCommand = !enabled || drone.safety_excluded ? "rejoin" : "off";
    const membershipLabel = safetyActive ? "SAFETY LOCK" :
      ((!enabled || drone.safety_excluded) ? "REJOIN" : "OFF");
    return `<article class="drone-card ${routingEnabled ? "" : "excluded"}" style="border-top:2px solid ${droneColor(drone.drone_id)}">
      <div class="drone-title"><h3>${drone.drone_id}</h3><span class="badge ${drone.connected && routingEnabled ? "online" : ""}">${badgeText}</span></div>
      <div class="drone-position"><div><span>X</span>${p.x.toFixed(1)}</div><div><span>Y</span>${p.y.toFixed(1)}</div><div><span>Z</span>${p.z.toFixed(1)}</div></div>
      <div class="speed-line"><span>SPEED</span><b>${speed == null ? "—" : speed.toFixed(1)} m/s</b></div>
      ${displayedRoute.length ? `<div class="route-summary"><span>${routeKind}</span><b>${displayedRoute.length} targets · ${routeCost == null ? "—" : routeCost.toFixed(1) + " s"}</b></div>` : ""}
      <div class="battery-line ${battery.tone}">
        <div><span>${battery.label}</span></div>
        <div class="battery-track"><i style="width:${battery.valid ? battery.percentage : 0}%"></i></div>
      </div>
      <div class="speed-control">
        <label>COMMAND<input type="number" min="10" max="20" step="0.5" value="${speedCommand}" data-drone-speed="${drone.drone_id}" ${controlsDisabled}></label>
        <button class="button primary" data-set-drone-speed="${drone.drone_id}" ${controlsDisabled}>SET</button>
        <button class="button ghost" data-clear-drone-speed="${drone.drone_id}" ${drone.has_speed_override && controlsDisabled === "" ? "" : "disabled"}>AUTO</button>
      </div>
      <div class="speed-source">${drone.has_speed_override ? `Override ${drone.speed_override_m_s.toFixed(1)} m/s` : "Using route speed · default 15.0 m/s"}</div>
      <div class="lidar-control">
        <label>ACTIVE LiDAR / APF RANGE <input type="range" min="70" max="300" step="10" value="${lidarRange}" data-drone-lidar-range="${drone.drone_id}" ${controlsDisabled}></label>
        <output data-lidar-range-output="${drone.drone_id}">${lidarRange} m</output>
        <button class="button primary" data-set-drone-lidar-range="${drone.drone_id}" ${controlsDisabled}>SET</button>
      </div>
      <div class="drone-flags">
        <span class="flag ${drone.armed ? "on" : ""}">${drone.armed ? "ARMED" : "DISARMED"}</span>
        <span class="flag ${drone.offboard ? "on" : ""}">${drone.offboard ? "OFFBOARD" : "MANUAL"}</span>
        <span class="flag ${drone.localized ? "on" : ""}">LOCALIZED</span>
        <span class="flag ${drone.available ? "on" : ""}">${!routingEnabled ? "EXCLUDED" : (drone.busy ? "BUSY" : "AVAILABLE")}</span>
      </div>
      <div class="apf-line">APF ${telemetry?.active_mode || "—"} · obstacle ${obstacle == null ? "—" : obstacle.toFixed(1) + " m"}</div>
      <div class="drone-actions">
        <button class="button arm" data-drone-arm="${drone.drone_id}" ${controlsDisabled}>ARM</button>
        <button class="button primary" data-drone-takeoff="${drone.drone_id}" ${controlsDisabled}>TAKEOFF</button>
        <button class="button" data-drone-home="${drone.drone_id}" ${controlsDisabled}>HOME</button>
        <button class="button danger" data-drone-land="${drone.drone_id}" ${controlsDisabled}>LAND</button>
        <button class="button ${enabled ? "danger ghost" : "arm"}" data-drone-membership="${drone.drone_id}" data-membership-command="${membershipCommand}" ${membershipDisabled}>${membershipLabel}</button>
      </div>
    </article>`;
  }).join("");
}

function renderCameras() {
  const ids = state.drones.filter(drone => drone.connected).map(drone => drone.drone_id);
  if (JSON.stringify(ids) === JSON.stringify(cameraIds)) return;
  cameraIds = ids;
  const grid = document.getElementById("camera-grid");
  grid.innerHTML = ids.length ? ids.map(id =>
    `<div class="camera"><img id="camera-${id}" alt="${id} camera"><span class="camera-label">${id}</span></div>`
  ).join("") : `<div class="camera-empty">Camera streams appear when drones connect</div>`;
}

function refreshCameras() {
  const stamp = Date.now();
  cameraIds.forEach(id => {
    const image = document.getElementById(`camera-${id}`);
    if (image && state.camera_drones.includes(id)) {
      image.src = `/api/camera/${encodeURIComponent(id)}.jpg?t=${stamp}`;
    }
  });
}

function resizeCanvas() {
  const ratio = window.devicePixelRatio || 1;
  const bounds = canvas.getBoundingClientRect();
  canvas.width = Math.round(bounds.width * ratio);
  canvas.height = Math.round(bounds.height * ratio);
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  drawMap();
}

function screenToWorld(clientX, clientY) {
  const bounds = canvas.getBoundingClientRect();
  const x = clientX - bounds.left;
  const y = clientY - bounds.top;
  return {
    x: view.x + (x - bounds.width / 2) * view.metersPerPixel,
    y: view.y - (y - bounds.height / 2) * view.metersPerPixel,
  };
}

function worldToScreen(point) {
  const bounds = canvas.getBoundingClientRect();
  return {
    x: bounds.width / 2 + (point.x - view.x) / view.metersPerPixel,
    y: bounds.height / 2 - (point.y - view.y) / view.metersPerPixel,
  };
}

function drawMap() {
  if (!state) return;
  const bounds = canvas.getBoundingClientRect();
  context.clearRect(0, 0, bounds.width, bounds.height);
  context.fillStyle = "#081219";
  context.fillRect(0, 0, bounds.width, bounds.height);
  drawGrid(bounds);
  Object.entries(state.paths).forEach(([id, paths]) => {
    drawPath(paths.flown || [], droneColor(id), 2.1, .8);
    drawPath(paths.nominal || [], droneColor(id), 1.1, .45, [7, 6]);
  });
  drawAssignments(state.assignments, false);
  drawAssignments(state.planned_assignments || [], true);
  state.pending_targets.forEach(target => drawTarget(target.position, target.target_id, "#ffbd52"));
  state.drones.forEach(drawDrone);
}

function drawGrid(bounds) {
  const preferred = view.metersPerPixel * 90;
  const power = 10 ** Math.floor(Math.log10(preferred));
  const grid = [1, 2, 5, 10].find(value => value * power >= preferred) * power;
  const left = view.x - bounds.width / 2 * view.metersPerPixel;
  const right = view.x + bounds.width / 2 * view.metersPerPixel;
  const bottom = view.y - bounds.height / 2 * view.metersPerPixel;
  const top = view.y + bounds.height / 2 * view.metersPerPixel;
  context.lineWidth = 1;
  context.font = "10px sans-serif";
  context.fillStyle = "#536873";
  for (let x = Math.ceil(left / grid) * grid; x <= right; x += grid) {
    const point = worldToScreen({ x, y: 0 });
    context.strokeStyle = Math.abs(x) < .001 ? "#3b6872" : "#182a33";
    context.beginPath(); context.moveTo(point.x, 0); context.lineTo(point.x, bounds.height); context.stroke();
    context.fillText(`${x}m`, point.x + 4, 13);
  }
  for (let y = Math.ceil(bottom / grid) * grid; y <= top; y += grid) {
    const point = worldToScreen({ x: 0, y });
    context.strokeStyle = Math.abs(y) < .001 ? "#3b6872" : "#182a33";
    context.beginPath(); context.moveTo(0, point.y); context.lineTo(bounds.width, point.y); context.stroke();
    context.fillText(`${y}m`, 4, point.y - 4);
  }
}

function drawPath(points, color, width, alpha, dash = []) {
  if (points.length < 2) return;
  context.save(); context.strokeStyle = color; context.globalAlpha = alpha; context.lineWidth = width; context.setLineDash(dash);
  context.beginPath();
  points.forEach((point, index) => { const p = worldToScreen(point); index ? context.lineTo(p.x, p.y) : context.moveTo(p.x, p.y); });
  context.stroke(); context.restore();
}

function drawAssignments(assignments, preview) {
  const grouped = {};
  assignments.forEach(item => (grouped[item.drone_id] ||= []).push(item));
  Object.entries(grouped).forEach(([id, items]) => {
    const drone = state.drones.find(candidate => candidate.drone_id === id);
    if (!drone) return;
    drawPath(
      [drone.position, ...items.map(item => item.position)],
      droneColor(id), preview ? 3.0 : 1.5, preview ? 1.0 : .7,
      preview ? [10, 5] : [4, 5]);
    items.forEach((item, index) =>
      drawTarget(item.position, index + 1, droneColor(id), preview));
  });
}

function drawTarget(point, label, color, preview = false) {
  const p = worldToScreen(point);
  context.save(); context.fillStyle = preview ? "#081219" : color; context.strokeStyle = color; context.lineWidth = preview ? 3 : 2;
  context.beginPath(); context.arc(p.x, p.y, 8, 0, Math.PI * 2); context.fill(); context.stroke();
  context.fillStyle = preview ? color : "#071116"; context.font = "bold 9px sans-serif"; context.textAlign = "center"; context.textBaseline = "middle"; context.fillText(label, p.x, p.y + .5); context.restore();
}

function drawDrone(drone) {
  const p = worldToScreen(drone.position);
  const color = drone.connected && drone.operator_enabled && !drone.safety_excluded ?
    droneColor(drone.drone_id) : "#59636a";
  context.save(); context.translate(p.x, p.y); context.fillStyle = color; context.strokeStyle = "#071014"; context.lineWidth = 2;
  context.beginPath(); context.moveTo(0, -11); context.lineTo(9, 9); context.lineTo(0, 5); context.lineTo(-9, 9); context.closePath(); context.fill(); context.stroke(); context.restore();
  const speed = state.motion[drone.drone_id]?.speed_m_s;
  const speedLabel = speed == null ? "—" : `${speed.toFixed(1)}m/s`;
  context.fillStyle = color; context.font = "bold 10px sans-serif"; context.textAlign = "left"; context.fillText(`${drone.drone_id}  ${drone.position.z.toFixed(1)}m  ${speedLabel}`, p.x + 12, p.y - 8);
  const telemetry = state.telemetry[drone.drone_id];
  if (telemetry) {
    drawVector(drone.position, telemetry.attractive, "#57df8c");
    drawVector(drone.position, telemetry.repulsive, "#ff6572");
    drawVector(drone.position, telemetry.safe_command, "#62a9ff");
  }
}

function drawVector(origin, vector, color) {
  const magnitude = Math.hypot(vector.x, vector.y);
  if (magnitude < .001) return;
  const scale = Math.min(80, Math.max(16, magnitude * 12));
  const start = worldToScreen(origin);
  const dx = vector.x / magnitude * scale;
  const dy = -vector.y / magnitude * scale;
  context.save(); context.strokeStyle = color; context.fillStyle = color; context.lineWidth = 2;
  context.beginPath(); context.moveTo(start.x, start.y); context.lineTo(start.x + dx, start.y + dy); context.stroke();
  const angle = Math.atan2(dy, dx); context.translate(start.x + dx, start.y + dy); context.rotate(angle);
  context.beginPath(); context.moveTo(0, 0); context.lineTo(-7, -3); context.lineTo(-7, 3); context.closePath(); context.fill(); context.restore();
}

function allDrones(kind) {
  const drones = (state?.drones || []).filter(
    drone => drone.connected && drone.operator_enabled && !drone.safety_excluded);
  if (!drones.length) return toast("No connected drones", true);
  const altitude = Number(document.getElementById("takeoff-altitude").value);
  api("/api/swarm-command", { command: kind, altitude_m: altitude }).catch(() => {});
}

function targetPayload() {
  return {
    x: Number(document.getElementById("target-x").value),
    y: Number(document.getElementById("target-y").value),
    z: Number(document.getElementById("target-z").value),
    cruise_speed_m_s: Number(document.getElementById("target-speed").value),
    use_fixed_wing: document.getElementById("target-fixed-wing").checked,
  };
}

function addCurrentTarget() {
  return api("/api/targets", targetPayload());
}

const droneGrid = document.getElementById("drone-grid");
droneGrid.addEventListener("pointerdown", () => {
  droneInteractionUntil = Date.now() + 1500;
});
droneGrid.addEventListener("input", event => {
  droneInteractionUntil = Date.now() + 1500;
  if (event.target.dataset.droneSpeed) {
    speedDrafts.set(event.target.dataset.droneSpeed, event.target.value);
  }
  if (event.target.dataset.droneLidarRange) {
    const droneId = event.target.dataset.droneLidarRange;
    lidarRangeDrafts.set(droneId, event.target.value);
    const output = droneGrid.querySelector(`[data-lidar-range-output="${droneId}"]`);
    if (output) output.value = `${event.target.value} m`;
  }
});

document.addEventListener("click", event => {
  const target = event.target;
  if (target.dataset.command) api("/api/swarm-command", { command: target.dataset.command }).catch(() => {});
  if (target.dataset.all) allDrones(target.dataset.all);
  if (target.dataset.droneArm) api("/api/swarm-command", { command: "arm", drone_id: target.dataset.droneArm }).catch(() => {});
  if (target.dataset.droneTakeoff) api("/api/swarm-command", { command: "takeoff", drone_id: target.dataset.droneTakeoff, altitude_m: Number(document.getElementById("takeoff-altitude").value) }).catch(() => {});
  if (target.dataset.droneHome) api("/api/swarm-command", { command: "home", drone_id: target.dataset.droneHome }).catch(() => {});
  if (target.dataset.droneLand) api("/api/swarm-command", { command: "land", drone_id: target.dataset.droneLand }).catch(() => {});
  if (target.dataset.droneMembership) api("/api/swarm-command", {
    command: target.dataset.membershipCommand, drone_id: target.dataset.droneMembership,
  }).then(() => { droneInteractionUntil = 0; refresh(); }).catch(() => {});
  if (target.dataset.setDroneSpeed) {
    const input = document.querySelector(`[data-drone-speed="${target.dataset.setDroneSpeed}"]`);
    api("/api/drone/speed", { drone_id: target.dataset.setDroneSpeed, cruise_speed_m_s: Number(input.value) })
      .then(() => { speedDrafts.delete(target.dataset.setDroneSpeed); droneInteractionUntil = 0; refresh(); })
      .catch(() => {});
  }
  if (target.dataset.clearDroneSpeed) {
    api("/api/drone/speed", { drone_id: target.dataset.clearDroneSpeed, clear: true })
      .then(() => { speedDrafts.delete(target.dataset.clearDroneSpeed); droneInteractionUntil = 0; refresh(); })
      .catch(() => {});
  }
  if (target.dataset.setDroneLidarRange) {
    const droneId = target.dataset.setDroneLidarRange;
    const input = document.querySelector(`[data-drone-lidar-range="${droneId}"]`);
    api("/api/drone/lidar-range", { drone_id: droneId, lidar_range_m: Number(input.value) })
      .then(() => { lidarRangeDrafts.delete(droneId); droneInteractionUntil = 0; refresh(); })
      .catch(() => {});
  }
  if (target.dataset.removeTarget) api("/api/targets/remove", { target_id: Number(target.dataset.removeTarget) }).catch(() => {});
});

document.getElementById("add-target").addEventListener("click", () => {
  addCurrentTarget().catch(() => {});
});

canvas.addEventListener("mousedown", event => { dragging = true; moved = false; dragStart = { x: event.clientX, y: event.clientY, viewX: view.x, viewY: view.y }; });
window.addEventListener("mouseup", event => {
  if (!dragging) return;
  dragging = false;
  if (!moved && event.target === canvas) {
    const point = screenToWorld(event.clientX, event.clientY);
    document.getElementById("target-x").value = point.x.toFixed(1);
    document.getElementById("target-y").value = point.y.toFixed(1);
    addCurrentTarget().catch(() => {});
  }
});
window.addEventListener("mousemove", event => {
  if (dragging) {
    const dx = event.clientX - dragStart.x, dy = event.clientY - dragStart.y;
    if (Math.abs(dx) + Math.abs(dy) > 4) moved = true;
    view.x = dragStart.viewX - dx * view.metersPerPixel;
    view.y = dragStart.viewY + dy * view.metersPerPixel;
    drawMap();
  }
  const point = screenToWorld(event.clientX, event.clientY);
  document.getElementById("map-coordinates").textContent = `E ${point.x.toFixed(1)} · N ${point.y.toFixed(1)}`;
});
canvas.addEventListener("wheel", event => {
  event.preventDefault();
  const before = screenToWorld(event.clientX, event.clientY);
  view.metersPerPixel = Math.min(20, Math.max(.08, view.metersPerPixel * (event.deltaY > 0 ? 1.15 : .87)));
  const after = screenToWorld(event.clientX, event.clientY);
  view.x += before.x - after.x; view.y += before.y - after.y; drawMap();
}, { passive: false });
document.getElementById("zoom-in").onclick = () => { view.metersPerPixel *= .75; drawMap(); };
document.getElementById("zoom-out").onclick = () => { view.metersPerPixel *= 1.3; drawMap(); };
document.getElementById("map-center").onclick = () => { view = { x: 0, y: 0, metersPerPixel: 2.2 }; drawMap(); };
window.addEventListener("resize", resizeCanvas);

resizeCanvas();
refresh();
setInterval(refresh, 500);
setInterval(refreshCameras, 250);
