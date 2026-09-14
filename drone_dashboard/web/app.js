"use strict";

const colors = ["#ff6572", "#57df8c", "#62a9ff", "#ffbd52", "#bb77ff", "#49d8d0"];
const DASHBOARD_RATE_HZ = 60;
const DASHBOARD_INTERVAL_MS = 1000 / DASHBOARD_RATE_HZ;
const canvas = document.getElementById("map");
const context = canvas.getContext("2d");
let state = null;
let view = { x: 0, y: 0, metersPerPixel: 2.2 };
let dragging = false;
let moved = false;
let dragStart = null;
let cameraIds = [];
let cameraCleanups = [];
let photoParentDirectoryHandle = null;
let photoDirectoryHandle = null;
let photoServerDirectoryConfigured = false;
let photoServerDirectoryPath = "";
let recordParentDirectoryHandle = null;
let recordDirectoryHandle = null;
let recordServerDirectoryConfigured = false;
let recordServerDirectoryPath = "";
const cameraRecordings = new Map();
const RECORDING_FRAME_RATE = 30;
const RECORDING_STOP_TIMEOUT_MS = 5000;
let gimbalTargetIds = [];
let heldGimbal = null;
let droneInteractionUntil = 0;
const speedDrafts = new Map();
let runtimeSettings = [];
let runtimeSettingValues = {};
let runtimeSettingTargets = [];
let settingsTarget = "ALL";
const runtimeSettingDrafts = new Map();
let settingsDirty = false;
let settingsPreviousTarget = "ALL";
let settingsApplying = false;
let settingsLoaded = false;
let runtimeSettingProfiles = [];
let selectedSettingsProfile = "";

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

async function api(path, payload, { silent = false } = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const result = await response.json();
  if (!silent) {
    toast(result.message || (result.ok ? "Command accepted" : "Command rejected"), !result.ok);
  }
  if (!response.ok) throw new Error(result.message);
  return result;
}

async function getJson(path) {
  const response = await fetch(path, { cache: "no-store" });
  const result = await response.json();
  if (!response.ok) throw new Error(result.message || `Request failed: ${response.status}`);
  return result;
}

function toast(message, error = false) {
  const element = document.getElementById("toast");
  element.textContent = message;
  element.className = `toast show${error ? " error" : ""}`;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => element.className = "toast", 3600);
}

async function refresh(signal) {
  if (document.hidden) return;
  try {
    const response = await fetch("/api/state", { cache: "no-store", signal });
    if (!response.ok) throw new Error(`State request failed: ${response.status}`);
    state = await response.json();
    if (signal.aborted) return;
    render();
  } catch (error) {
    setConnection(false, "Dashboard offline");
    throw error;
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
  renderGimbalTargets();
  renderSettingsTargets();
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
    const controlsDisabled = drone.connected && routingEnabled ? "" : "disabled";
    const takeoffDisabled = controlsDisabled === "" && drone.armed ? "" : "disabled";
    const forceDisarmDisabled = drone.connected && drone.armed ? "" : "disabled";
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
      <div class="drone-flags">
        <span class="flag ${drone.armed ? "on" : ""}">${drone.armed ? "ARMED" : "DISARMED"}</span>
        <span class="flag ${drone.offboard ? "on" : ""}">${drone.offboard ? "OFFBOARD" : "MANUAL"}</span>
        <span class="flag ${drone.localized ? "on" : ""}">LOCALIZED</span>
        <span class="flag ${drone.available ? "on" : ""}">${!routingEnabled ? "EXCLUDED" : (drone.busy ? "BUSY" : "AVAILABLE")}</span>
      </div>
      <div class="apf-line">APF ${telemetry?.active_mode || "—"} · obstacle ${obstacle == null ? "—" : obstacle.toFixed(1) + " m"}</div>
      <div class="drone-actions">
        <button class="button arm" data-drone-arm="${drone.drone_id}" ${controlsDisabled}>ARM</button>
        <button class="button primary" data-drone-takeoff="${drone.drone_id}" ${takeoffDisabled}>TAKEOFF</button>
        <button class="button" data-drone-home="${drone.drone_id}" ${controlsDisabled}>HOME</button>
        <button class="button danger" data-drone-land="${drone.drone_id}" ${controlsDisabled}>LAND</button>
        <button class="button danger ghost" data-drone-force-disarm="${drone.drone_id}" ${forceDisarmDisabled}>FORCE DISARM</button>
        <button class="button ${enabled ? "danger ghost" : "arm"}" data-drone-membership="${drone.drone_id}" data-membership-command="${membershipCommand}" ${membershipDisabled}>${membershipLabel}</button>
      </div>
    </article>`;
  }).join("");
}

function renderCameras() {
  const ids = state.drones.filter(drone => drone.connected).map(drone => drone.drone_id);
  if (JSON.stringify(ids) === JSON.stringify(cameraIds)) return;
  cameraRecordings.forEach((recording, id) => {
    if (!ids.includes(id)) stopCameraRecording(id);
  });
  cameraCleanups.forEach(cleanup => cleanup());
  cameraIds = ids;
  const grid = document.getElementById("camera-grid");
  const snapshotDisabled = photoDirectoryHandle || photoServerDirectoryConfigured ? "" : "disabled";
  const recordDisabled = recordDirectoryHandle || recordServerDirectoryConfigured ? "" : "disabled";
  grid.innerHTML = ids.length ? ids.map(id =>
    `<div class="camera">
      <img id="camera-${id}" alt="${id} camera">
      <span class="camera-label">${id}</span>
      <div class="camera-actions">
        <button class="button camera-snapshot" data-camera-snapshot="${id}" ${snapshotDisabled}>SNAPSHOT</button>
        <button class="button camera-record" data-camera-record="${id}" ${recordDisabled}>RECORD</button>
      </div>
    </div>`
  ).join("") : `<div class="camera-empty">Camera streams appear when drones connect</div>`;
  cameraCleanups = ids.map(id => startCamera(id));
  updateCameraActionAvailability();
}

function renderGimbalTargets() {
  const select = document.getElementById("gimbal-target");
  if (!select) return;
  const ids = state.drones
    .filter(drone => drone.registered && drone.connected && drone.has_gimbal &&
      drone.operator_enabled && !drone.safety_excluded)
    .map(drone => drone.drone_id)
    .sort();
  const current = select.value || "ALL";
  if (JSON.stringify(ids) !== JSON.stringify(gimbalTargetIds)) {
    gimbalTargetIds = ids;
    select.innerHTML = `<option value="ALL">ALL</option>` +
      ids.map(id => `<option value="${id}">${id}</option>`).join("");
  }
  select.value = ids.includes(current) || current === "ALL" ? current : "ALL";
  const disabled = ids.length === 0;
  document.querySelectorAll("[data-gimbal-command]").forEach(button => {
    button.disabled = disabled;
  });
  document.getElementById("gimbal-status").textContent = disabled ?
    "No connected registered drone reports gimbal capability." :
    "Hold a direction to move the selected gimbal. Release to stop.";
}

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, character => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
  }[character]));
}

function renderSettingsTargets() {
  const select = document.getElementById("settings-target");
  if (!select || !state) return;
  const ids = state.drones.filter(drone => drone.connected)
    .map(drone => drone.drone_id).sort();
  const options = ["ALL", ...ids];
  const currentOptions = [...select.options].map(option => option.value);
  if (JSON.stringify(options) !== JSON.stringify(currentOptions)) {
    select.innerHTML = `<option value="ALL">ALL CONNECTED DRONES</option>` +
      ids.map(id => `<option value="${escapeHtml(id)}">${escapeHtml(id)}</option>`).join("");
  }
  if (settingsTarget !== "ALL" && !ids.includes(settingsTarget)) settingsTarget = "ALL";
  select.value = settingsTarget;
  const status = document.getElementById("settings-target-status");
  if (status) {
    status.textContent = settingsTarget === "ALL" ?
      "Edit several values, then apply them together to every connected drone." :
      `Edit several values, then apply them together to ${settingsTarget}.`;
  }
  updateSettingsFooter();
}

function renderSettingsProfiles() {
  const select = document.getElementById("settings-profile-select");
  if (!select) return;
  const options = ["", ...runtimeSettingProfiles.map(profile => profile.name)];
  const currentOptions = [...select.options].map(option => option.value);
  if (JSON.stringify(options) !== JSON.stringify(currentOptions)) {
    select.innerHTML = `<option value="">NO PROFILE SELECTED</option>` +
      runtimeSettingProfiles.map(profile =>
        `<option value="${escapeHtml(profile.name)}">${escapeHtml(profile.name)}</option>`
      ).join("");
  }
  if (!runtimeSettingProfiles.some(profile => profile.name === selectedSettingsProfile)) {
    selectedSettingsProfile = "";
  }
  select.value = selectedSettingsProfile;
  const profile = runtimeSettingProfiles.find(item => item.name === selectedSettingsProfile);
  const status = document.getElementById("settings-profile-status");
  if (status) {
    status.textContent = profile ?
      `Saved ${profile.name} profile · ${Object.keys(profile.values || {}).length} values. Loading it creates unapplied changes; press APPLY CHANGES to send them.` :
      "Loading a profile creates unapplied changes. Press APPLY CHANGES to send them to the selected drone(s).";
  }
  const loadButton = document.getElementById("load-settings-profile");
  const deleteButton = document.getElementById("delete-settings-profile");
  if (loadButton) loadButton.disabled = !profile || settingsApplying;
  if (deleteButton) deleteButton.disabled = !profile || settingsApplying;
}

function settingTargetsForDisplay() {
  if (settingsTarget !== "ALL") return [settingsTarget];
  return runtimeSettingTargets.length ? runtimeSettingTargets :
    (state?.drones || []).filter(drone => drone.connected).map(drone => drone.drone_id);
}

function settingValue(setting) {
  if (setting.scope === "dashboard") {
    return runtimeSettingValues.dashboard?.[setting.key] ?? setting.default;
  }
  const values = settingTargetsForDisplay()
    .map(droneId => runtimeSettingValues[droneId]?.[setting.key])
    .filter(value => value !== undefined && value !== null);
  if (!values.length) return setting.default;
  if (values.every(value => value === values[0])) return values[0];
  return null;
}

function displayedSettingValue(setting) {
  return runtimeSettingDrafts.has(setting.key) ?
    runtimeSettingDrafts.get(setting.key) : settingValue(setting);
}

function draftMatchesCurrent(setting, rawValue) {
  const current = settingValue(setting);
  if (current === null || current === undefined) return false;
  if (setting.type === "bool") return Boolean(rawValue) === Boolean(current);
  if (setting.type === "choice") return String(rawValue) === String(current);
  const value = Number(rawValue);
  return Number.isFinite(value) && value === Number(current);
}

function updateSettingsFooter() {
  const status = document.getElementById("settings-dirty-status");
  const applyButton = document.getElementById("apply-settings");
  const discardButton = document.getElementById("discard-settings");
  const revertButton = document.getElementById("revert-settings");
  const count = runtimeSettingDrafts.size;
  settingsDirty = count > 0;
  if (status) {
    status.textContent = settingsDirty ?
      `${count} unapplied change${count === 1 ? "" : "s"}` : "No unapplied changes";
    status.classList.toggle("is-dirty", settingsDirty);
  }
  if (applyButton) applyButton.disabled = settingsApplying || !settingsDirty;
  if (discardButton) discardButton.disabled = settingsApplying || !settingsDirty;
  if (revertButton) revertButton.disabled = settingsApplying || !runtimeSettings.length;
}

function setSettingsError(message = "") {
  const error = document.getElementById("settings-error");
  if (!error) return;
  error.textContent = message;
  error.hidden = !message;
}

function clearInvalidSettingInputs() {
  document.querySelectorAll("[data-setting-input].setting-invalid")
    .forEach(input => input.classList.remove("setting-invalid"));
}

function markSettingsInvalid(keys, message) {
  for (const key of keys) {
    const input = document.querySelector(`[data-setting-input="${CSS.escape(key)}"]`);
    if (input) input.classList.add("setting-invalid");
  }
  setSettingsError(message);
}

function renderSettings() {
  const container = document.getElementById("settings-content");
  if (!container) return;
  if (!runtimeSettings.length) {
    container.innerHTML = `<div class="settings-empty">No runtime settings are available yet.</div>`;
    return;
  }
  const existingGroups = container.querySelectorAll("details.settings-group");
  const openGroups = new Set([...existingGroups].filter(group => group.open)
    .map(group => group.dataset.settingsCategory));
  const hasRenderedGroups = existingGroups.length > 0;
  const groups = new Map();
  runtimeSettings.forEach(setting => {
    if (!groups.has(setting.category)) groups.set(setting.category, []);
    groups.get(setting.category).push(setting);
  });
  container.innerHTML = [...groups.entries()].map(([category, settings], index) => `
    <details class="settings-group" data-settings-category="${escapeHtml(category)}" ${
      (!hasRenderedGroups && index === 0) || openGroups.has(category) ? "open" : ""}>
      <summary class="settings-group-title">
        <span class="settings-folder-label"><strong>${escapeHtml(category)}</strong></span>
        <span class="settings-folder-chevron" aria-hidden="true">⌄</span>
      </summary>
      <div class="settings-group-body">
      ${settings.map(setting => {
        const value = displayedSettingValue(setting);
        const mixed = value === null;
        const description = escapeHtml(setting.description);
        const label = escapeHtml(setting.label);
        const key = escapeHtml(setting.key);
        const unit = setting.unit ? `<span class="setting-unit">${escapeHtml(setting.unit)}</span>` : "";
        const control = setting.type === "bool" ?
          `<label class="setting-switch"><input type="checkbox" data-setting-input="${key}" ${value ? "checked" : ""}><span></span></label>` :
          setting.type === "choice" ?
          `<select class="setting-select" data-setting-input="${key}">
            ${setting.options.map(option => `<option value="${escapeHtml(option)}" ${String(value) === String(option) ? "selected" : ""}>${escapeHtml(option)}</option>`).join("")}
          </select>` :
          `<input class="setting-number" type="number" data-setting-input="${key}"
            ${mixed ? "" : `value="${escapeHtml(value)}"`} ${mixed ? 'placeholder="Mixed"' : ""}
            min="${setting.min}" max="${setting.max}" step="${setting.step}">`;
        return `<div class="setting-row">
          <div class="setting-name"><span>${label}</span>
            <button class="setting-info" type="button" title="${description}" aria-label="Explain ${label}">ⓘ<span class="setting-tooltip">${description}</span></button>
            ${mixed ? `<small>different values</small>` : ""}
          </div>
          <div class="setting-control">${control}${unit}</div>
        </div>`;
      }).join("")}
      </div>
    </details>`).join("");
}

async function loadSettings() {
  try {
    const [result, profilesResult] = await Promise.all([
      getJson("/api/settings"),
      getJson("/api/settings/profiles"),
    ]);
    runtimeSettings = result.settings || [];
    runtimeSettingValues = result.values || {};
    runtimeSettingTargets = result.targets || [];
    runtimeSettingProfiles = profilesResult.profiles || [];
    runtimeSettingDrafts.clear();
    settingsDirty = false;
    settingsLoaded = true;
    setSettingsError();
    renderSettingsTargets();
    renderSettingsProfiles();
    renderSettings();
    updateSettingsFooter();
    restartCameraStreams();
  } catch (error) {
    setSettingsError(error.message || "Could not load runtime settings");
    document.getElementById("settings-content").innerHTML =
      `<div class="settings-empty">Runtime settings could not be loaded.</div>`;
  }
}

function profileValues() {
  const values = {};
  for (const setting of runtimeSettings) {
    const value = displayedSettingValue(setting);
    if (value === null || value === undefined || value === "") continue;
    if (setting.type === "bool") values[setting.key] = Boolean(value);
    else if (setting.type === "choice" && typeof setting.options[0] === "number") {
      values[setting.key] = Number(value);
    } else if (setting.type === "double" || setting.type === "integer") {
      values[setting.key] = Number(value);
    } else {
      values[setting.key] = String(value);
    }
  }
  return values;
}

async function saveSettingsProfile() {
  const input = document.getElementById("settings-profile-name");
  const name = input.value.trim();
  if (!name) {
    setSettingsError("Enter a profile name before saving.");
    input.focus();
    return;
  }
  const values = profileValues();
  if (!Object.keys(values).length) {
    setSettingsError("No complete runtime settings are available to save.");
    return;
  }
  try {
    await api("/api/settings/profiles/save", { name, values });
    selectedSettingsProfile = name;
    await loadSettingProfiles();
    renderSettingsProfiles();
  } catch (error) {
    setSettingsError(error.message || "Could not save settings profile");
  }
}

async function loadSettingProfiles() {
  const result = await getJson("/api/settings/profiles");
  runtimeSettingProfiles = result.profiles || [];
}

function loadSettingsProfile() {
  const profile = runtimeSettingProfiles.find(item => item.name === selectedSettingsProfile);
  if (!profile) return;
  if (settingsDirty && !window.confirm(
    "Discard the current unapplied changes and load this profile?")) return;
  runtimeSettingDrafts.clear();
  for (const setting of runtimeSettings) {
    if (!Object.prototype.hasOwnProperty.call(profile.values || {}, setting.key)) continue;
    const value = profile.values[setting.key];
    if (!draftMatchesCurrent(setting, value)) runtimeSettingDrafts.set(setting.key, value);
  }
  setSettingsError();
  renderSettings();
  updateSettingsFooter();
  renderSettingsProfiles();
}

async function deleteSettingsProfile() {
  const profile = runtimeSettingProfiles.find(item => item.name === selectedSettingsProfile);
  if (!profile || !window.confirm(`Delete the '${profile.name}' settings profile?`)) return;
  try {
    await api("/api/settings/profiles/delete", { name: profile.name });
    selectedSettingsProfile = "";
    document.getElementById("settings-profile-name").value = "";
    await loadSettingProfiles();
    renderSettingsProfiles();
  } catch (error) {
    setSettingsError(error.message || "Could not delete settings profile");
  }
}

function openSettings() {
  const modal = document.getElementById("settings-modal");
  modal.hidden = false;
  settingsPreviousTarget = settingsTarget;
  renderSettingsTargets();
  if (settingsLoaded) renderSettings();
  else loadSettings();
  document.getElementById("close-settings").focus();
}

function closeSettings() {
  closeSettingsConfirm();
  document.getElementById("settings-modal").hidden = true;
}

function requestCloseSettings() {
  if (!settingsDirty || settingsApplying) {
    closeSettings();
    return;
  }
  openSettingsConfirm();
}

function openSettingsConfirm() {
  const confirm = document.getElementById("settings-confirm");
  confirm.hidden = false;
  document.getElementById("confirm-apply-settings").focus();
}

function closeSettingsConfirm() {
  const confirm = document.getElementById("settings-confirm");
  if (confirm) confirm.hidden = true;
}

function discardSettings() {
  runtimeSettingDrafts.clear();
  settingsDirty = false;
  setSettingsError();
  renderSettings();
  updateSettingsFooter();
}

function revertSettingsToDefaults() {
  setSettingsError();
  for (const setting of runtimeSettings) {
    if (draftMatchesCurrent(setting, setting.default)) runtimeSettingDrafts.delete(setting.key);
    else runtimeSettingDrafts.set(setting.key, setting.default);
  }
  renderSettings();
  updateSettingsFooter();
}

async function applySettings(closeAfter = false) {
  if (!runtimeSettingDrafts.size || settingsApplying) {
    if (closeAfter && !settingsDirty) closeSettings();
    return;
  }
  setSettingsError();
  clearInvalidSettingInputs();
  const changes = [];
  for (const [key, rawValue] of runtimeSettingDrafts) {
    const setting = runtimeSettings.find(candidate => candidate.key === key);
    if (!setting) continue;
    const value = setting.type === "bool" ? Boolean(rawValue) :
      (setting.type === "choice" ?
        (typeof setting.options[0] === "number" ? Number(rawValue) : String(rawValue)) :
        Number(rawValue));
    const numericSetting = setting.type === "double" || setting.type === "integer";
    if (numericSetting &&
        (!Number.isFinite(value) || value < setting.min || value > setting.max)) {
      markSettingsInvalid([key], `${setting.label}: enter a value from ${setting.min} to ${setting.max}`);
      return;
    }
    if (setting.type === "integer" && !Number.isInteger(value)) {
      markSettingsInvalid([key], `${setting.label}: enter a whole number`);
      return;
    }
    if (setting.type === "choice" && !setting.options.some(option =>
      String(option) === String(value))) {
      markSettingsInvalid([key], `${setting.label}: choose one of ${setting.options.join(", ")}`);
      return;
    }
    changes.push({ key, value });
  }
  if (!changes.length) return;
  settingsApplying = true;
  updateSettingsFooter();
  try {
    await api("/api/settings", { target: settingsTarget, changes }, { silent: true });
    toast("Runtime settings applied");
    await loadSettings();
    closeSettingsConfirm();
    if (closeAfter) closeSettings();
  } catch (error) {
    markSettingsInvalid(changes.map(change => change.key),
      error.message || "Could not apply runtime settings");
  } finally {
    settingsApplying = false;
    updateSettingsFooter();
  }
}

function startCamera(id) {
  const image = document.getElementById(`camera-${id}`);
  const label = image.parentElement.querySelector(".camera-label");
  let displayedUrl = null;
  const stop = startPolling(async signal => {
    if (document.hidden) return;
    let nextUrl = null;
    try {
      const response = await fetch(`/api/camera/${encodeURIComponent(id)}.jpg`, {
        cache: "no-store", signal,
      });
      if (!response.ok) throw new Error(`Camera request failed: ${response.status}`);
      nextUrl = URL.createObjectURL(await response.blob());
      const nextImage = new Image();
      nextImage.src = nextUrl;
      await nextImage.decode();
      if (signal.aborted || !image.isConnected) return;
      drawRecordingFrame(id, nextImage);
      // Replace only after a complete image is decoded; retain the last frame on error.
      image.src = nextUrl;
      if (displayedUrl) URL.revokeObjectURL(displayedUrl);
      displayedUrl = nextUrl;
      nextUrl = null;
      label.textContent = id;
    } catch (error) {
      label.textContent = `${id} · reconnecting${displayedUrl ? " (last frame)" : ""}`;
      throw error;
    } finally {
      if (nextUrl) URL.revokeObjectURL(nextUrl);
    }
  }, cameraPollingIntervalMs());
  return () => {
    stop();
    if (displayedUrl) URL.revokeObjectURL(displayedUrl);
  };
}

function cameraPollingIntervalMs() {
  const fps = Number(runtimeSettingValues.dashboard?.camera_fps || DASHBOARD_RATE_HZ);
  return 1000 / Math.max(1, Math.min(DASHBOARD_RATE_HZ, fps));
}

function restartCameraStreams() {
  if (!cameraIds.length) return;
  cameraCleanups.forEach(cleanup => cleanup());
  cameraCleanups = cameraIds.map(id => startCamera(id));
}

async function saveCameraSnapshot(id, button) {
  button.disabled = true;
  try {
    if (photoDirectoryHandle) {
      await ensureDirectoryWritePermission(photoDirectoryHandle);
      const response = await fetch(`/api/camera/${encodeURIComponent(id)}.jpg`, {
        cache: "no-store",
      });
      if (!response.ok) throw new Error(`Camera frame unavailable: ${response.status}`);
      const droneDirectory = await photoDirectoryHandle.getDirectoryHandle(id, { create: true });
      const filename = `snapshot_${new Date().toISOString().replace(/[:.]/g, "-")}.jpg`;
      const fileHandle = await droneDirectory.getFileHandle(filename, { create: true });
      const writable = await fileHandle.createWritable();
      try {
        await writable.write(await response.blob());
        await writable.close();
      } catch (error) {
        await writable.abort();
        throw error;
      }
      toast(`Snapshot saved: ${photoDirectoryHandle.name}/${id}/${filename}`);
    } else if (photoServerDirectoryConfigured) {
      const response = await fetch(`/api/camera/${encodeURIComponent(id)}/snapshot`, {
        method: "POST",
      });
      const result = await response.json();
      if (!response.ok) throw new Error(result.message || "Snapshot failed");
      toast(result.message || "Snapshot saved");
    } else {
      throw new Error("Choose a snapshot folder first");
    }
  } catch (error) {
    toast(error.message || "Snapshot failed", true);
  } finally {
    button.disabled = false;
  }
}

async function ensureDirectoryWritePermission(directoryHandle) {
  if (!directoryHandle.queryPermission || !directoryHandle.requestPermission) return;
  const current = await directoryHandle.queryPermission({ mode: "readwrite" });
  if (current === "granted") return;
  const requested = await directoryHandle.requestPermission({ mode: "readwrite" });
  if (requested !== "granted") throw new Error("Folder write permission was not granted");
}

function setMediaStorageStatus(kind, message, error = false) {
  const status = document.getElementById(`${kind}-storage-status`);
  status.textContent = message;
  status.className = error ? "media-storage-error" : "";
  updateCameraActionAvailability();
}

function mediaParentDirectory(kind) {
  return kind === "photo" ? photoParentDirectoryHandle : recordParentDirectoryHandle;
}

function setMediaDirectories(kind, parentDirectory, directory) {
  if (kind === "photo") {
    photoParentDirectoryHandle = parentDirectory;
    photoDirectoryHandle = directory;
    photoServerDirectoryConfigured = false;
    photoServerDirectoryPath = "";
  } else {
    recordParentDirectoryHandle = parentDirectory;
    recordDirectoryHandle = directory;
    recordServerDirectoryConfigured = false;
    recordServerDirectoryPath = "";
  }
}

async function chooseMediaFolder(kind) {
  if (!window.showDirectoryPicker) {
    await chooseServerMediaFolder(kind);
    return;
  }
  try {
    const directoryHandle = await window.showDirectoryPicker({ mode: "readwrite" });
    await ensureDirectoryWritePermission(directoryHandle);
    setMediaDirectories(kind, directoryHandle, directoryHandle);
    setMediaStorageStatus(kind, `Using local folder: ${directoryHandle.name}`);
  } catch (error) {
    if (error.name !== "AbortError") {
      setMediaStorageStatus(kind, error.message || "Could not select folder", true);
      toast(error.message || "Could not select folder", true);
    }
  }
}

function activateServerMediaDirectory(kind, path) {
  if (kind === "photo") {
    photoParentDirectoryHandle = null;
    photoDirectoryHandle = null;
    photoServerDirectoryConfigured = true;
    photoServerDirectoryPath = path;
  } else {
    recordParentDirectoryHandle = null;
    recordDirectoryHandle = null;
    recordServerDirectoryConfigured = true;
    recordServerDirectoryPath = path;
  }
  setMediaStorageStatus(kind, `Using server folder: ${path}`);
}

async function loadPreconfiguredMediaStorage() {
  try {
    const response = await fetch("/api/storage", { cache: "no-store" });
    const result = await response.json();
    if (!response.ok || !result.preconfigured) return;
    activateServerMediaDirectory("photo", result.photo_path);
    activateServerMediaDirectory("record", result.record_path);
    document.getElementById("photo-server-path").value = result.photo_path;
    document.getElementById("record-server-path").value = result.record_path;
  } catch (error) {
    // Manual folder selection remains available if startup discovery fails.
  }
}

async function chooseServerMediaFolder(kind) {
  try {
    setMediaStorageStatus(kind, "Waiting for native folder selection...");
    const response = await fetch("/api/storage/pick", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ kind }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.message || "Could not open folder picker");
    if (result.cancelled) {
      setMediaStorageStatus(kind, "Folder selection cancelled.");
      return;
    }
    activateServerMediaDirectory(kind, result.path);
    toast(result.message || "Folder ready");
  } catch (error) {
    setMediaStorageStatus(kind, error.message || "Could not open folder picker", true);
    toast(error.message || "Could not open folder picker", true);
  }
}

async function configureServerMediaDirectory(kind, requestedPath = "") {
  const path = requestedPath || document.getElementById(`${kind}-server-path`).value.trim();
  if (!path) {
    toast("Enter an absolute folder path", true);
    return;
  }
  try {
    const response = await fetch("/api/storage", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ kind, path }),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.message || "Could not configure folder");
    activateServerMediaDirectory(kind, path);
    toast(result.message || "Folder ready");
  } catch (error) {
    setMediaStorageStatus(kind, error.message || "Could not configure folder", true);
    toast(error.message || "Could not configure folder", true);
  }
}

async function createMediaFolder(kind) {
  const parentDirectory = mediaParentDirectory(kind);
  const name = document.getElementById(`${kind}-folder-name`).value.trim();
  if (!name || name === "." || name === ".." || /[\\/:*?"<>|]/.test(name)) {
    toast("Enter a valid folder name", true);
    return;
  }
  if (!parentDirectory) {
    const serverParentPath = kind === "photo" ?
      photoServerDirectoryPath : recordServerDirectoryPath;
    if (!serverParentPath) {
      toast("Choose a parent folder first", true);
      return;
    }
    const childPath = `${serverParentPath.replace(/\/+$/, "")}/${name}`;
    await configureServerMediaDirectory(kind, childPath);
    return;
  }
  try {
    const directoryHandle = await parentDirectory.getDirectoryHandle(name, { create: true });
    await ensureDirectoryWritePermission(directoryHandle);
    setMediaDirectories(kind, parentDirectory, directoryHandle);
    setMediaStorageStatus(kind, `Using local folder: ${parentDirectory.name}/${directoryHandle.name}`);
    toast(`Folder ready: ${directoryHandle.name}`);
  } catch (error) {
    setMediaStorageStatus(kind, error.message || "Could not create folder", true);
    toast(error.message || "Could not create folder", true);
  }
}

function updateCameraActionAvailability() {
  document.querySelectorAll("[data-camera-snapshot]").forEach(button => {
    button.disabled = !photoDirectoryHandle && !photoServerDirectoryConfigured;
  });
  document.querySelectorAll("[data-camera-record]").forEach(button => {
    const id = button.dataset.cameraRecord;
    const recording = cameraRecordings.get(id);
    const active = recording?.state === "recording";
    const finalizing = recording && !active;
    button.disabled = Boolean(finalizing) ||
      (!recordDirectoryHandle && !recordServerDirectoryConfigured && !recording);
    button.textContent = finalizing ? "SAVING..." : (active ? "STOP RECORDING" : "RECORD");
    button.classList.toggle("is-recording", active);
  });
}

function recordingMimeType() {
  return [
    "video/webm;codecs=vp9,opus",
    "video/webm;codecs=vp8,opus",
    "video/webm",
  ].find(type => MediaRecorder.isTypeSupported(type)) || "";
}

function drawRecordingFrame(id, sourceImage) {
  const recording = cameraRecordings.get(id);
  if (!recording || recording.serverManaged || recording.recorder.state !== "recording") return;
  const width = sourceImage.naturalWidth || sourceImage.width;
  const height = sourceImage.naturalHeight || sourceImage.height;
  if (!width || !height) return;
  if (recording.canvas.width !== width || recording.canvas.height !== height) {
    recording.canvas.width = width;
    recording.canvas.height = height;
  }
  recording.context.drawImage(sourceImage, 0, 0, width, height);
}

async function finishCameraRecording(id, recording) {
  try {
    const blob = new Blob(recording.chunks, { type: recording.recorder.mimeType || "video/webm" });
    if (!blob.size) throw new Error("Recording did not contain video data");
    if (recording.serverStorage) {
      const response = await fetch(`/api/camera/${encodeURIComponent(id)}/recording`, {
        method: "POST",
        headers: { "Content-Type": blob.type || "video/webm" },
        body: blob,
      });
      const result = await response.json();
      if (!response.ok) throw new Error(result.message || "Recording upload failed");
      toast(result.message || "Recording saved");
    } else {
      const droneDirectory = await recording.directoryHandle.getDirectoryHandle(id, { create: true });
      const filename = `recording_${new Date().toISOString().replace(/[:.]/g, "-")}.webm`;
      const fileHandle = await droneDirectory.getFileHandle(filename, { create: true });
      const writable = await fileHandle.createWritable();
      try {
        await writable.write(blob);
        await writable.close();
      } catch (error) {
        await writable.abort();
        throw error;
      }
      toast(`Recording saved: ${recording.directoryHandle.name}/${id}/${filename}`);
    }
  } catch (error) {
    toast(error.message || "Recording failed", true);
  } finally {
    clearTimeout(recording.stopTimer);
    recording.stream.getTracks().forEach(track => track.stop());
    if (cameraRecordings.get(id) === recording) cameraRecordings.delete(id);
    updateCameraActionAvailability();
  }
}

function finalizeCameraRecording(id, recording) {
  if (recording.finalizeStarted) return;
  recording.finalizeStarted = true;
  recording.state = "saving";
  clearTimeout(recording.stopTimer);
  updateCameraActionAvailability();
  finishCameraRecording(id, recording);
}

async function stopServerCameraRecording(id, recording) {
  try {
    const response = await fetch(`/api/camera/${encodeURIComponent(id)}/recording/stop`, {
      method: "POST",
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.message || "Recording could not stop");
    toast(result.message || "Recording saved");
  } catch (error) {
    toast(error.message || "Recording could not stop", true);
  } finally {
    if (cameraRecordings.get(id) === recording) cameraRecordings.delete(id);
    updateCameraActionAvailability();
  }
}

function stopCameraRecording(id) {
  const recording = cameraRecordings.get(id);
  if (!recording || recording.state !== "recording") return;
  recording.state = "stopping";
  updateCameraActionAvailability();
  if (recording.serverManaged) {
    stopServerCameraRecording(id, recording);
    return;
  }
  recording.stopTimer = setTimeout(
    () => finalizeCameraRecording(id, recording),
    RECORDING_STOP_TIMEOUT_MS,
  );
  if (recording.recorder.state === "inactive") {
    finalizeCameraRecording(id, recording);
    return;
  }
  try {
    recording.recorder.requestData();
  } catch (error) {
    // Some browsers flush the last chunk only through stop().
  }
  try {
    recording.recorder.stop();
  } catch (error) {
    finalizeCameraRecording(id, recording);
  }
}

async function startServerCameraRecording(id, button) {
  button.disabled = true;
  try {
    const response = await fetch(`/api/camera/${encodeURIComponent(id)}/recording/start`, {
      method: "POST",
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.message || "Recording could not start");
    cameraRecordings.set(id, {
      state: "recording",
      serverManaged: true,
    });
    toast(result.message || `Recording started: ${id}`);
  } catch (error) {
    toast(error.message || "Recording could not start", true);
  } finally {
    button.disabled = false;
    updateCameraActionAvailability();
  }
}

async function startCameraRecording(id, button) {
  if (!recordDirectoryHandle && !recordServerDirectoryConfigured) {
    toast("Choose a recording folder first", true);
    return;
  }
  if (!recordDirectoryHandle && recordServerDirectoryConfigured) {
    await startServerCameraRecording(id, button);
    return;
  }
  if (!window.MediaRecorder || !HTMLCanvasElement.prototype.captureStream) {
    toast("This browser does not support video recording", true);
    return;
  }
  button.disabled = true;
  let recording = null;
  try {
    if (recordDirectoryHandle) await ensureDirectoryWritePermission(recordDirectoryHandle);
    const image = document.getElementById(`camera-${id}`);
    const canvas = document.createElement("canvas");
    canvas.width = image.naturalWidth || 640;
    canvas.height = image.naturalHeight || 360;
    const mimeType = recordingMimeType();
    if (!mimeType) throw new Error("No supported WebM recording format");
    const stream = canvas.captureStream(RECORDING_FRAME_RATE);
    const recorder = new MediaRecorder(stream, {
      mimeType,
      videoBitsPerSecond: 4_000_000,
    });
    recording = {
      canvas,
      context: canvas.getContext("2d"),
      directoryHandle: recordDirectoryHandle,
      serverStorage: !recordDirectoryHandle,
      recorder,
      stream,
      chunks: [],
      state: "recording",
      finalizeStarted: false,
      stopTimer: null,
    };
    recorder.ondataavailable = event => {
      if (event.data.size > 0) recording.chunks.push(event.data);
    };
    recorder.onerror = event => {
      recording.errorMessage = event.error?.message || "MediaRecorder failed";
      stopCameraRecording(id);
    };
    recorder.onstop = () => { finalizeCameraRecording(id, recording); };
    recorder.start(1000);
    cameraRecordings.set(id, recording);
    button.disabled = false;
    updateCameraActionAvailability();
    toast(`Recording started: ${id}`);
  } catch (error) {
    if (recording) {
      clearTimeout(recording.stopTimer);
      recording.stream.getTracks().forEach(track => track.stop());
      if (cameraRecordings.get(id) === recording) cameraRecordings.delete(id);
    }
    toast(error.message || "Could not start recording", true);
    button.disabled = false;
    updateCameraActionAvailability();
  }
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
  const heading = drone.heading_valid && Number.isFinite(drone.heading_ned_rad) ?
    drone.heading_ned_rad : 0;
  context.save(); context.translate(p.x, p.y); context.rotate(heading); context.fillStyle = color; context.strokeStyle = "#071014"; context.lineWidth = 2;
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
  const drones = kind === "force_disarm" ?
    (state?.drones || []).filter(drone => drone.connected && drone.armed) :
    (state?.drones || []).filter(
      drone => drone.connected && drone.operator_enabled && !drone.safety_excluded);
  if (!drones.length) return toast("No connected drones", true);
  if (kind === "takeoff") {
    const unarmed = drones.filter(drone => !drone.armed).map(drone => drone.drone_id);
    if (unarmed.length) return toast(`ARM first: ${unarmed.join(", ")}`, true);
  }
  if (kind === "force_disarm" && !window.confirm(
    `FORCE DISARM ${drones.length} drone(s)? This stops the motors immediately and can cause a crash.`)) return;
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

function gimbalTargetPayload() {
  const target = document.getElementById("gimbal-target").value || "ALL";
  return target === "ALL" ?
    { target_mode: "all" } :
    { target_mode: "drone", drone_id: target };
}

function sendGimbalCommand(command, pressed, target = gimbalTargetPayload()) {
  return api("/api/gimbal-command", {
    ...target,
    command: command.toLowerCase(),
    pressed,
  });
}

function releaseGimbal() {
  if (!heldGimbal) return;
  const held = heldGimbal;
  heldGimbal = null;
  held.button.classList.remove("is-held");
  sendGimbalCommand("STOP", false, held.target).catch(() => {});
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
});

document.addEventListener("click", event => {
  const target = event.target;
  if (target.closest?.("#apply-settings")) {
    applySettings().catch(() => {});
    return;
  }
  if (target.closest?.("#discard-settings")) {
    discardSettings();
    return;
  }
  if (target.closest?.("#revert-settings")) {
    revertSettingsToDefaults();
    return;
  }
  if (target.closest?.("#confirm-keep-editing")) {
    closeSettingsConfirm();
    return;
  }
  if (target.closest?.("#confirm-discard-settings")) {
    discardSettings();
    closeSettings();
    return;
  }
  if (target.closest?.("#confirm-apply-settings")) {
    applySettings(true).catch(() => {});
    return;
  }
  const closeSettingsButton = target.closest?.("[data-close-settings]");
  if (closeSettingsButton) {
    requestCloseSettings();
    return;
  }
  if (target.dataset.cameraSnapshot) {
    saveCameraSnapshot(target.dataset.cameraSnapshot, target);
    return;
  }
  if (target.dataset.cameraRecord) {
    const id = target.dataset.cameraRecord;
    if (cameraRecordings.has(id)) stopCameraRecording(id);
    else startCameraRecording(id, target);
    return;
  }
  if (target.dataset.gimbalCommand === "HOME") {
    sendGimbalCommand("HOME", true).catch(() => {});
    return;
  }
  if (target.dataset.command) api("/api/swarm-command", { command: target.dataset.command }).catch(() => {});
  if (target.dataset.all) allDrones(target.dataset.all);
  if (target.dataset.droneArm) api("/api/swarm-command", { command: "arm", drone_id: target.dataset.droneArm }).catch(() => {});
  if (target.dataset.droneTakeoff) api("/api/swarm-command", { command: "takeoff", drone_id: target.dataset.droneTakeoff, altitude_m: Number(document.getElementById("takeoff-altitude").value) }).catch(() => {});
  if (target.dataset.droneHome) api("/api/swarm-command", { command: "home", drone_id: target.dataset.droneHome }).catch(() => {});
  if (target.dataset.droneLand) api("/api/swarm-command", { command: "land", drone_id: target.dataset.droneLand }).catch(() => {});
  if (target.dataset.droneForceDisarm && window.confirm(
    `FORCE DISARM ${target.dataset.droneForceDisarm}? This stops its motors immediately and can cause a crash.`)) {
    api("/api/swarm-command", {
      command: "force_disarm", drone_id: target.dataset.droneForceDisarm,
    }).catch(() => {});
  }
  if (target.dataset.droneMembership) api("/api/swarm-command", {
    command: target.dataset.membershipCommand, drone_id: target.dataset.droneMembership,
  }).then(() => { droneInteractionUntil = 0; }).catch(() => {});
  if (target.dataset.setDroneSpeed) {
    const input = document.querySelector(`[data-drone-speed="${target.dataset.setDroneSpeed}"]`);
    api("/api/drone/speed", { drone_id: target.dataset.setDroneSpeed, cruise_speed_m_s: Number(input.value) })
      .then(() => { speedDrafts.delete(target.dataset.setDroneSpeed); droneInteractionUntil = 0; })
      .catch(() => {});
  }
  if (target.dataset.clearDroneSpeed) {
    api("/api/drone/speed", { drone_id: target.dataset.clearDroneSpeed, clear: true })
      .then(() => { speedDrafts.delete(target.dataset.clearDroneSpeed); droneInteractionUntil = 0; })
      .catch(() => {});
  }
  if (target.dataset.removeTarget) api("/api/targets/remove", { target_id: Number(target.dataset.removeTarget) }).catch(() => {});
});

document.addEventListener("pointerdown", event => {
  const button = event.target.closest?.("[data-gimbal-command]");
  if (!button || button.dataset.gimbalCommand === "HOME" || button.disabled) return;
  event.preventDefault();
  releaseGimbal();
  heldGimbal = {
    button,
    target: gimbalTargetPayload(),
  };
  button.classList.add("is-held");
  button.setPointerCapture?.(event.pointerId);
  sendGimbalCommand(button.dataset.gimbalCommand, true, heldGimbal.target).catch(() => {});
});

document.addEventListener("pointerup", event => {
  releaseGimbal();
});
document.addEventListener("pointercancel", () => releaseGimbal());
window.addEventListener("blur", () => releaseGimbal());

document.getElementById("add-target").addEventListener("click", () => {
  addCurrentTarget().catch(() => {});
});
document.getElementById("open-settings").addEventListener("click", openSettings);
document.getElementById("close-settings").addEventListener("click", requestCloseSettings);
document.getElementById("settings-content").addEventListener("input", event => {
  const input = event.target.closest?.("[data-setting-input]");
  if (!input) return;
  const setting = runtimeSettings.find(candidate => candidate.key === input.dataset.settingInput);
  if (!setting) return;
  input.classList.remove("setting-invalid");
  setSettingsError();
  const value = setting.type === "bool" ? input.checked : input.value;
  if (draftMatchesCurrent(setting, value)) runtimeSettingDrafts.delete(setting.key);
  else runtimeSettingDrafts.set(setting.key, value);
  updateSettingsFooter();
});
document.getElementById("settings-target").addEventListener("change", event => {
  if (settingsDirty) {
    event.target.value = settingsPreviousTarget;
    toast("Apply or discard the current changes before changing the target", true);
    return;
  }
  settingsTarget = event.target.value;
  settingsPreviousTarget = settingsTarget;
  renderSettings();
});
document.getElementById("settings-profile-select").addEventListener("change", event => {
  selectedSettingsProfile = event.target.value;
  const profile = runtimeSettingProfiles.find(item => item.name === selectedSettingsProfile);
  document.getElementById("settings-profile-name").value = profile?.name || "";
  renderSettingsProfiles();
});
document.getElementById("load-settings-profile").addEventListener("click", loadSettingsProfile);
document.getElementById("save-settings-profile").addEventListener("click", () => {
  saveSettingsProfile().catch(() => {});
});
document.getElementById("delete-settings-profile").addEventListener("click", () => {
  deleteSettingsProfile().catch(() => {});
});
document.addEventListener("keydown", event => {
  if (event.key === "Escape" && !document.getElementById("settings-modal").hidden) {
    if (!document.getElementById("settings-confirm").hidden) closeSettingsConfirm();
    else requestCloseSettings();
  }
});
document.getElementById("choose-photo-folder").addEventListener("click", () => {
  chooseMediaFolder("photo");
});
document.getElementById("create-photo-folder").addEventListener("click", () => {
  createMediaFolder("photo");
});
document.getElementById("configure-photo-path").addEventListener("click", () => {
  configureServerMediaDirectory("photo");
});
document.getElementById("choose-record-folder").addEventListener("click", () => {
  chooseMediaFolder("record");
});
document.getElementById("create-record-folder").addEventListener("click", () => {
  createMediaFolder("record");
});
document.getElementById("configure-record-path").addEventListener("click", () => {
  configureServerMediaDirectory("record");
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
loadPreconfiguredMediaStorage();
const stopStatePolling = startPolling(refresh, DASHBOARD_INTERVAL_MS);
window.addEventListener("pagehide", event => {
  if (event.persisted) return;
  cameraRecordings.forEach((recording, id) => stopCameraRecording(id));
  stopStatePolling();
  cameraCleanups.forEach(cleanup => cleanup());
});
