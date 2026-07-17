const statusEl = document.getElementById('status');
const logEl = document.getElementById('log');
const logFilterEl = document.getElementById('logFilter');
const logCountEl = document.getElementById('logCount');
const logAutoscrollEl = document.getElementById('logAutoscroll');
const screenControlEl = document.getElementById('screenControl');
const screenPlanEl = document.getElementById('screenPlan');
const showControlBtn = document.getElementById('showControlBtn');
const showPlanBtn = document.getElementById('showPlanBtn');

const connectionEl = document.getElementById('connection');
const baudEl = document.getElementById('baud');
const takeoffAltEl = document.getElementById('takeoffAlt');
const gotoLatEl = document.getElementById('gotoLat');
const gotoLonEl = document.getElementById('gotoLon');
const gotoAltEl = document.getElementById('gotoAlt');
const addPointModeBtn = document.getElementById('addPointModeBtn');
const removeSelectedBtn = document.getElementById('removeSelectedBtn');
const clearLocalPlanBtn = document.getElementById('clearLocalPlanBtn');
const loadMissionBtn = document.getElementById('loadMissionBtn');
const uploadMissionBtn = document.getElementById('uploadMissionBtn');
const clearMissionBtn = document.getElementById('clearMissionBtn');
const readFileBtn = document.getElementById('readFileBtn');
const writeFileBtn = document.getElementById('writeFileBtn');
const missionFileInput = document.getElementById('missionFileInput');
const waypointTableBody = document.getElementById('waypointTableBody');

let planMap;
let markerLayer;
let pathLayer;
let homeMarker;
let addPointMode = true;
let selectedWaypointIndex = null;
let waypoints = [];
let homePosition = null;
let homeCentered = false;

const LOG_LIMIT = 500;
const logEntries = [];

function formatPayload(payload) {
  if (payload == null) {
    return '';
  }
  try {
    return JSON.stringify(payload, null, 2);
  } catch {
    return String(payload);
  }
}

function formatEntry(entry) {
  const head = `[${entry.time}] ${entry.msg}`;
  return entry.body ? `${head}\n${entry.body}` : head;
}

function renderLog() {
  const needle = logFilterEl.value.trim().toLowerCase();
  const visible = needle
    ? logEntries.filter((e) => formatEntry(e).toLowerCase().includes(needle))
    : logEntries;
  logEl.textContent = visible.map(formatEntry).join('\n\n');
  const suffix = needle ? ` (${logEntries.length} total)` : '';
  logCountEl.textContent = `${visible.length} ${visible.length === 1 ? 'entry' : 'entries'}${suffix}`;
  if (logAutoscrollEl.checked) {
    logEl.scrollTop = logEl.scrollHeight;
  }
}

function log(msg, payload) {
  logEntries.push({
    time: new Date().toISOString(),
    msg: String(msg),
    body: formatPayload(payload),
  });
  // Keep memory bounded; drop oldest first so recent context survives.
  if (logEntries.length > LOG_LIMIT) {
    logEntries.splice(0, logEntries.length - LOG_LIMIT);
  }
  renderLog();
}

async function api(path, method = 'GET', body) {
  const res = await fetch(path, {
    method,
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  });
  const data = await res.json();
  if (!res.ok) {
    throw new Error(data.detail || JSON.stringify(data));
  }
  return data;
}

async function refreshStatus() {
  try {
    applyState(await api('/status'));
  } catch (e) {
    statusEl.textContent = `Status error: ${e.message}`;
  }
}

function bind(id, fn) {
  document.getElementById(id).addEventListener('click', fn);
}

function showScreen(screen) {
  const showControl = screen === 'control';
  screenControlEl.classList.toggle('active', showControl);
  screenPlanEl.classList.toggle('active', !showControl);
  showControlBtn.classList.toggle('active', showControl);
  showPlanBtn.classList.toggle('active', !showControl);
  if (!showControl && planMap) {
    setTimeout(() => planMap.invalidateSize(), 100);
  }
}

showControlBtn.addEventListener('click', () => showScreen('control'));
showPlanBtn.addEventListener('click', () => showScreen('plan'));

bind('connectBtn', async () => {
  try {
    const payload = {
      connection: connectionEl.value.trim(),
      baud: Number(baudEl.value || 115200),
    };
    const r = await api('/connect', 'POST', payload);
    log('Connected', r);
    await refreshStatus();
  } catch (e) {
    log(`Connect failed: ${e.message}`);
  }
});

bind('armBtn', () => command('/command/arm', { arm: true }, 'Arm'));
bind('disarmBtn', () => command('/command/arm', { arm: false }, 'Disarm'));
bind('guidedBtn', () => command('/command/mode', { mode: 'GUIDED' }, 'Mode GUIDED'));
bind('rtlBtn', () => command('/command/rtl', null, 'RTL'));

bind('takeoffBtn', () => command('/command/takeoff', {
  altitude: Number(takeoffAltEl.value || 20),
}, 'Takeoff'));

bind('gotoBtn', () => command('/command/goto', {
  lat: Number(gotoLatEl.value),
  lon: Number(gotoLonEl.value),
  altitude: Number(gotoAltEl.value || 20),
}, 'Goto'));

async function command(path, body, label) {
  try {
    const r = await api(path, 'POST', body || undefined);
    log(`${label} result`, r);
    await refreshStatus();
  } catch (e) {
    log(`${label} failed: ${e.message}`);
  }
}

function waypointPopup(wp, index) {
  return `
<strong>Point ${index + 1}</strong><br/>
Latitude: ${wp.lat.toFixed(7)}<br/>
Longitude: ${wp.lon.toFixed(7)}<br/>
Altitude: ${wp.alt.toFixed(2)} m
`;
}

function syncMarkers() {
  if (!planMap || !markerLayer) {
    return;
  }
  markerLayer.clearLayers();
  waypoints.forEach((wp, index) => {
    const marker = L.marker([wp.lat, wp.lon], {
      draggable: true,
      keyboard: false,
    });
    marker.bindPopup(waypointPopup(wp, index));
    marker.on('click', () => {
      selectedWaypointIndex = index;
      renderWaypointTable();
      marker.openPopup();
    });
    marker.on('mouseover', () => marker.openPopup());
    marker.on('mouseout', () => marker.closePopup());
    marker.on('dragend', (ev) => {
      const { lat, lng } = ev.target.getLatLng();
      waypoints[index].lat = lat;
      waypoints[index].lon = lng;
      renderPlan();
    });
    marker.addTo(markerLayer);
  });
}

function syncHomeMarker() {
  if (!planMap) {
    return;
  }
  if (!homePosition) {
    if (homeMarker) {
      homeMarker.remove();
      homeMarker = null;
    }
    return;
  }
  const latlng = [homePosition.lat, homePosition.lon];
  if (!homeMarker) {
    homeMarker = L.circleMarker(latlng, {
      radius: 7,
      color: '#7dd3fc',
      fillColor: '#0ea5e9',
      fillOpacity: 1,
      weight: 2,
    }).addTo(planMap);
    homeMarker.bindTooltip('Home (vehicle position)', { direction: 'top' });
  } else {
    homeMarker.setLatLng(latlng);
  }
}

// Dotted path runs home -> wp1 -> wp2 -> ...; home is only a leg when known.
function syncPath() {
  if (!planMap || !pathLayer) {
    return;
  }
  pathLayer.clearLayers();
  const legs = waypoints.map((wp) => [wp.lat, wp.lon]);
  if (homePosition) {
    legs.unshift([homePosition.lat, homePosition.lon]);
  }
  if (legs.length < 2) {
    return;
  }
  L.polyline(legs, {
    color: '#7dd3fc',
    weight: 2,
    opacity: 0.9,
    dashArray: '6 8',
  }).addTo(pathLayer);
}

function setHomePosition(lat, lon) {
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
    return;
  }
  // The backend reports 0/0 before the vehicle has a GPS fix.
  if (Math.abs(lat) < 1e-8 && Math.abs(lon) < 1e-8) {
    return;
  }
  homePosition = { lat, lon };
  syncHomeMarker();
  syncPath();
  if (planMap && !homeCentered) {
    homeCentered = true;
    planMap.setView([lat, lon], 17);
  }
}

function applyState(state) {
  statusEl.textContent = JSON.stringify(state, null, 2);
  if (state && typeof state === 'object') {
    setHomePosition(Number(state.lat), Number(state.lon));
  }
}

function renderWaypointTable() {
  waypointTableBody.innerHTML = '';
  waypoints.forEach((wp, index) => {
    const tr = document.createElement('tr');
    tr.className = index === selectedWaypointIndex ? 'selected' : '';
    tr.innerHTML = `
      <td>${index + 1}</td>
      <td><input data-index="${index}" data-field="lat" type="number" step="any" value="${wp.lat.toFixed(7)}" /></td>
      <td><input data-index="${index}" data-field="lon" type="number" step="any" value="${wp.lon.toFixed(7)}" /></td>
      <td><input data-index="${index}" data-field="alt" type="number" step="any" value="${wp.alt.toFixed(2)}" /></td>
      <td><button type="button" data-remove="${index}">Remove</button></td>
    `;
    tr.addEventListener('click', () => {
      selectedWaypointIndex = index;
      renderWaypointTable();
    });
    waypointTableBody.appendChild(tr);
  });
  removeSelectedBtn.disabled = selectedWaypointIndex == null;
}

function renderPlan() {
  syncMarkers();
  syncPath();
  renderWaypointTable();
  addPointModeBtn.textContent = `Add Point Mode: ${addPointMode ? 'ON' : 'OFF'}`;
  addPointModeBtn.classList.toggle('active', addPointMode);
}

function addWaypoint(lat, lon, alt = 20) {
  waypoints.push({ lat, lon, alt });
  selectedWaypointIndex = waypoints.length - 1;
  renderPlan();
}

function removeWaypoint(index) {
  if (index == null || index < 0 || index >= waypoints.length) {
    return;
  }
  waypoints.splice(index, 1);
  if (waypoints.length === 0) {
    selectedWaypointIndex = null;
  } else if (selectedWaypointIndex >= waypoints.length) {
    selectedWaypointIndex = waypoints.length - 1;
  }
  renderPlan();
}

function initPlanMap() {
  if (!window.L) {
    log('Leaflet failed to load; map is unavailable.');
    return;
  }
  planMap = L.map('planMap', { zoomControl: true }).setView([20, 0], 2);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; OpenStreetMap contributors',
  }).addTo(planMap);
  pathLayer = L.layerGroup().addTo(planMap);
  markerLayer = L.layerGroup().addTo(planMap);
  syncHomeMarker();
  if (homePosition && !homeCentered) {
    homeCentered = true;
    planMap.setView([homePosition.lat, homePosition.lon], 17);
  }
  planMap.on('click', (ev) => {
    if (!addPointMode) {
      return;
    }
    addWaypoint(ev.latlng.lat, ev.latlng.lng, 20);
  });
}

addPointModeBtn.addEventListener('click', () => {
  addPointMode = !addPointMode;
  renderPlan();
});

removeSelectedBtn.addEventListener('click', () => removeWaypoint(selectedWaypointIndex));
clearLocalPlanBtn.addEventListener('click', () => {
  waypoints = [];
  selectedWaypointIndex = null;
  renderPlan();
});

waypointTableBody.addEventListener('input', (ev) => {
  const input = ev.target;
  if (!(input instanceof HTMLInputElement)) {
    return;
  }
  const index = Number(input.dataset.index);
  const field = input.dataset.field;
  if (Number.isNaN(index) || !field || !waypoints[index]) {
    return;
  }
  const value = Number(input.value);
  if (Number.isNaN(value)) {
    return;
  }
  if (field === 'lat') {
    waypoints[index].lat = Math.max(-90, Math.min(90, value));
  } else if (field === 'lon') {
    waypoints[index].lon = Math.max(-180, Math.min(180, value));
  } else if (field === 'alt') {
    waypoints[index].alt = Math.max(0.1, value);
  }
  renderPlan();
});

waypointTableBody.addEventListener('click', (ev) => {
  const button = ev.target;
  if (!(button instanceof HTMLButtonElement)) {
    return;
  }
  if (button.dataset.remove != null) {
    removeWaypoint(Number(button.dataset.remove));
  }
});

loadMissionBtn.addEventListener('click', async () => {
  try {
    const r = await api('/mission');
    const items = Array.isArray(r.items) ? r.items : [];
    const parsed = items
      .filter((it) => Number.isFinite(it.lat) && Number.isFinite(it.lon) && Number.isFinite(it.alt))
      .map((it) => ({ lat: Number(it.lat), lon: Number(it.lon), alt: Number(it.alt) }))
      .filter((it, idx) => !(idx === 0 && Math.abs(it.lat) < 1e-8 && Math.abs(it.lon) < 1e-8 && Math.abs(it.alt) < 1e-8));
    waypoints = parsed;
    selectedWaypointIndex = waypoints.length ? 0 : null;
    renderPlan();
    log('Mission loaded', { count: waypoints.length });
  } catch (e) {
    log(`Load mission failed: ${e.message}`);
  }
});

uploadMissionBtn.addEventListener('click', async () => {
  try {
    if (!waypoints.length) {
      log('Upload mission skipped: no waypoints.');
      return;
    }
    const body = {
      waypoints: waypoints.map((wp) => ({
        lat: wp.lat,
        lon: wp.lon,
        alt: wp.alt,
      })),
    };
    const r = await api('/mission', 'POST', body);
    log('Mission uploaded', r);
  } catch (e) {
    log(`Upload mission failed: ${e.message}`);
  }
});

clearMissionBtn.addEventListener('click', async () => {
  try {
    const r = await api('/mission', 'DELETE');
    waypoints = [];
    selectedWaypointIndex = null;
    renderPlan();
    log('Mission cleared on vehicle', r);
  } catch (e) {
    log(`Clear mission failed: ${e.message}`);
  }
});

const NAV_WAYPOINT = 16;
const FRAME_GLOBAL = 0;
const FRAME_GLOBAL_RELATIVE_ALT = 3;

function parseWaypointFile(text) {
  const lines = text.split(/\r?\n/).map((l) => l.trim()).filter(Boolean);
  if (!lines.length || !/^QGC WPL 1\d\d$/i.test(lines[0])) {
    throw new Error('not a QGC WPL 110 file');
  }
  const parsed = [];
  lines.slice(1).forEach((line) => {
    const cols = line.split(/\s+/);
    if (cols.length < 12) {
      return;
    }
    const index = Number(cols[0]);
    const command = Number(cols[3]);
    const lat = Number(cols[8]);
    const lon = Number(cols[9]);
    const alt = Number(cols[10]);
    // Index 0 is the home position by QGC WPL convention, not a plan waypoint.
    if (index === 0 || command !== NAV_WAYPOINT) {
      return;
    }
    if (!Number.isFinite(lat) || !Number.isFinite(lon) || !Number.isFinite(alt)) {
      return;
    }
    parsed.push({
      lat: Math.max(-90, Math.min(90, lat)),
      lon: Math.max(-180, Math.min(180, lon)),
      alt: Math.max(0.1, alt),
    });
  });
  if (!parsed.length) {
    throw new Error('no waypoints found');
  }
  return parsed;
}

function serializeWaypointFile(points) {
  const row = (index, current, frame, lat, lon, alt) => [
    index, current, frame, NAV_WAYPOINT, 0, 0, 0, 0,
    lat.toFixed(8), lon.toFixed(8), alt.toFixed(6), 1,
  ].join('\t');
  // Prefer the vehicle's reported position for home; fall back to the first
  // waypoint so the row is never bogus 0/0.
  const home = homePosition || points[0];
  const rows = [row(0, 1, FRAME_GLOBAL, home.lat, home.lon, 0)];
  points.forEach((wp, i) => {
    rows.push(row(i + 1, 0, FRAME_GLOBAL_RELATIVE_ALT, wp.lat, wp.lon, wp.alt));
  });
  return `QGC WPL 110\n${rows.join('\n')}\n`;
}

readFileBtn.addEventListener('click', () => missionFileInput.click());

missionFileInput.addEventListener('change', async () => {
  const file = missionFileInput.files && missionFileInput.files[0];
  if (!file) {
    return;
  }
  try {
    waypoints = parseWaypointFile(await file.text());
    selectedWaypointIndex = 0;
    renderPlan();
    if (planMap) {
      planMap.fitBounds(waypoints.map((wp) => [wp.lat, wp.lon]), { padding: [40, 40] });
    }
    log('Mission file read', { file: file.name, count: waypoints.length });
  } catch (e) {
    log(`Read file failed: ${e.message}`);
  } finally {
    missionFileInput.value = '';
  }
});

writeFileBtn.addEventListener('click', () => {
  if (!waypoints.length) {
    log('Write file skipped: no waypoints.');
    return;
  }
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  const url = URL.createObjectURL(
    new Blob([serializeWaypointFile(waypoints)], { type: 'text/plain' }),
  );
  const a = document.createElement('a');
  a.href = url;
  a.download = `mission-${stamp}.waypoints`;
  a.click();
  URL.revokeObjectURL(url);
  log('Mission file written', { file: a.download, count: waypoints.length });
});

logFilterEl.addEventListener('input', renderLog);

document.getElementById('clearLogBtn').addEventListener('click', () => {
  logEntries.length = 0;
  renderLog();
});

document.getElementById('copyLogBtn').addEventListener('click', async () => {
  try {
    await navigator.clipboard.writeText(logEntries.map(formatEntry).join('\n\n'));
    log('Logs copied to clipboard', { count: logEntries.length });
  } catch (e) {
    log(`Copy logs failed: ${e.message}`);
  }
});

document.getElementById('downloadLogBtn').addEventListener('click', () => {
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  const url = URL.createObjectURL(
    new Blob([logEntries.map(formatEntry).join('\n\n')], { type: 'text/plain' }),
  );
  const a = document.createElement('a');
  a.href = url;
  a.download = `gcs-log-${stamp}.txt`;
  a.click();
  URL.revokeObjectURL(url);
});

// Surface failures that would otherwise only reach the devtools console.
window.addEventListener('error', (ev) => {
  log(`Uncaught error: ${ev.message}`, { source: ev.filename, line: ev.lineno });
});
window.addEventListener('unhandledrejection', (ev) => {
  log(`Unhandled rejection: ${(ev.reason && ev.reason.message) || ev.reason}`);
});

function connectTelemetry() {
  const proto = window.location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${window.location.host}/ws/telemetry`);
  ws.onmessage = (ev) => {
    try {
      applyState(JSON.parse(ev.data));
    } catch {
      statusEl.textContent = ev.data;
    }
  };
  ws.onopen = () => log('Telemetry connected');
  ws.onerror = () => log('Telemetry error');
  ws.onclose = () => {
    log('Telemetry disconnected; retrying...');
    setTimeout(connectTelemetry, 2000);
  };
}

refreshStatus();
setInterval(refreshStatus, 4000);
connectTelemetry();
initPlanMap();
renderPlan();
