const statusEl = document.getElementById('status');
const logEl = document.getElementById('log');
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
const waypointTableBody = document.getElementById('waypointTableBody');

let planMap;
let markerLayer;
let addPointMode = true;
let selectedWaypointIndex = null;
let waypoints = [];

function log(msg, payload) {
  const line = `[${new Date().toISOString()}] ${msg}`;
  const body = payload ? `\n${JSON.stringify(payload, null, 2)}` : '';
  logEl.textContent = `${line}${body}\n\n${logEl.textContent}`.slice(0, 8000);
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
    const s = await api('/status');
    statusEl.textContent = JSON.stringify(s, null, 2);
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
  markerLayer = L.layerGroup().addTo(planMap);
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

function connectTelemetry() {
  const proto = window.location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${window.location.host}/ws/telemetry`);
  ws.onmessage = (ev) => {
    try {
      statusEl.textContent = JSON.stringify(JSON.parse(ev.data), null, 2);
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
