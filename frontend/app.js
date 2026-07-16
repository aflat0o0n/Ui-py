const statusEl = document.getElementById('status');
const logEl = document.getElementById('log');

const connectionEl = document.getElementById('connection');
const baudEl = document.getElementById('baud');
const takeoffAltEl = document.getElementById('takeoffAlt');
const gotoLatEl = document.getElementById('gotoLat');
const gotoLonEl = document.getElementById('gotoLon');
const gotoAltEl = document.getElementById('gotoAlt');

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
