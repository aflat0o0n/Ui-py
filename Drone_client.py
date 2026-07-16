"""
drone_client.py — drop-in PyQt6 client for the GCS backend.

Give this to the GUI team. It handles all HTTP/WebSocket plumbing on
background threads and exposes everything through Qt signals, so the
GUI never freezes and never touches networking code.

Install (GUI machine):
    pip install PyQt6 requests websocket-client

Usage in any PyQt window:

    from drone_client import DroneClient

    self.drone = DroneClient()                       # once, in __init__
    self.drone.telemetry.connect(self.on_telemetry)  # 10 Hz dict
    self.drone.command_result.connect(self.on_result)
    self.drone.connection_changed.connect(self.on_conn)
    self.drone.error.connect(self.on_error)

    # wire buttons directly:
    self.connect_btn.clicked.connect(
        lambda: self.drone.connect_drone("tcp:127.0.0.1:5760"))
    self.arm_btn.clicked.connect(lambda: self.drone.arm(True))
    self.takeoff_btn.clicked.connect(lambda: self.drone.takeoff(20))
"""

import json

import requests
from PyQt6.QtCore import QObject, QThread, QRunnable, QThreadPool, pyqtSignal

BACKEND = "http://localhost:8000"


class _Job(QRunnable):
    """Runs one HTTP request off the GUI thread."""

    def __init__(self, fn):
        super().__init__()
        self.fn = fn

    def run(self):
        self.fn()


class _TelemetryThread(QThread):
    """Persistent WebSocket reader; emits state dicts at 10 Hz."""
    received = pyqtSignal(dict)
    dropped = pyqtSignal(str)

    def run(self):
        import websocket  # websocket-client
        url = BACKEND.replace("http", "ws") + "/ws/telemetry"
        try:
            ws = websocket.create_connection(url, timeout=10)
            while not self.isInterruptionRequested():
                self.received.emit(json.loads(ws.recv()))
        except Exception as e:
            self.dropped.emit(f"telemetry stream ended: {e}")


class DroneClient(QObject):
    """All drone I/O behind Qt signals. Create once, share across windows."""

    # -------- signals the GUI subscribes to --------
    telemetry = pyqtSignal(dict)            # {"lat","lon","alt_rel","mode",
                                            #  "armed","link_alive",...} 10 Hz
    connection_changed = pyqtSignal(bool)   # backend<->drone link state
    command_result = pyqtSignal(str, dict)  # ("ARM", {"accepted": True, ...})
    error = pyqtSignal(str)                 # human-readable problems

    def __init__(self, backend_url: str = BACKEND):
        super().__init__()
        global BACKEND
        BACKEND = backend_url
        self._pool = QThreadPool.globalInstance()
        self._tele: _TelemetryThread | None = None

    # -------- internal request helpers --------
    def _post(self, name: str, path: str, body: dict | None = None,
              timeout: float = 60):
        def work():
            try:
                r = requests.post(BACKEND + path, json=body, timeout=timeout)
                data = r.json()
                if r.status_code != 200:
                    self.error.emit(f"{name}: {data.get('detail', r.text)}")
                    data = {"accepted": False, "detail": data.get("detail")}
                self.command_result.emit(name, data)
            except requests.exceptions.RequestException as e:
                self.error.emit(f"{name}: backend unreachable ({e})")
        self._pool.start(_Job(work))

    def _get(self, name: str, path: str, timeout: float = 90):
        def work():
            try:
                r = requests.get(BACKEND + path, timeout=timeout)
                self.command_result.emit(name, r.json())
            except requests.exceptions.RequestException as e:
                self.error.emit(f"{name}: backend unreachable ({e})")
        self._pool.start(_Job(work))

    # -------- connection --------
    def connect_drone(self, connection: str, baud: int = 115200):
        """connection: 'tcp:127.0.0.1:5760' (SITL) or '/dev/ttyUSB0' etc."""
        def work():
            try:
                r = requests.post(BACKEND + "/connect",
                                  json={"connection": connection,
                                        "baud": baud}, timeout=25)
                ok = r.status_code == 200
                if ok:
                    self._start_telemetry()
                else:
                    self.error.emit(f"connect: {r.json().get('detail')}")
                self.connection_changed.emit(ok)
            except requests.exceptions.RequestException as e:
                self.error.emit(f"connect: backend unreachable ({e})")
                self.connection_changed.emit(False)
        self._pool.start(_Job(work))

    def _start_telemetry(self):
        if self._tele and self._tele.isRunning():
            return
        self._tele = _TelemetryThread()
        self._tele.received.connect(self.telemetry)
        self._tele.dropped.connect(self.error)
        self._tele.start()

    def shutdown(self):
        """Call from the window's closeEvent."""
        if self._tele:
            self._tele.requestInterruption()
            self._tele.wait(2000)

    # -------- flight commands (results arrive via command_result) --------
    def arm(self, arm: bool = True):
        self._post("ARM" if arm else "DISARM", "/command/arm", {"arm": arm})

    def set_mode(self, mode: str):
        self._post(f"MODE {mode}", "/command/mode", {"mode": mode})

    def takeoff(self, altitude: float):
        self._post("TAKEOFF", "/command/takeoff", {"altitude": altitude})

    def goto(self, lat: float, lon: float, altitude: float):
        self._post("GOTO", "/command/goto",
                   {"lat": lat, "lon": lon, "altitude": altitude})

    def rtl(self):
        self._post("RTL", "/command/rtl")

    # -------- missions --------
    def upload_mission(self, waypoints: list[dict]):
        """waypoints: [{"lat":.., "lon":.., "alt":..}, ...]"""
        self._post("MISSION_UPLOAD", "/mission", {"waypoints": waypoints})

    def download_mission(self):
        self._get("MISSION_DOWNLOAD", "/mission")

    def clear_mission(self):
        def work():
            try:
                r = requests.delete(BACKEND + "/mission", timeout=15)
                self.command_result.emit("MISSION_CLEAR", r.json())
            except requests.exceptions.RequestException as e:
                self.error.emit(f"MISSION_CLEAR: {e}")
        self._pool.start(_Job(work))

    def start_mission(self):
        self._post("MISSION_START", "/command/mission_start")

    # -------- parameters --------
    def fetch_all_parameters(self):
        self._get("PARAMS_ALL", "/parameters")   # slow: up to 60 s

    def get_parameter(self, name: str):
        self._get(f"PARAM {name}", f"/parameters/{name}")

    def set_parameter(self, name: str, value: float):
        def work():
            try:
                r = requests.put(BACKEND + f"/parameters/{name}",
                                 json={"value": value}, timeout=20)
                self.command_result.emit(f"PARAM_SET {name}", r.json())
            except requests.exceptions.RequestException as e:
                self.error.emit(f"PARAM_SET {name}: {e}")
        self._pool.start(_Job(work))
