"""
gcs_launcher.py — starts the whole GCS stack when the GUI launches.

The GUI calls start_all() before showing its window and stop_all() on
exit. The router (MAVProxy) and backend (uvicorn) run as hidden child
processes; the operator only ever sees the GUI.

Configuration lives in gcs_config.json next to this file:
{
    "drone_connection": "COM7",          // or "tcp:127.0.0.1:5760" for SITL
    "drone_baud": 57600,
    "backend_port": 8000,
    "gcs_port": 14551,                   // backend attaches here
    "mp_port": 14550                     // spare seat for Mission Planner
}

Windows packaging note: when frozen with PyInstaller, ship python/uvicorn
alongside or run the backend in-process; during development this launcher
uses the same interpreter that runs the GUI (sys.executable).
"""

import atexit
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import requests

HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "gcs_config.json"

_children: list[subprocess.Popen] = []


def _load_config() -> dict:
    defaults = {
        "drone_connection": "udp:127.0.0.1:14551",
        "drone_baud": 57600,
        "backend_port": 8000,
        "gcs_port": 14551,
        "mp_port": 14550,
    }
    if CONFIG_PATH.exists():
        defaults.update(json.loads(CONFIG_PATH.read_text()))
    return defaults


def _popen_hidden(cmd: list[str], env: dict | None = None) -> subprocess.Popen:
    """Start a child with no visible console window (Windows) and
    grouped for clean teardown."""
    kwargs: dict = {
        "stdout": subprocess.DEVNULL,
        "stderr": subprocess.DEVNULL,
        "env": {**os.environ, **(env or {})},
    }
    if os.name == "nt":
        kwargs["creationflags"] = (
            subprocess.CREATE_NO_WINDOW
            | subprocess.CREATE_NEW_PROCESS_GROUP)
    else:
        kwargs["start_new_session"] = True
    p = subprocess.Popen(cmd, **kwargs)
    _children.append(p)
    return p


def _backend_url(cfg) -> str:
    return f"http://127.0.0.1:{cfg['backend_port']}"


def start_all(timeout_s: float = 30) -> dict:
    """Launch router + backend; block until the backend answers /status.
    Returns the config in use. Raises RuntimeError on failure."""
    cfg = _load_config()

    # 1. Router: drone link fanned out to GCS port + spare MP seat.
    conn = cfg["drone_connection"]
    master = ([f"--master={conn}", f"--baudrate={cfg['drone_baud']}"]
              if not conn.startswith(("tcp", "udp"))
              else [f"--master={conn}"])
    _popen_hidden([sys.executable, "-m", "MAVProxy.mavproxy", *master,
                   f"--out=udp:127.0.0.1:{cfg['gcs_port']}",
                   f"--out=udp:127.0.0.1:{cfg['mp_port']}",
                   "--daemon"])

    # 2. Backend: attaches to the router's GCS port by default. Bind to
    #    0.0.0.0 so the self-hosted UI is reachable from other machines on
    #    the LAN; the health check below still probes it over loopback.
    _popen_hidden(
        [sys.executable, "-m", "uvicorn", "GCS_backend_service:app",
         "--host", "0.0.0.0", "--port", str(cfg["backend_port"])],
        env={"GCS_CONNECTION": f"udp:127.0.0.1:{cfg['gcs_port']}"})

    # 3. Wait until the backend is answering.
    deadline = time.time() + timeout_s
    url = _backend_url(cfg) + "/status"
    while time.time() < deadline:
        try:
            requests.get(url, timeout=2)
            return cfg
        except requests.exceptions.RequestException:
            time.sleep(0.5)
    stop_all()
    raise RuntimeError(
        "Backend did not come up. Check that gcs_backend_service.py is "
        "in the same folder and dependencies are installed.")


def stop_all():
    """Terminate the stack. Safe to call more than once."""
    for p in _children:
        if p.poll() is None:
            p.terminate()
    deadline = time.time() + 5
    for p in _children:
        while p.poll() is None and time.time() < deadline:
            time.sleep(0.1)
        if p.poll() is None:
            p.kill()
    _children.clear()


atexit.register(stop_all)


if __name__ == "__main__":
    cfg = start_all()
    print("stack up:", cfg)
    print("Ctrl+C to stop")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_all()
