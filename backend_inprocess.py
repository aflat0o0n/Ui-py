"""
backend_inprocess.py — run the GCS backend inside the GUI process.

For packaging as a single executable: instead of spawning uvicorn as a child
process (which breaks under PyInstaller because sys.executable becomes the
frozen app, not Python), this starts the same FastAPI app on a daemon thread
in the GUI process. One process, nothing to spawn.

Host/port come from gcs_config.json next to the app (or defaults):
    { "backend_host": "0.0.0.0", "backend_port": 8000 }

backend_host "0.0.0.0" makes the API, /panel, and telemetry reachable from
other devices (a phone, a tablet, or Mission Planner on another PC). Use
"127.0.0.1" to keep it local-only.

The drone connection itself is separate — set it via the connect dialog at
runtime, or GCS_CONNECTION / gcs_config.json "drone_connection".
"""

import json
import os
import socket
import threading
import time
from pathlib import Path

import requests
import uvicorn

_server = None
_actual_port = None
_actual_host = None


def _config() -> dict:
    p = Path(__file__).with_name("gcs_config.json")
    if p.exists():
        try:
            return json.loads(p.read_text())
        except Exception:
            pass
    return {}


def _port_is_free(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((host if host != "0.0.0.0" else "", port))
            return True
        except OSError:
            return False


def _pick_port(host: str, preferred: int) -> int:
    if _port_is_free(host, preferred):
        return preferred
    # preferred busy: try a small range, then any free port
    for p in range(preferred + 1, preferred + 20):
        if _port_is_free(host, p):
            return p
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host if host != "0.0.0.0" else "", 0))
        return s.getsockname()[1]


def start_backend(connection: str | None = None,
                  baud: int | None = None,
                  host: str | None = None,
                  port: int | None = None,
                  wait_timeout: float = 15) -> dict:
    """Start the backend on a background thread; block until it answers.
    Returns {"host","port","url"} — the URL the GUI client should use.
    Explicit args override gcs_config.json which overrides the defaults."""
    global _server, _actual_port, _actual_host
    cfg = _config()

    if connection:
        os.environ["GCS_CONNECTION"] = connection
    elif cfg.get("drone_connection"):
        os.environ["GCS_CONNECTION"] = cfg["drone_connection"]
    if baud:
        os.environ["GCS_BAUD"] = str(baud)
    elif cfg.get("drone_baud"):
        os.environ["GCS_BAUD"] = str(cfg["drone_baud"])

    host = host or cfg.get("backend_host", "0.0.0.0")
    preferred = port or cfg.get("backend_port", 8000)
    port = _pick_port(host, preferred)

    _actual_host, _actual_port = host, port

    # import after env is set so module-level defaults pick it up
    try:
        import GCS_backend_service as backend_service
    except ImportError:
        import gcs_backend_service as backend_service

    config = uvicorn.Config(backend_service.app, host=host, port=port,
                            log_level="warning")
    _server = uvicorn.Server(config)
    threading.Thread(target=_server.run, daemon=True).start()

    # the GUI on the same machine always reaches the backend via loopback,
    # even when the server is bound to 0.0.0.0
    client_host = "127.0.0.1"
    deadline = time.time() + wait_timeout
    url = f"http://{client_host}:{port}/status"
    while time.time() < deadline:
        try:
            requests.get(url, timeout=1)
            return {"host": host, "port": port,
                    "url": f"http://{client_host}:{port}"}
        except requests.exceptions.RequestException:
            time.sleep(0.3)
    raise RuntimeError(f"Backend did not start on {host}:{port} in time")


def backend_url() -> str:
    """LAN URL others (phone, MP PC) can use — reports the real bound host."""
    return f"http://{_actual_host}:{_actual_port}"


def stop_backend() -> None:
    if _server:
        _server.should_exit = True


if __name__ == "__main__":
    info = start_backend(connection="tcp:127.0.0.1:5760")
    print("backend up:", info)
    print("GET /status ->",
          requests.get(info["url"] + "/status").json()["connected"])
    time.sleep(1)
    stop_backend()
    print("stopped")
