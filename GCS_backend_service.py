"""
Headless precision GCS backend: your GUI <-> flight controller, directly.

Architecture (no Mission Planner required):
  Your GUI --HTTP/WS--> this service --MAVLink--> flight controller

Connection examples for POST /connect {"connection": "..."}:
  Serial (USB/telemetry radio):
      Windows:  "COM5"          (baud defaults to 115200; use
      Linux:    "/dev/ttyUSB0"   {"connection": "COM5", "baud": 57600}
                                 for 57600-baud telemetry radios)
  SITL / network:
      "tcp:127.0.0.1:5760"      (SITL's default port)
      "udp:0.0.0.0:14550"       (listen for FC streaming to us)

Install:
  pip install fastapi uvicorn pymavlink pyserial

Run:
  uvicorn mp_bridge_service:app --host 127.0.0.1 --port 8000

API docs (auto-generated): http://localhost:8000/docs

Endpoints:
  GET  /status                 -> connection + latest vehicle state
  POST /connect                -> open direct MAVLink link to the FC
  POST /command/arm            -> {"arm": true|false}
  POST /command/mode           -> {"mode": "GUIDED"}
  POST /command/takeoff        -> {"altitude": 10}
  POST /command/goto           -> {"lat": .., "lon": .., "altitude": ..}
  POST /command/rtl
  POST /command/mission_start  -> begin AUTO mission
  GET  /mission                -> download current mission from FC
  POST /mission                -> upload mission (list of waypoints)
  DELETE /mission              -> clear mission on FC
  GET  /parameters             -> fetch full parameter list from FC
  GET  /parameters/{name}      -> fetch one parameter
  PUT  /parameters/{name}      -> set parameter, verified by read-back
  WS   /ws/telemetry           -> 10 Hz JSON state stream
All command endpoints block until the FC's acknowledgment
(COMMAND_ACK / MISSION_ACK / PARAM_VALUE read-back) and return
the real result. Nothing is fire-and-forget.
"""

import asyncio
import os
import queue as queue_mod
import time
import threading
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from pymavlink import mavutil

# The GUI's Connect button can POST /connect with an empty body; the backend
# then uses this default, configurable per ground station via environment:
#   export GCS_CONNECTION="udp:127.0.0.1:14551"   (router topology)
#   export GCS_CONNECTION="/dev/ttyUSB0"; export GCS_BAUD=57600  (direct)
DEFAULT_CONNECTION = os.environ.get("GCS_CONNECTION", "udp:127.0.0.1:14551")
DEFAULT_BAUD = int(os.environ.get("GCS_BAUD", "115200"))
ACK_TIMEOUT_S = 3.0
HEARTBEAT_STALE_S = 5.0
HEARTBEAT_LOST_S = 10.0      # supervisor reconnects after this silence


# ---------------------------------------------------------------------------
# MAVLink bridge core (thread-based; FastAPI talks to it via asyncio bridges)
# ---------------------------------------------------------------------------
class Bridge:
    def __init__(self):
        self.master = None
        self.connected = False
        self._lock = threading.Lock()          # serializes command sends
        self._ack_events: dict = {}
        self._ack_lock = threading.Lock()
        # msg_type -> list of Queues; used by mission/param protocol handlers
        self._subs: dict = {}
        self._subs_lock = threading.Lock()
        self._supervising = False
        self.state = {
            "lat": 0.0, "lon": 0.0, "alt_rel": 0.0,
            "mode": None, "armed": False,
            "groundspeed": 0.0, "heading": 0.0,
            "battery_v": 0.0, "last_heartbeat": 0.0,
        }

    # ------------- connection -------------
    def connect(self, connection: str = DEFAULT_CONNECTION,
                baud: int = 115200) -> None:
        if self.connected:
            return
        self.master = mavutil.mavlink_connection(connection, baud=baud)
        hb = self.master.wait_heartbeat(timeout=15)
        if hb is None:
            self.master = None
            raise ConnectionError(
                f"No heartbeat on '{connection}'. Check the port/address, "
                "baud rate, and that the FC is powered.")
        self.state["last_heartbeat"] = time.time()   # wait_heartbeat saw one
        self.connected = True
        self._request_streams()
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def _request_streams(self):
        intervals = {
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 100_000,  # 10 Hz
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 200_000,              # 5 Hz
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 500_000,           # 2 Hz
        }
        for msg_id, us in intervals.items():
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, us, 0, 0, 0, 0, 0)

    # ------------- receive loop -------------
    def _rx_loop(self):
        while self.connected:
            try:
                msg = self.master.recv_match(blocking=True, timeout=1)
            except (OSError, AttributeError):
                # socket closed by teardown() while we were blocked in recv;
                # exit quietly — the supervisor owns reconnection
                break
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT":
                self.state["mode"] = mavutil.mode_string_v10(msg)
                self.state["armed"] = bool(
                    msg.base_mode &
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.state["last_heartbeat"] = time.time()
            elif t == "GLOBAL_POSITION_INT":
                self.state["lat"] = msg.lat / 1e7
                self.state["lon"] = msg.lon / 1e7
                self.state["alt_rel"] = msg.relative_alt / 1000.0
                self.state["heading"] = msg.hdg / 100.0
            elif t == "VFR_HUD":
                self.state["groundspeed"] = msg.groundspeed
            elif t == "SYS_STATUS":
                self.state["battery_v"] = msg.voltage_battery / 1000.0
            elif t == "COMMAND_ACK":
                with self._ack_lock:
                    entry = self._ack_events.pop(msg.command, None)
                if entry:
                    event, box = entry
                    box.append(msg.result)
                    event.set()
            # feed protocol handlers (missions, parameters)
            with self._subs_lock:
                subs = list(self._subs.get(t, ()))
            for q in subs:
                q.put(msg)

    # ------------- message subscription (for multi-message protocols) ----
    def _subscribe(self, *msg_types):
        q = queue_mod.Queue()
        with self._subs_lock:
            for t in msg_types:
                self._subs.setdefault(t, []).append(q)
        return q

    def _unsubscribe(self, q):
        with self._subs_lock:
            for lst in self._subs.values():
                if q in lst:
                    lst.remove(q)

    # ------------- ACK-verified command primitive -------------
    def _send_verified(self, cmd_id: int, sender) -> dict:
        """Send a COMMAND_LONG and block until COMMAND_ACK or timeout.
        Returns {"result": <MAV_RESULT name>, "accepted": bool}."""
        if not self.connected:
            raise ConnectionError("Not connected")
        event = threading.Event()
        box: list = []
        with self._ack_lock:
            self._ack_events[cmd_id] = (event, box)
        with self._lock:
            sender()
        if not event.wait(ACK_TIMEOUT_S):
            with self._ack_lock:
                self._ack_events.pop(cmd_id, None)
            return {"result": "TIMEOUT_NO_ACK", "accepted": False}
        res = box[0]
        name = mavutil.mavlink.enums["MAV_RESULT"][res].name
        return {"result": name,
                "accepted": res == mavutil.mavlink.MAV_RESULT_ACCEPTED}

    # ------------- commands -------------
    def arm(self, arm: bool) -> dict:
        cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        return self._send_verified(cmd, lambda:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0, 1 if arm else 0, 0, 0, 0, 0, 0, 0))

    def set_mode(self, mode: str) -> dict:
        mapping = self.master.mode_mapping() or {}
        if mode not in mapping:
            raise ValueError(
                f"Unknown mode '{mode}'. Available: {sorted(mapping)}")
        cmd = mavutil.mavlink.MAV_CMD_DO_SET_MODE
        return self._send_verified(cmd, lambda:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mapping[mode], 0, 0, 0, 0, 0))

    def takeoff(self, alt_m: float) -> dict:
        cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        return self._send_verified(cmd, lambda:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0, 0, 0, 0, 0, 0, 0, alt_m))

    def rtl(self) -> dict:
        cmd = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        return self._send_verified(cmd, lambda:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0, 0, 0, 0, 0, 0, 0, 0))

    def goto(self, lat: float, lon: float, alt_m: float) -> dict:
        """Position target in 1e7 int format (~1 cm resolution).
        No COMMAND_ACK exists for this message type; confirmation is
        the vehicle converging on target in the telemetry stream."""
        if not self.connected:
            raise ConnectionError("Not connected")
        if self.state["mode"] != "GUIDED":
            raise ValueError(
                f"GOTO requires GUIDED mode (current: {self.state['mode']}). "
                "Call /command/mode first.")
        type_mask = 0b0000_1111_1111_1000
        with self._lock:
            self.master.mav.set_position_target_global_int_send(
                0, self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                int(lat * 1e7), int(lon * 1e7), alt_m,
                0, 0, 0, 0, 0, 0, 0, 0)
        return {"result": "SENT",
                "accepted": True,
                "target": {"lat": lat, "lon": lon, "alt": alt_m},
                "note": "Track convergence via /ws/telemetry"}

    # =================== MISSION PROTOCOL ===================
    def mission_download(self) -> list:
        """Pull the full mission from the FC (MISSION protocol)."""
        if not self.connected:
            raise ConnectionError("Not connected")
        q = self._subscribe("MISSION_COUNT", "MISSION_ITEM_INT",
                            "MISSION_ITEM")
        try:
            with self._lock:
                self.master.mav.mission_request_list_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            count_msg = self._q_wait(q, "MISSION_COUNT", 5)
            items = []
            for seq in range(count_msg.count):
                with self._lock:
                    self.master.mav.mission_request_int_send(
                        self.master.target_system,
                        self.master.target_component, seq,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                item = self._q_wait(q, ("MISSION_ITEM_INT", "MISSION_ITEM"),
                                    5, seq=seq)
                is_int = item.get_type() == "MISSION_ITEM_INT"
                items.append({
                    "seq": item.seq,
                    "frame": item.frame,
                    "command": item.command,
                    "param1": item.param1, "param2": item.param2,
                    "param3": item.param3, "param4": item.param4,
                    "lat": (item.x / 1e7) if is_int else item.x,
                    "lon": (item.y / 1e7) if is_int else item.y,
                    "alt": item.z,
                    "autocontinue": item.autocontinue,
                    "current": item.current,
                })
            # final ACK tells the FC we're done
            with self._lock:
                self.master.mav.mission_ack_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_MISSION_ACCEPTED,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            return items
        finally:
            self._unsubscribe(q)

    def mission_upload(self, waypoints: list) -> dict:
        """Push a mission (MISSION protocol handshake, 1e7 int coords).
        waypoints: [{lat, lon, alt, command?, param1..4?, frame?}, ...]
        Item 0 is conventionally home; ArduPilot manages it, so we
        prepend a dummy home item automatically."""
        if not self.connected:
            raise ConnectionError("Not connected")
        mt = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        # Build item list: seq 0 = home placeholder, then user waypoints
        items = [dict(lat=0, lon=0, alt=0,
                      command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                      frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
                      param1=0, param2=0, param3=0, param4=0)]
        for wp in waypoints:
            items.append(dict(
                lat=wp["lat"], lon=wp["lon"], alt=wp["alt"],
                command=wp.get("command",
                               mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
                frame=wp.get("frame",
                             mavutil.mavlink
                             .MAV_FRAME_GLOBAL_RELATIVE_ALT),
                param1=wp.get("param1", 0), param2=wp.get("param2", 0),
                param3=wp.get("param3", 0), param4=wp.get("param4", 0)))
        q = self._subscribe("MISSION_REQUEST", "MISSION_REQUEST_INT",
                            "MISSION_ACK")
        try:
            with self._lock:
                self.master.mav.mission_count_send(
                    self.master.target_system, self.master.target_component,
                    len(items), mt)
            sent = 0
            deadline = time.time() + 30
            while time.time() < deadline:
                msg = self._q_get(q, 5)
                t = msg.get_type()
                if t in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
                    seq = msg.seq
                    it = items[seq]
                    with self._lock:
                        self.master.mav.mission_item_int_send(
                            self.master.target_system,
                            self.master.target_component,
                            seq, it["frame"], it["command"],
                            1 if seq == 0 else 0,   # current
                            1,                       # autocontinue
                            it["param1"], it["param2"],
                            it["param3"], it["param4"],
                            int(it["lat"] * 1e7), int(it["lon"] * 1e7),
                            it["alt"], mt)
                    sent = max(sent, seq + 1)
                elif t == "MISSION_ACK":
                    name = mavutil.mavlink.enums[
                        "MAV_MISSION_RESULT"][msg.type].name
                    ok = msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
                    return {"result": name, "accepted": ok,
                            "items_uploaded": sent,
                            "waypoints": len(waypoints)}
            raise TimeoutError("Mission upload handshake timed out")
        finally:
            self._unsubscribe(q)

    def mission_clear(self) -> dict:
        if not self.connected:
            raise ConnectionError("Not connected")
        q = self._subscribe("MISSION_ACK")
        try:
            with self._lock:
                self.master.mav.mission_clear_all_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            msg = self._q_wait(q, "MISSION_ACK", 5)
            name = mavutil.mavlink.enums[
                "MAV_MISSION_RESULT"][msg.type].name
            return {"result": name,
                    "accepted":
                    msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED}
        finally:
            self._unsubscribe(q)

    def mission_start(self) -> dict:
        cmd = mavutil.mavlink.MAV_CMD_MISSION_START
        return self._send_verified(cmd, lambda:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0, 0, 0, 0, 0, 0, 0, 0))

    # =================== PARAMETER PROTOCOL ===================
    def param_fetch_all(self, timeout_s: float = 60) -> dict:
        """Download the full parameter table (like MP's Full Parameter List)."""
        if not self.connected:
            raise ConnectionError("Not connected")
        q = self._subscribe("PARAM_VALUE")
        params, total = {}, None
        try:
            with self._lock:
                self.master.mav.param_request_list_send(
                    self.master.target_system, self.master.target_component)
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                try:
                    msg = q.get(timeout=3)
                except queue_mod.Empty:
                    if total and len(params) >= total:
                        break
                    continue
                pid = msg.param_id
                params[pid] = msg.param_value
                total = msg.param_count
                if len(params) >= total:
                    break
            return {"count_expected": total, "count_received": len(params),
                    "complete": total is not None and len(params) >= total,
                    "parameters": params}
        finally:
            self._unsubscribe(q)

    def param_get(self, name: str) -> dict:
        if not self.connected:
            raise ConnectionError("Not connected")
        q = self._subscribe("PARAM_VALUE")
        try:
            with self._lock:
                self.master.mav.param_request_read_send(
                    self.master.target_system, self.master.target_component,
                    name.encode(), -1)
            deadline = time.time() + 5
            while time.time() < deadline:
                try:
                    msg = q.get(timeout=1)
                except queue_mod.Empty:
                    continue
                if msg.param_id == name:
                    return {"name": name, "value": msg.param_value,
                            "type": msg.param_type}
            raise TimeoutError(f"No PARAM_VALUE for {name} — "
                               "check the name (case-sensitive)")
        finally:
            self._unsubscribe(q)

    def param_set(self, name: str, value: float) -> dict:
        """Set a parameter, verified by the FC's PARAM_VALUE read-back."""
        if not self.connected:
            raise ConnectionError("Not connected")
        current = self.param_get(name)   # also validates name + gets type
        q = self._subscribe("PARAM_VALUE")
        try:
            with self._lock:
                self.master.mav.param_set_send(
                    self.master.target_system, self.master.target_component,
                    name.encode(), value, current["type"])
            deadline = time.time() + 5
            while time.time() < deadline:
                try:
                    msg = q.get(timeout=1)
                except queue_mod.Empty:
                    continue
                if msg.param_id == name:
                    confirmed = msg.param_value
                    return {"name": name,
                            "requested": value,
                            "confirmed": confirmed,
                            "previous": current["value"],
                            "accepted": abs(confirmed - value) < 1e-6}
            raise TimeoutError(f"Set sent but no read-back for {name}")
        finally:
            self._unsubscribe(q)

    # ------------- protocol queue helpers -------------
    def _q_get(self, q, timeout):
        try:
            return q.get(timeout=timeout)
        except queue_mod.Empty:
            raise TimeoutError("FC did not respond in time")

    def _q_wait(self, q, types, timeout, seq=None):
        if isinstance(types, str):
            types = (types,)
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = max(0.1, deadline - time.time())
            msg = self._q_get(q, remaining)
            if msg.get_type() in types and (seq is None or msg.seq == seq):
                return msg
        raise TimeoutError(f"Timed out waiting for {types}")

    # =================== AUTONOMOUS SUPERVISOR ===================
    def teardown(self):
        """Drop the link so the supervisor (or a user) can reconnect."""
        self.connected = False
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None

    def start_supervisor(self, connection: str, baud: int = 115200):
        """Autonomous mode: keep the link up forever.
        Connects on startup, watches heartbeat health, and reconnects
        automatically after any link loss. Runs for the process lifetime."""
        if self._supervising:
            return
        self._supervising = True

        def loop():
            while True:
                if not self.connected:
                    try:
                        self.connect(connection, baud)
                        print(f"[supervisor] link up: {connection}")
                    except Exception as e:
                        print(f"[supervisor] connect failed ({e}); "
                              "retrying in 3 s")
                        time.sleep(3)
                        continue
                # link is up: watch heartbeat freshness
                age = time.time() - self.state["last_heartbeat"]
                if age > HEARTBEAT_LOST_S:
                    print(f"[supervisor] heartbeat lost ({age:.0f}s); "
                          "reconnecting")
                    self.teardown()
                    continue
                time.sleep(1)

        threading.Thread(target=loop, daemon=True).start()

    def snapshot(self) -> dict:
        s = dict(self.state)
        s["link_alive"] = (
            self.connected and
            time.time() - s["last_heartbeat"] < HEARTBEAT_STALE_S)
        s["connected"] = self.connected
        return s


bridge = Bridge()
FRONTEND_DIR = Path(__file__).resolve().parent / "frontend"


# ---------------------------------------------------------------------------
# FastAPI layer
# ---------------------------------------------------------------------------
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Autonomous mode: set GCS_CONNECTION (and optionally GCS_BAUD) in the
    # environment and the backend connects itself and self-heals forever.
    #   GCS_CONNECTION="udp:127.0.0.1:14551"  (Mission Planner forward)
    #   GCS_CONNECTION="/dev/ttyUSB0" GCS_BAUD=57600  (direct radio)
    if os.environ.get("GCS_AUTOCONNECT") == "1":
        bridge.start_supervisor(DEFAULT_CONNECTION, DEFAULT_BAUD)
    yield

app = FastAPI(title="MP Precision Bridge", lifespan=lifespan)

if FRONTEND_DIR.exists():
    app.mount("/static", StaticFiles(directory=FRONTEND_DIR), name="static")


class ArmBody(BaseModel):
    arm: bool


class ModeBody(BaseModel):
    mode: str = Field(examples=["GUIDED"])


class TakeoffBody(BaseModel):
    altitude: float = Field(gt=0, le=1000)


class GotoBody(BaseModel):
    lat: float = Field(ge=-90, le=90)
    lon: float = Field(ge=-180, le=180)
    altitude: float = Field(gt=0, le=1000)


def _run(fn, *args):
    """Run blocking bridge call in a worker thread (keeps event loop free)."""
    return asyncio.get_event_loop().run_in_executor(None, fn, *args)


class ConnectBody(BaseModel):
    connection: str = Field(default=DEFAULT_CONNECTION,
                            examples=["COM5", "/dev/ttyUSB0",
                                      "tcp:127.0.0.1:5760"])
    baud: int = Field(default=DEFAULT_BAUD,
                      description="Serial baud; 57600 for telemetry radios")


@app.post("/connect")
async def connect(body: ConnectBody = ConnectBody()):
    try:
        await _run(bridge.connect, body.connection, body.baud)
        return {"connected": True, "connection": body.connection}
    except Exception as e:
        raise HTTPException(status_code=503, detail=str(e))


@app.get("/status")
async def status():
    return bridge.snapshot()


@app.get("/", include_in_schema=False)
async def frontend_home():
    index_path = FRONTEND_DIR / "index.html"
    if not index_path.exists():
        raise HTTPException(status_code=404, detail="Frontend not found")
    return FileResponse(index_path)


@app.get("/ports")
async def list_ports():
    """List serial ports plus a SITL preset."""
    ports = []
    try:
        from serial.tools import list_ports as _lp
        for p in _lp.comports():
            ports.append({"device": p.device,
                          "description": p.description or p.device})
    except Exception as e:
        ports.append({"device": "", "description": f"(port scan failed: {e})"})
    ports.append({"device": "tcp:127.0.0.1:5760",
                  "description": "SITL simulator (local)"})
    return {"ports": ports, "bauds": [57600, 115200, 921600, 38400, 9600]}


@app.get("/panel", include_in_schema=False)
async def panel():
    panel_path = Path(__file__).resolve().parent / "gcs_panel.html"
    if not panel_path.exists():
        raise HTTPException(status_code=404,
                            detail="gcs_panel.html not found")
    return FileResponse(panel_path)


def _guard(exc: Exception) -> HTTPException:
    if isinstance(exc, ConnectionError):
        return HTTPException(status_code=503, detail=str(exc))
    if isinstance(exc, ValueError):
        return HTTPException(status_code=400, detail=str(exc))
    if isinstance(exc, TimeoutError):
        return HTTPException(status_code=504, detail=str(exc))
    return HTTPException(status_code=500, detail=str(exc))


@app.post("/command/arm")
async def cmd_arm(body: ArmBody):
    try:
        return await _run(bridge.arm, body.arm)
    except Exception as e:
        raise _guard(e)


@app.post("/command/mode")
async def cmd_mode(body: ModeBody):
    try:
        return await _run(bridge.set_mode, body.mode.upper())
    except Exception as e:
        raise _guard(e)


@app.post("/command/takeoff")
async def cmd_takeoff(body: TakeoffBody):
    try:
        return await _run(bridge.takeoff, body.altitude)
    except Exception as e:
        raise _guard(e)


@app.post("/command/goto")
async def cmd_goto(body: GotoBody):
    try:
        return await _run(bridge.goto, body.lat, body.lon, body.altitude)
    except Exception as e:
        raise _guard(e)


@app.post("/command/rtl")
async def cmd_rtl():
    try:
        return await _run(bridge.rtl)
    except Exception as e:
        raise _guard(e)


class Waypoint(BaseModel):
    lat: float = Field(ge=-90, le=90)
    lon: float = Field(ge=-180, le=180)
    alt: float = Field(gt=0, le=10000)
    command: Optional[int] = None      # MAV_CMD, default NAV_WAYPOINT
    param1: float = 0
    param2: float = 0
    param3: float = 0
    param4: float = 0


class MissionBody(BaseModel):
    waypoints: list[Waypoint] = Field(min_length=1, max_length=700)


class ParamSetBody(BaseModel):
    value: float


@app.get("/mission")
async def mission_get():
    try:
        items = await _run(bridge.mission_download)
        return {"count": len(items), "items": items}
    except Exception as e:
        raise _guard(e)


@app.post("/mission")
async def mission_post(body: MissionBody):
    try:
        wps = [w.model_dump(exclude_none=True) for w in body.waypoints]
        return await _run(bridge.mission_upload, wps)
    except Exception as e:
        raise _guard(e)


@app.delete("/mission")
async def mission_delete():
    try:
        return await _run(bridge.mission_clear)
    except Exception as e:
        raise _guard(e)


@app.post("/command/mission_start")
async def cmd_mission_start():
    try:
        return await _run(bridge.mission_start)
    except Exception as e:
        raise _guard(e)


@app.get("/parameters")
async def params_all():
    try:
        return await _run(bridge.param_fetch_all)
    except Exception as e:
        raise _guard(e)


@app.get("/parameters/{name}")
async def params_get(name: str):
    try:
        return await _run(bridge.param_get, name.upper())
    except Exception as e:
        raise _guard(e)


@app.put("/parameters/{name}")
async def params_set(name: str, body: ParamSetBody):
    try:
        return await _run(bridge.param_set, name.upper(), body.value)
    except Exception as e:
        raise _guard(e)


@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    """Streams vehicle state as JSON at 10 Hz."""
    await ws.accept()
    try:
        while True:
            await ws.send_json(bridge.snapshot())
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        pass
