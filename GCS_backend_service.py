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
  POST /command/transition     -> {"fixed_wing": true|false} VTOL transition
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
import collections
import csv
import math
import os
import queue as queue_mod
import time
import threading
from contextlib import asynccontextmanager
from datetime import datetime
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

# Our GCS system ID. Mission Planner uses 255; we MUST differ so the FC,
# the router, and ACK routing can tell the two ground stations apart.
GCS_SOURCE_SYSTEM = int(os.environ.get("GCS_SOURCE_SYSTEM", "254"))

# MAV_TYPE -> human vehicle family (a QuadPlane reports FIXED_WING;
# confirm VTOL capability via the Q_ENABLE parameter after connecting)
_VEHICLE_FAMILY = {
    1:  "plane",       # MAV_TYPE_FIXED_WING (incl. QuadPlane)
    2:  "copter",      # MAV_TYPE_QUADROTOR
    13: "copter",      # MAV_TYPE_HEXAROTOR
    14: "copter",      # MAV_TYPE_OCTOROTOR
    19: "vtol",        # MAV_TYPE_VTOL_TAILSITTER_DUOROTOR
    20: "vtol",        # MAV_TYPE_VTOL_TAILSITTER_QUADROTOR
    21: "vtol",        # MAV_TYPE_VTOL_TILTROTOR
    22: "vtol",        # MAV_TYPE_VTOL_FIXEDROTOR
}
_VTOL_STATE_NAMES = {0: "undefined", 1: "transition_to_fw",
                     2: "transition_to_mc", 3: "mc", 4: "fw"}
_GPS_FIX_NAMES = {0: "NO_GPS", 1: "NO_FIX", 2: "2D_FIX", 3: "3D_FIX",
                  4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}
_SEVERITY_NAMES = {0: "emergency", 1: "alert", 2: "critical", 3: "error",
                   4: "warning", 5: "notice", 6: "info", 7: "debug"}
# EKF healthy = attitude + horizontal velocity + absolute horizontal
# position estimates all valid (EKF_STATUS_REPORT flags bits)
_EKF_HEALTHY_BITS = (1 | 2 | 16)   # ATTITUDE | VELOCITY_HORIZ | POS_HORIZ_ABS

# Flight logs (CSV per armed flight) land here:
LOG_DIR = Path(os.environ.get("GCS_LOG_DIR",
                              str(Path.home() / "gcs_flight_logs")))


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
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "groundspeed": 0.0, "airspeed": 0.0, "climb": 0.0,
            "throttle": 0, "heading": 0.0,
            "battery_v": 0.0, "battery_a": 0.0, "battery_pct": -1,
            "gps_fix": 0, "gps_fix_name": "NO_GPS",
            "satellites": 0, "hdop": 99.99,
            "ekf_ok": False, "ekf_flags": 0,
            "vehicle": None,          # "copter" | "plane" | "vtol"
            "vtol_state": None,       # "mc" | "fw" | "transition_*"
            "landed_state": None,     # 1 on ground, 2 in air (EXT_SYS_STATE)
            "current_wp": 0, "wp_dist": 0.0,
            "home_lat": None, "home_lon": None, "home_alt": None,
            "last_heartbeat": 0.0,
        }
        # STATUSTEXT / event ring buffer: [{"seq","time","severity","text"}]
        self._events = collections.deque(maxlen=200)
        self._event_seq = 0
        self._events_lock = threading.Lock()
        # link intent: user connected on purpose; cleared by /disconnect so
        # the supervisor doesn't fight an explicit disconnect
        self._want_link = False
        # flight logging (CSV per armed flight)
        self._log_fh = None
        self._log_lock = threading.Lock()
        self._log_last_row = 0.0

    # ------------- connection -------------
    def connect(self, connection: str = DEFAULT_CONNECTION,
                baud: int = 115200) -> None:
        if self.connected:
            return
        # source_system 254: never 255, or the FC/router confuses us with
        # Mission Planner running alongside on the same link.
        self.master = mavutil.mavlink_connection(
            connection, baud=baud, source_system=GCS_SOURCE_SYSTEM)
        hb = self.master.wait_heartbeat(timeout=15)
        if hb is None:
            self.master = None
            raise ConnectionError(
                f"No heartbeat on '{connection}'. Check the port/address, "
                "baud rate, and that the FC is powered.")
        self.state["last_heartbeat"] = time.time()   # wait_heartbeat saw one
        self.connected = True
        self._want_link = True
        self._request_streams()
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def _request_streams(self):
        intervals = {
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 100_000,  # 10 Hz
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 100_000,             # 10 Hz
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 200_000,              # 5 Hz
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 500_000,           # 2 Hz
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 500_000,          # 2 Hz
            mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: 500_000,
            mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT: 1_000_000,    # 1 Hz
            mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 1_000_000,  # 1 Hz
            mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION: 2_000_000,      # 0.5 Hz
            # VTOL transition + landed state (QuadPlane essential)
            mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 500_000,   # 2 Hz
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
            # CRITICAL when Mission Planner shares the link: the router
            # forwards MP's own heartbeats (MAV_TYPE_GCS, sysid 255) to us.
            # Only messages from the autopilot may drive vehicle state.
            from_vehicle = (msg.get_srcSystem() == self.master.target_system)
            if t == "HEARTBEAT":
                if (not from_vehicle or
                        msg.type == mavutil.mavlink.MAV_TYPE_GCS):
                    continue   # another ground station's heartbeat — ignore
                self.state["mode"] = mavutil.mode_string_v10(msg)
                armed = bool(msg.base_mode &
                             mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if armed != self.state["armed"]:      # arm/disarm edge
                    self._log_start() if armed else self._log_stop()
                self.state["armed"] = armed
                self.state["vehicle"] = _VEHICLE_FAMILY.get(msg.type,
                                                            f"type_{msg.type}")
                self.state["last_heartbeat"] = time.time()
            elif t == "GLOBAL_POSITION_INT":
                self.state["lat"] = msg.lat / 1e7
                self.state["lon"] = msg.lon / 1e7
                self.state["alt_rel"] = msg.relative_alt / 1000.0
                self.state["heading"] = msg.hdg / 100.0
                self._log_row()
            elif t == "ATTITUDE":
                self.state["roll"] = math.degrees(msg.roll)
                self.state["pitch"] = math.degrees(msg.pitch)
                self.state["yaw"] = math.degrees(msg.yaw)
            elif t == "VFR_HUD":
                self.state["groundspeed"] = msg.groundspeed
                self.state["airspeed"] = msg.airspeed      # VTOL-critical
                self.state["climb"] = msg.climb
                self.state["throttle"] = msg.throttle
            elif t == "SYS_STATUS":
                self.state["battery_v"] = msg.voltage_battery / 1000.0
                if msg.current_battery >= 0:
                    self.state["battery_a"] = msg.current_battery / 100.0
                self.state["battery_pct"] = msg.battery_remaining
            elif t == "GPS_RAW_INT":
                self.state["gps_fix"] = msg.fix_type
                self.state["gps_fix_name"] = _GPS_FIX_NAMES.get(
                    msg.fix_type, str(msg.fix_type))
                self.state["satellites"] = msg.satellites_visible
                if msg.eph != 65535:
                    self.state["hdop"] = msg.eph / 100.0
            elif t == "EKF_STATUS_REPORT":
                self.state["ekf_flags"] = msg.flags
                self.state["ekf_ok"] = (
                    (msg.flags & _EKF_HEALTHY_BITS) == _EKF_HEALTHY_BITS)
            elif t == "MISSION_CURRENT":
                self.state["current_wp"] = msg.seq
            elif t == "NAV_CONTROLLER_OUTPUT":
                self.state["wp_dist"] = msg.wp_dist
            elif t == "HOME_POSITION":
                self.state["home_lat"] = msg.latitude / 1e7
                self.state["home_lon"] = msg.longitude / 1e7
                self.state["home_alt"] = msg.altitude / 1000.0
            elif t == "STATUSTEXT":
                if from_vehicle:
                    self._push_event(msg.severity, msg.text)
            elif t == "EXTENDED_SYS_STATE":
                self.state["vtol_state"] = _VTOL_STATE_NAMES.get(
                    msg.vtol_state, msg.vtol_state)
                self.state["landed_state"] = msg.landed_state
            elif t == "COMMAND_ACK":
                # MAVLink2 ACKs carry a target; drop ACKs addressed to the
                # other GCS (e.g. Mission Planner) so results don't cross.
                tgt = getattr(msg, "target_system", 0)
                if tgt not in (0, GCS_SOURCE_SYSTEM):
                    continue
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

    # ------------- STATUSTEXT events -------------
    def _push_event(self, severity: int, text: str):
        with self._events_lock:
            self._event_seq += 1
            evt = {"seq": self._event_seq,
                   "time": time.time(),
                   "severity": severity,
                   "severity_name": _SEVERITY_NAMES.get(severity,
                                                        str(severity)),
                   "text": text}
            self._events.append(evt)
        self._log_event(evt)

    def events_since(self, seq: int) -> list:
        """Events newer than seq (frontend polls or WS streams these)."""
        with self._events_lock:
            return [e for e in self._events if e["seq"] > seq]

    # ------------- flight logging (CSV per armed flight) -------------
    _LOG_FIELDS = ["time", "lat", "lon", "alt_rel", "mode", "armed",
                   "roll", "pitch", "yaw", "groundspeed", "airspeed",
                   "climb", "throttle", "heading", "battery_v",
                   "battery_pct", "gps_fix_name", "satellites",
                   "vtol_state", "current_wp", "wp_dist"]

    def _log_start(self):
        name = None
        with self._log_lock:
            if self._log_fh:
                return
            try:
                LOG_DIR.mkdir(parents=True, exist_ok=True)
                name = datetime.now().strftime("flight_%Y%m%d_%H%M%S.csv")
                self._log_fh = open(LOG_DIR / name, "w", newline="")
                w = csv.writer(self._log_fh)
                w.writerow(self._LOG_FIELDS + ["event"])
            except OSError as e:
                self._log_fh = None
                name = None
                print(f"[log] could not open flight log: {e}")
        if name:
            # outside the lock: _push_event -> _log_event re-acquires it
            self._push_event(6, f"GCS: flight log started ({name})")

    def _log_stop(self):
        with self._log_lock:
            if self._log_fh:
                try:
                    self._log_fh.close()
                except OSError:
                    pass
                self._log_fh = None

    def _log_row(self):
        """~5 Hz telemetry rows while armed (called from 10 Hz position)."""
        if not self._log_fh or time.time() - self._log_last_row < 0.2:
            return
        self._log_last_row = time.time()
        with self._log_lock:
            if not self._log_fh:
                return
            try:
                csv.writer(self._log_fh).writerow(
                    [f"{time.time():.2f}"] +
                    [self.state[k] for k in self._LOG_FIELDS[1:]] + [""])
            except (OSError, ValueError):
                pass

    def _log_event(self, evt: dict):
        with self._log_lock:
            if not self._log_fh:
                return
            try:
                csv.writer(self._log_fh).writerow(
                    [f"{evt['time']:.2f}"] +
                    [""] * (len(self._LOG_FIELDS) - 1) +
                    [f"[{evt['severity_name']}] {evt['text']}"])
            except (OSError, ValueError):
                pass

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

    def available_modes(self) -> dict:
        """Flight modes valid for THIS vehicle (copter vs plane differ
        completely — even the custom_mode numbers). The frontend should
        build its mode buttons from this instead of hardcoding."""
        if not self.connected:
            raise ConnectionError("Not connected")
        mapping = self.master.mode_mapping() or {}
        return {"vehicle": self.state["vehicle"],
                "modes": sorted(mapping)}

    def takeoff(self, alt_m: float) -> dict:
        """Copter: guided takeoff. QuadPlane (ArduPlane >= 4.2): the same
        NAV_TAKEOFF in GUIDED performs a vertical VTOL takeoff, provided
        Q_GUIDED_MODE is configured; the FC's ACK tells us the truth."""
        cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        return self._send_verified(cmd, lambda:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0, 0, 0, 0, 0, 0, 0, alt_m))

    def rtl(self, vtol_land: bool = False) -> dict:
        """Return to launch.
        vtol_land=True on a QuadPlane switches to QRTL: fly home, then
        transition to hover and land vertically. Plain RTL on a plane
        only loiters at home altitude and never lands (unless
        Q_RTL_MODE=1 is set on the FC). Copter ignores the flag."""
        if vtol_land:
            mapping = self.master.mode_mapping() or {}
            if "QRTL" in mapping:
                return self.set_mode("QRTL")
            # not a quadplane — fall through to normal RTL
        cmd = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        return self._send_verified(cmd, lambda:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0, 0, 0, 0, 0, 0, 0, 0))

    def land(self) -> dict:
        """Land at the current position. QuadPlane: QLAND (vertical);
        copter: LAND. The big red button, distinct from RTL."""
        if not self.connected:
            raise ConnectionError("Not connected")
        mapping = self.master.mode_mapping() or {}
        for m in ("QLAND", "LAND"):
            if m in mapping:
                return self.set_mode(m)
        raise ValueError("No LAND/QLAND mode on this vehicle")

    def pause(self) -> dict:
        """Hold position: QLOITER (QuadPlane hover) or LOITER (copter).
        On a fixed-wing-only plane LOITER circles in place — still a
        pause. Mission state is preserved; resume() continues it."""
        if not self.connected:
            raise ConnectionError("Not connected")
        mapping = self.master.mode_mapping() or {}
        for m in ("QLOITER", "LOITER"):
            if m in mapping:
                return self.set_mode(m)
        raise ValueError("No loiter mode available to pause into")

    def resume(self) -> dict:
        """Continue the interrupted AUTO mission from the current item."""
        return self.set_mode("AUTO")

    def change_speed(self, speed_ms: float, airspeed: bool = True) -> dict:
        """In-flight speed change. airspeed=True targets airspeed (what a
        plane/VTOL flies by); False targets groundspeed (copter-style)."""
        cmd = mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED
        stype = 0 if airspeed else 1
        return self._send_verified(cmd, lambda:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0, stype, speed_ms, -1, 0, 0, 0, 0))

    def change_altitude(self, alt_m: float) -> dict:
        """GUIDED altitude change, keeping the current position target.
        Plane/QuadPlane: MAV_CMD_GUIDED_CHANGE_ALTITUDE. Copter: re-send
        the position target at the current lat/lon with the new alt."""
        if not self.connected:
            raise ConnectionError("Not connected")
        if self.state["mode"] != "GUIDED":
            raise ValueError("Altitude change requires GUIDED mode "
                             f"(current: {self.state['mode']})")
        if self.state["vehicle"] in ("plane", "vtol"):
            cmd = mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE
            return self._send_verified(cmd, lambda:
                self.master.mav.command_int_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    cmd, 0, 0, 0, 0, 0, 0, 0, 0, alt_m))
        return self.goto(self.state["lat"], self.state["lon"], alt_m)

    def set_current_wp(self, seq: int) -> dict:
        """Jump the active mission item ("skip to this waypoint on the
        map"). Confirmed by the FC echoing MISSION_CURRENT."""
        if not self.connected:
            raise ConnectionError("Not connected")
        q = self._subscribe("MISSION_CURRENT")
        try:
            with self._lock:
                self.master.mav.mission_set_current_send(
                    self.master.target_system, self.master.target_component,
                    seq)
            deadline = time.time() + 3
            while time.time() < deadline:
                try:
                    msg = q.get(timeout=1)
                except queue_mod.Empty:
                    continue
                if msg.seq == seq:
                    return {"result": "ACCEPTED", "accepted": True,
                            "current_wp": seq}
            return {"result": "TIMEOUT_NO_CONFIRM", "accepted": False}
        finally:
            self._unsubscribe(q)

    def vtol_transition(self, to_fixed_wing: bool) -> dict:
        """Command a QuadPlane VTOL transition (MAV_CMD_DO_VTOL_TRANSITION).
        to_fixed_wing=True -> fixed-wing (forward flight);
        to_fixed_wing=False -> multicopter (hover)."""
        state = (mavutil.mavlink.MAV_VTOL_STATE_FW if to_fixed_wing
                 else mavutil.mavlink.MAV_VTOL_STATE_MC)
        cmd = mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION
        return self._send_verified(cmd, lambda:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                cmd, 0, state, 0, 0, 0, 0, 0, 0))

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

    # =================== MISSION / FENCE / RALLY PROTOCOL ===============
    _MISSION_TYPES = {
        "mission": 0,   # MAV_MISSION_TYPE_MISSION
        "fence": 1,     # MAV_MISSION_TYPE_FENCE
        "rally": 2,     # MAV_MISSION_TYPE_RALLY
    }

    def mission_download(self, kind: str = "mission") -> list:
        """Pull the full mission/fence/rally list from the FC."""
        if not self.connected:
            raise ConnectionError("Not connected")
        mt = self._MISSION_TYPES[kind]
        q = self._subscribe("MISSION_COUNT", "MISSION_ITEM_INT",
                            "MISSION_ITEM")
        try:
            with self._lock:
                self.master.mav.mission_request_list_send(
                    self.master.target_system, self.master.target_component,
                    mt)
            count_msg = self._q_wait(q, "MISSION_COUNT", 5)
            items = []
            for seq in range(count_msg.count):
                with self._lock:
                    self.master.mav.mission_request_int_send(
                        self.master.target_system,
                        self.master.target_component, seq, mt)
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
                    mavutil.mavlink.MAV_MISSION_ACCEPTED, mt)
            return items
        finally:
            self._unsubscribe(q)

    def mission_upload(self, waypoints: list,
                       kind: str = "mission") -> dict:
        """Push a mission/fence/rally list (MISSION protocol handshake,
        1e7 int coords). waypoints: [{lat, lon, alt, command?, param1..4?,
        frame?}, ...]. For kind="mission", item 0 is conventionally home;
        ArduPilot manages it, so we prepend a dummy home item. Fence and
        rally lists have no home item — items upload as given (fence
        vertices use command NAV_FENCE_* with param1 = vertex count;
        rally points use NAV_RALLY_POINT — supplied by the caller)."""
        if not self.connected:
            raise ConnectionError("Not connected")
        mt = self._MISSION_TYPES[kind]
        items = []
        if kind == "mission":
            # seq 0 = home placeholder, then user waypoints
            items.append(dict(lat=0, lon=0, alt=0,
                              command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                              frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
                              param1=0, param2=0, param3=0, param4=0))
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

    def mission_clear(self, kind: str = "mission") -> dict:
        if not self.connected:
            raise ConnectionError("Not connected")
        mt = self._MISSION_TYPES[kind]
        q = self._subscribe("MISSION_ACK")
        try:
            with self._lock:
                self.master.mav.mission_clear_all_send(
                    self.master.target_system, self.master.target_component,
                    mt)
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
        self._log_stop()
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None

    def disconnect(self) -> dict:
        """User-intended disconnect: drop the link AND tell the
        supervisor to stand down, so it doesn't auto-reconnect. Lets the
        operator switch between SITL and the real drone at runtime."""
        self._want_link = False
        was = self.connected
        self.teardown()
        return {"disconnected": True, "was_connected": was}

    def start_supervisor(self, connection: str, baud: int = 115200):
        """Autonomous mode: keep the link up forever.
        Connects on startup, watches heartbeat health, and reconnects
        automatically after any link loss. Runs for the process lifetime.
        Stands down while the user has explicitly disconnected."""
        if self._supervising:
            return
        self._supervising = True
        self._want_link = True

        def loop():
            while True:
                if not self._want_link:
                    time.sleep(1)          # user disconnected on purpose
                    continue
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
    return asyncio.get_running_loop().run_in_executor(None, fn, *args)


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


@app.post("/disconnect")
async def disconnect():
    """Drop the drone link and stand down auto-reconnect, so the
    operator can switch targets (e.g. SITL -> real drone) at runtime."""
    return await _run(bridge.disconnect)


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


class RtlBody(BaseModel):
    vtol_land: bool = Field(
        default=False,
        description="QuadPlane: use QRTL (fly home, hover, land "
                    "vertically) instead of fixed-wing RTL loiter")


@app.post("/command/rtl")
async def cmd_rtl(body: RtlBody = RtlBody()):
    try:
        return await _run(bridge.rtl, body.vtol_land)
    except Exception as e:
        raise _guard(e)


@app.get("/modes")
async def modes():
    """Modes valid for the connected vehicle — frontend builds its mode
    buttons from this list (copter and QuadPlane modes are disjoint)."""
    try:
        return await _run(bridge.available_modes)
    except Exception as e:
        raise _guard(e)


@app.post("/command/land")
async def cmd_land():
    """Land at the current position (QLAND on QuadPlane, LAND on copter)."""
    try:
        return await _run(bridge.land)
    except Exception as e:
        raise _guard(e)


@app.post("/command/pause")
async def cmd_pause():
    """Hold position now (QLOITER/LOITER). Mission resumes with /resume."""
    try:
        return await _run(bridge.pause)
    except Exception as e:
        raise _guard(e)


@app.post("/command/resume")
async def cmd_resume():
    """Continue the paused AUTO mission from the current item."""
    try:
        return await _run(bridge.resume)
    except Exception as e:
        raise _guard(e)


class SpeedBody(BaseModel):
    speed: float = Field(gt=0, le=100, description="m/s")
    airspeed: bool = Field(
        default=True,
        description="True = airspeed target (plane/VTOL); "
                    "False = groundspeed (copter)")


@app.post("/command/speed")
async def cmd_speed(body: SpeedBody):
    try:
        return await _run(bridge.change_speed, body.speed, body.airspeed)
    except Exception as e:
        raise _guard(e)


class AltitudeBody(BaseModel):
    altitude: float = Field(gt=0, le=1000, description="relative alt, m")


@app.post("/command/altitude")
async def cmd_altitude(body: AltitudeBody):
    """GUIDED altitude nudge, keeping the current position target."""
    try:
        return await _run(bridge.change_altitude, body.altitude)
    except Exception as e:
        raise _guard(e)


class SetWpBody(BaseModel):
    seq: int = Field(ge=0, le=700)


@app.post("/command/set_wp")
async def cmd_set_wp(body: SetWpBody):
    """Jump the active mission to this item (map click: 'skip to here')."""
    try:
        return await _run(bridge.set_current_wp, body.seq)
    except Exception as e:
        raise _guard(e)


@app.get("/events")
async def events(since: int = 0):
    """FC status messages (arming failures, prearm reasons, warnings)
    newer than `since`. WS clients get these pushed; this is for polling
    or backfilling a message console on (re)connect."""
    return {"events": bridge.events_since(since)}


@app.get("/logs")
async def logs_list():
    """Flight logs recorded by the backend (one CSV per armed flight)."""
    if not LOG_DIR.exists():
        return {"dir": str(LOG_DIR), "logs": []}
    files = sorted(LOG_DIR.glob("flight_*.csv"), reverse=True)
    return {"dir": str(LOG_DIR),
            "logs": [{"name": f.name, "size": f.stat().st_size}
                     for f in files]}


@app.get("/logs/{name}")
async def logs_get(name: str):
    # strict name check — no path traversal
    if not (name.startswith("flight_") and name.endswith(".csv")
            and "/" not in name and "\\" not in name and ".." not in name):
        raise HTTPException(status_code=400, detail="Bad log name")
    path = LOG_DIR / name
    if not path.exists():
        raise HTTPException(status_code=404, detail="Log not found")
    return FileResponse(path, media_type="text/csv", filename=name)


class TransitionBody(BaseModel):
    fixed_wing: bool = Field(
        description="True -> fixed-wing (forward flight); "
                    "False -> multicopter (hover)")


@app.post("/command/transition")
async def cmd_transition(body: TransitionBody):
    try:
        return await _run(bridge.vtol_transition, body.fixed_wing)
    except Exception as e:
        raise _guard(e)


class Waypoint(BaseModel):
    lat: float = Field(ge=-90, le=90)
    lon: float = Field(ge=-180, le=180)
    alt: float = Field(ge=0, le=10000)   # ge=0: fence vertices use alt 0
    command: Optional[int] = None      # MAV_CMD, default NAV_WAYPOINT
    frame: Optional[int] = None        # MAV_FRAME, default GLOBAL_RELATIVE_ALT
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


def _list_endpoints(kind: str):
    """GET/POST/DELETE trio for fence and rally via the same transport."""

    @app.get(f"/{kind}", name=f"{kind}_get")
    async def _get():
        try:
            items = await _run(bridge.mission_download, kind)
            return {"count": len(items), "items": items}
        except Exception as e:
            raise _guard(e)

    @app.post(f"/{kind}", name=f"{kind}_post")
    async def _post(body: MissionBody):
        try:
            wps = [w.model_dump(exclude_none=True) for w in body.waypoints]
            return await _run(bridge.mission_upload, wps, kind)
        except Exception as e:
            raise _guard(e)

    @app.delete(f"/{kind}", name=f"{kind}_delete")
    async def _delete():
        try:
            return await _run(bridge.mission_clear, kind)
        except Exception as e:
            raise _guard(e)


# /fence: polygon vertices (NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1 =
# vertex count) or circles; /rally: NAV_RALLY_POINT items — safe QRTL
# landing spots other than home. Frontend supplies the command numbers.
_list_endpoints("fence")
_list_endpoints("rally")


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
    """Typed JSON frames:
      {"type": "state", ...snapshot}  at 10 Hz
      {"type": "event", "seq", "severity", "severity_name", "text", "time"}
        pushed as FC STATUSTEXT arrives (arming failures, warnings, etc.)
    Frontends route on "type"; unknown types must be ignored (forward
    compatibility)."""
    await ws.accept()
    last_seq = 0
    # backfill recent events so a (re)connecting client sees context
    for evt in bridge.events_since(0)[-20:]:
        last_seq = evt["seq"]
    try:
        while True:
            for evt in bridge.events_since(last_seq):
                last_seq = evt["seq"]
                await ws.send_json({"type": "event", **evt})
            await ws.send_json({"type": "state", **bridge.snapshot()})
            await asyncio.sleep(0.1)
    except (WebSocketDisconnect, Exception):
        # abrupt client drops raise transport errors, not only
        # WebSocketDisconnect — either way, just end this stream
        pass
