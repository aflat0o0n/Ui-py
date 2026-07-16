"""End-to-end flight test of the bridge API against ArduPilot SITL.
Exercises: connect, status, params, mode, arm, takeoff, goto,
mission upload/download, AUTO flight, RTL — via HTTP only,
exactly as the GUI will.
"""
import time
import requests

B = "http://localhost:8000"
ok_count = 0
fail_count = 0


def check(name, cond, info=""):
    global ok_count, fail_count
    mark = "PASS" if cond else "FAIL"
    if cond:
        ok_count += 1
    else:
        fail_count += 1
    print(f"[{mark}] {name} {info}")


def post(path, body=None, timeout=90):
    r = requests.post(B + path, json=body, timeout=timeout)
    return r.status_code, r.json()


def get(path, timeout=90):
    r = requests.get(B + path, timeout=timeout)
    return r.status_code, r.json()


def status():
    return get("/status")[1]


def wait_until(desc, pred, timeout_s, poll=0.5):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        s = status()
        if pred(s):
            return True, s, time.time() - t0
        time.sleep(poll)
    return False, status(), timeout_s


print("=" * 60)
print("BRIDGE + SITL END-TO-END FLIGHT TEST")
print("=" * 60)

# 1. connect
code, r = post("/connect", {"connection": "udp:127.0.0.1:14551"})
check("connect", code == 200 and r.get("connected"), r)

# 2. telemetry flowing
okf, s, dt = wait_until("hb", lambda s: s["link_alive"], 15)
check("telemetry link alive", okf, f"mode={s['mode']}")

# 3. wait for EKF/GPS readiness (SITL needs ~20-40s after boot)
print("... waiting for GPS/EKF (SITL warm-up)")
okf, s, dt = wait_until("gps", lambda s: abs(s["lat"]) > 1, 90)
check("position fix", okf, f"lat={s['lat']:.7f} lon={s['lon']:.7f} ({dt:.0f}s)")
home = (s["lat"], s["lon"])

# 4. parameter read + verified write
code, r = get("/parameters/WP_SPD")
check("param read WP_SPD", code == 200, r)
if code == 200:
    orig = r["value"]
    r2 = requests.put(B + "/parameters/WP_SPD",
                      json={"value": orig + 1}, timeout=30).json()
    check("param set + read-back verify", r2.get("accepted"),
          f"{orig} -> confirmed={r2.get('confirmed')}")
    requests.put(B + "/parameters/WP_SPD", json={"value": orig}, timeout=30)
else:
    check("param set + read-back verify", False, "skipped: read failed")

# 5. mode GUIDED
code, r = post("/command/mode", {"mode": "GUIDED"})
check("set mode GUIDED", r.get("accepted"), r)

# 6. arm (retry: pre-arm checks may still be settling)
armed = False
for attempt in range(10):
    code, r = post("/command/arm", {"arm": True})
    if r.get("accepted"):
        armed = True
        break
    time.sleep(5)
check("arm (pre-arm checks passed)", armed, r)
okf, s, dt = wait_until("armed", lambda s: s["armed"], 10)
check("armed state in telemetry", okf)

# 7. takeoff to 20 m
code, r = post("/command/takeoff", {"altitude": 20})
check("takeoff command ACK", r.get("accepted"), r)
okf, s, dt = wait_until("alt", lambda s: s["alt_rel"] > 18, 60)
check("reached 20 m", okf, f"alt={s['alt_rel']:.1f}m in {dt:.0f}s")

# 8. precision goto ~80 m north
tgt_lat = home[0] + 0.0007
tgt_lon = home[1]
code, r = post("/command/goto", {"lat": tgt_lat, "lon": tgt_lon,
                                 "altitude": 20})
check("goto sent", code == 200, r.get("result"))
okf, s, dt = wait_until(
    "converge",
    lambda s: abs(s["lat"] - tgt_lat) < 0.00003 and
              abs(s["lon"] - tgt_lon) < 0.00003, 90)
err_m = ((s["lat"] - tgt_lat) ** 2 + (s["lon"] - tgt_lon) ** 2) ** 0.5 * 111_139
check("goto convergence", okf, f"final error ~{err_m:.1f} m in {dt:.0f}s")

# 9. mission upload (square pattern)
d = 0.0005
wps = [
    {"lat": home[0] + d, "lon": home[1],     "alt": 25},
    {"lat": home[0] + d, "lon": home[1] + d, "alt": 25},
    {"lat": home[0],     "lon": home[1] + d, "alt": 25},
    {"lat": home[0],     "lon": home[1],     "alt": 25},
]
code, r = post("/mission", {"waypoints": wps})
check("mission upload", r.get("accepted"),
      f"{r.get('result')} items={r.get('items_uploaded')}")

# 10. mission download & verify round-trip precision
code, r = get("/mission")
items = [i for i in r.get("items", []) if i["seq"] > 0]
match = (len(items) == 4 and
         all(abs(items[k]["lat"] - wps[k]["lat"]) < 1e-7 and
             abs(items[k]["lon"] - wps[k]["lon"]) < 1e-7
             for k in range(4)))
check("mission download round-trip (1e-7 deg precision)", match,
      f"{len(items)} items")

# 11. fly the mission in AUTO
code, r = post("/command/mode", {"mode": "AUTO"})
check("set mode AUTO", r.get("accepted"), r)
code, r = post("/command/mission_start")
check("mission start ACK", r.get("accepted"), r)
okf, s, dt = wait_until(
    "wp2", lambda s: abs(s["lat"] - wps[1]["lat"]) < 0.0001 and
                     abs(s["lon"] - wps[1]["lon"]) < 0.0001, 180)
check("mission progressing (reached WP2 area)", okf, f"{dt:.0f}s")

# 12. RTL
code, r = post("/command/rtl")
check("RTL command ACK", r.get("accepted"), r)
okf, s, dt = wait_until("rtl mode", lambda s: s["mode"] == "RTL", 10)
check("mode switched to RTL", okf, f"mode={s['mode']}")
okf, s, dt = wait_until("landed", lambda s: s["alt_rel"] < 0.5
                        and not s["armed"], 240)
check("landed + auto-disarmed", okf,
      f"alt={s['alt_rel']:.2f}m armed={s['armed']} in {dt:.0f}s")

print("=" * 60)
print(f"RESULT: {ok_count} passed, {fail_count} failed")
print("=" * 60)
