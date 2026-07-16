## 7. Test procedure

Run these phases in order. Each step lists its expected result.

### Phase 1 — backend alone against SITL

Terminal 1, the simulated drone:

```bash
cd ~/ardupilot
./build/sitl/bin/arducopter --model quad --speedup 4 \
    --home 33.6844,73.0479,500,0 --serial0 tcp:0
```

Expected: `bind port 5760 for SERIAL0`. Leave running. A repeating "Waiting for internal clock bits" line is a harmless simulated-LED message and can be ignored.

Terminal 2, the backend connected directly to SITL:

```bash
cd ~/gcs
GCS_CONNECTION="tcp:127.0.0.1:5760" python3 -m uvicorn \
    gcs_backend_service:app --host 127.0.0.1 --port 8000
```

Expected: `Uvicorn running on http://127.0.0.1:8000`. Leave running.

Terminal 3, connect and verify:

```bash
curl -X POST http://localhost:8000/connect \
    -H 'Content-Type: application/json' -d '{}'
curl http://localhost:8000/status
```

Expected: `{"connected":true,...}` and a status containing `"lat":33.68...`, `"mode":"STABILIZE"`, `"link_alive":true`. If "No heartbeat": SITL is not running or still booting; wait 20 seconds and retry.

Full regression (the certification step):

```bash
cd ~/gcs && python3 flight_test.py
```

Expected: PASS lines for connect, position fix, parameter read and verified write, GUIDED, arm, takeoff to 20 m, precision goto, mission upload and verified download, AUTO flight, RTL, landing — finishing with `RESULT: 20 passed, 0 failed`. A failed "position fix" step means the simulated GPS needed longer to converge; rerun. Never run this script against a real vehicle — it commands an actual takeoff.

Stop both terminals with Ctrl+C when done.

### Phase 2 — full production topology

This reproduces exactly what the packaged GUI will do.

Terminal 1: restart SITL (same command as above). Terminal 2, the launcher in standalone test mode:

```bash
cd ~/gcs && python3 gcs_launcher.py
```

Expected: `stack up: {...}` within about ten seconds, proving the launcher spawned the router and backend and health-checked them. Leave running.

Terminal 3, the operator flow and the Mission Planner seat:

```bash
curl -X POST http://localhost:8000/connect \
    -H 'Content-Type: application/json' -d '{}'
curl http://localhost:8000/status

python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udp:127.0.0.1:14550')
m.wait_heartbeat(timeout=10)
print('MP seat OK')"
```

Expected: connected via `udp:127.0.0.1:14551`, `link_alive: true`, then `MP seat OK` — confirming a real Mission Planner could join in parallel on 14550.

Teardown check: press Ctrl+C in the launcher terminal, then:

```bash
curl -m 2 http://localhost:8000/status
```

Expected: connection refused — `stop_all()` cleaned up with no zombie processes.
