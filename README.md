# Self-Hosted Ground Control App

This repository contains a self-hosted full-stack Ground Control Station app.

- **Backend:** FastAPI MAVLink bridge in `/home/runner/work/Ui-py/Ui-py/GCS_backend_service.py`
- **Frontend:** Web UI in `/home/runner/work/Ui-py/Ui-py/frontend/`

---

## What was done

The project was updated to run as a complete self-hosted app:

1. Added a browser frontend with:
   - connect form (connection string + baud)
   - flight command controls (arm/disarm, GUIDED, takeoff, goto, RTL)
   - live status panel
   - activity log panel
2. Integrated frontend serving into the backend:
   - `GET /` serves `frontend/index.html`
   - `/static` serves frontend assets (`styles.css`, `app.js`)
3. Kept all existing backend APIs and WebSocket telemetry endpoints.

---

## How the app works

### Runtime flow

1. User opens `http://localhost:8000/`.
2. FastAPI serves the frontend page.
3. Frontend calls backend APIs:
   - `POST /connect` to connect the drone/SITL
   - `POST /command/*` for flight actions
   - `GET /status` for current state
4. Frontend opens `WS /ws/telemetry` for live updates (10 Hz).
5. Backend bridge communicates with MAVLink and returns command acknowledgments/results.

### Main components

- **Bridge layer** (`Bridge` class in `GCS_backend_service.py`)
  - manages MAVLink connection
  - receives telemetry
  - sends verified commands (ACK-aware)
- **FastAPI layer**
  - exposes REST + WebSocket endpoints
  - now also serves frontend files
- **Frontend layer** (`frontend/app.js`)
  - calls APIs using `fetch`
  - streams telemetry over WebSocket
  - updates UI panels in real time

---

## Run locally

1. Install dependencies:

```bash
pip install fastapi uvicorn pymavlink pyserial requests
```

2. Start the app:

```bash
cd /home/runner/work/Ui-py/Ui-py
uvicorn GCS_backend_service:app --host 0.0.0.0 --port 8000
```

3. Open in browser:

```text
http://localhost:8000/
```

4. (Optional) Open API docs:

```text
http://localhost:8000/docs
```

---

## Existing scripts

- `/home/runner/work/Ui-py/Ui-py/GCS_launcher.py`  
  Starts MAVProxy + backend stack.
- `/home/runner/work/Ui-py/Ui-py/Drone_client.py`  
  PyQt6 client integration for the backend.
- `/home/runner/work/Ui-py/Ui-py/test_flight.py`  
  SITL end-to-end flight test script.
