# Self-Hosted Ground Control App

This repository now provides a self-hosted full-stack app:

- **Backend:** FastAPI MAVLink bridge (`GCS_backend_service.py`)
- **Frontend:** Browser UI served by the backend (`/frontend`)

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

3. Open:

```text
http://localhost:8000/
```

## What the self-hosted UI supports

- Connect to a MAVLink endpoint (`/connect`)
- Live telemetry/status (`/ws/telemetry` + `/status`)
- Arm/disarm, mode switch, takeoff, goto, RTL commands
- Action log panel for command responses/errors

## API docs

- Swagger UI: `http://localhost:8000/docs`

## Existing scripts

- `GCS_launcher.py`: launches MAVProxy + backend stack
- `Drone_client.py`: PyQt6 client for backend integration
- `test_flight.py`: SITL end-to-end test script
