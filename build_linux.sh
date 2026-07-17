#!/bin/bash
# Build the Linux GCS binary. Run on Linux.
set -e
pip install PyQt6 fastapi uvicorn pymavlink pyserial requests \
    websocket-client websockets pyinstaller --break-system-packages
echo "=== testing unpackaged first ==="
python3 -c "import main" 2>&1 | head -5 || true
echo "=== building ==="
pyinstaller gcs.spec --noconfirm
echo "=== done: dist/GCS/GCS ==="
