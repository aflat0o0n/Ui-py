@echo off
REM Build the Windows GCS.exe. Run on Windows in the project folder.
pip install PyQt6 fastapi uvicorn pymavlink pyserial requests websocket-client websockets pyinstaller
echo === testing unpackaged first ===
python -c "import main"
echo === building ===
pyinstaller gcs.spec --noconfirm
echo === done: dist\GCS\GCS.exe ===
pause
