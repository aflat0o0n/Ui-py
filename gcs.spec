# -*- mode: python ; coding: utf-8 -*-
"""
PyInstaller spec for the Ground Control Station app.

Builds a one-folder bundle whose entry point is main.py, which starts the
FastAPI backend in-process and then shows the PyQt6 GUI.

Build:  pyinstaller gcs.spec --noconfirm
Output: dist/GCS/GCS        (Linux)
        dist/GCS/GCS.exe     (Windows)
"""
from PyInstaller.utils.hooks import collect_submodules

# Data files the running app reads relative to its own location.
# GCS_backend_service.py resolves these via Path(__file__).parent, which
# points at the bundle root when frozen, so they must sit at the top level.
datas = [
    ("frontend", "frontend"),                     # self-hosted UI (index/app/css)
    ("gcs_panel.html", "."),                       # built-in /panel test console
    ("gcs_config.example.json", "."),              # reference config for operators
]

# uvicorn and pymavlink pull their protocol/dialect modules in dynamically,
# so PyInstaller's static analysis misses them without help.
hiddenimports = (
    collect_submodules("uvicorn")
    + collect_submodules("pymavlink.dialects")
    + ["websockets", "websocket"]
)


a = Analysis(
    ["main.py"],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="GCS",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,          # GUI app: no console window on Windows
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="GCS",
)
