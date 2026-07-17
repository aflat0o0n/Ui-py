"""
main.py — entry point for the packaged GCS app (Windows and Linux).

Packaging pattern: start the backend IN-PROCESS (no subprocess), then build
the PyQt GUI. Replace the demo window with your team's real UI — only the
startup lines in main() matter for packaging.

Run in development:  python main.py
"""

import sys

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QMessageBox
)

import backend_inprocess
from Drone_client import DroneClient
from connect_dialog import ConnectDialog


class MainWindow(QMainWindow):
    def __init__(self, drone: DroneClient):
        super().__init__()
        self.drone = drone
        self.setWindowTitle("Ground Control Station")
        self._build()
        self.drone.telemetry.connect(self._on_tele)
        self.drone.command_result.connect(self._on_result)
        self.drone.connection_changed.connect(self._on_conn)
        self.drone.error.connect(self._on_error)

    def _build(self):
        root = QWidget(); v = QVBoxLayout(root)
        self.link = QLabel("NO LINK")
        self.link.setStyleSheet("color:#e5484d;font-size:18px")
        self.mode = QLabel("mode: -"); self.alt = QLabel("alt: -")
        v.addWidget(self.link); v.addWidget(self.mode); v.addWidget(self.alt)
        row = QHBoxLayout()
        for text, fn in [("Connect", self._open_connect),
                         ("GUIDED", lambda: self.drone.set_mode("GUIDED")),
                         ("Arm", lambda: self.drone.arm(True)),
                         ("Takeoff 20m", lambda: self.drone.takeoff(20)),
                         ("RTL", self.drone.rtl)]:
            b = QPushButton(text); b.clicked.connect(fn); row.addWidget(b)
        v.addLayout(row)
        self.status = QLabel(""); v.addWidget(self.status)
        self.setCentralWidget(root); self.resize(440, 220)

    def _open_connect(self):
        ConnectDialog(self.drone, self).exec()

    def _on_tele(self, s):
        alive = s.get("link_alive")
        self.link.setText("LINK OK" if alive else "NO LINK")
        self.link.setStyleSheet(
            f"color:{'#37c871' if alive else '#e5484d'};font-size:18px")
        self.mode.setText(f"mode: {s.get('mode')}")
        self.alt.setText(f"alt: {s.get('alt_rel', 0):.1f} m")

    def _on_result(self, name, r):
        ok = r.get("accepted")
        self.status.setText(
            f"{name}: {'OK' if ok else 'REJECTED - ' + str(r.get('result'))}")

    def _on_conn(self, ok):
        self.status.setText("Connected" if ok else "Connect failed")

    def _on_error(self, msg):
        self.status.setText(msg)

    def closeEvent(self, e):
        self.drone.shutdown()
        backend_inprocess.stop_backend()
        e.accept()


def main():
    app = QApplication(sys.argv)

    # 1. start the backend inside this process (packaging-safe).
    #    host/port/connection come from gcs_config.json (or defaults).
    try:
        info = backend_inprocess.start_backend()
    except RuntimeError as e:
        QMessageBox.critical(None, "Startup failed",
                             f"Backend did not start:\n{e}")
        return 1

    # 2. point the client at the backend's ACTUAL url (port may have
    #    fallen back if the preferred one was busy).
    DroneClient.set_backend_url(info["url"])

    # 3. build the GUI. Operator picks the connection in the Connect dialog.
    drone = DroneClient()
    win = MainWindow(drone)
    win.show()
    # helpful for LAN access (phone / Mission Planner on another PC):
    print("Backend reachable at", backend_inprocess.backend_url(),
          "(and /panel there)")
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
