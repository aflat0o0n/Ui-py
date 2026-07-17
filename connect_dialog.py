"""
connect_dialog.py — runtime connection picker for the GCS GUI.

Two ways to connect, operator's choice at runtime:
  • Serial: pick a scanned port + baud (telemetry radio / USB)
  • Network: choose the link type, enter address/port. The link type prevents
    the classic "Cannot assign requested address" error by making the
    direction explicit:
      - "UDP listen (receive)"  -> udp:0.0.0.0:PORT
          Use when something ELSE sends to this PC — e.g. Mission Planner's
          MAVLink forward set to UDP Client -> this PC's IP, or a companion
          computer streaming to us. You do NOT type the remote IP here.
      - "UDP connect to host"   -> udpout:IP:PORT
          Use when THIS app must reach out to a device that is listening.
      - "TCP connect to host"   -> tcp:IP:PORT
          SITL on another machine, or a TCP MAVLink server.

Usage:
    from connect_dialog import ConnectDialog
    ConnectDialog(self.drone, self).exec()
"""

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QPushButton, QLineEdit, QTabWidget, QWidget, QSpinBox
)


# link type label -> (scheme, needs_ip)
NET_MODES = {
    "UDP listen (receive from MP / companion)": ("udp_listen", False),
    "UDP connect to host": ("udpout", True),
    "TCP connect to host": ("tcp", True),
}


class ConnectDialog(QDialog):
    def __init__(self, drone, parent=None):
        super().__init__(parent)
        self.drone = drone
        self.setWindowTitle("Connect to drone")
        self.setMinimumWidth(460)
        self._build()
        self.drone.command_result.connect(self._on_result)
        self.drone.connection_changed.connect(self._on_conn)
        self._refresh()

    def _build(self):
        v = QVBoxLayout(self)
        self.tabs = QTabWidget()
        self.tabs.addTab(self._serial_tab(), "Serial")
        self.tabs.addTab(self._network_tab(), "Network")
        v.addWidget(self.tabs)

        self.status = QLabel("")
        self.status.setStyleSheet("color:#8b96a5")
        self.status.setWordWrap(True)
        v.addWidget(self.status)

        btns = QHBoxLayout()
        btns.addStretch()
        cancel = QPushButton("Cancel")
        cancel.clicked.connect(self.reject)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setDefault(True)
        self.connect_btn.clicked.connect(self._do_connect)
        btns.addWidget(cancel)
        btns.addWidget(self.connect_btn)
        v.addLayout(btns)

    def _serial_tab(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.addWidget(QLabel("Port"))
        row = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        row.addWidget(self.port_combo, 1)
        rescan = QPushButton("Rescan")
        rescan.clicked.connect(self._refresh)
        row.addWidget(rescan)
        v.addLayout(row)
        v.addWidget(QLabel("Baud rate"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200", "921600", "38400", "9600"])
        v.addWidget(self.baud_combo)
        v.addStretch()
        return w

    def _network_tab(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.addWidget(QLabel("Link type"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(list(NET_MODES.keys()))
        self.mode_combo.currentTextChanged.connect(self._mode_changed)
        v.addWidget(self.mode_combo)

        self.ip_label = QLabel("Remote IP address")
        v.addWidget(self.ip_label)
        self.ip_edit = QLineEdit()
        self.ip_edit.setPlaceholderText("e.g. 192.168.1.50")
        v.addWidget(self.ip_edit)

        v.addWidget(QLabel("Port"))
        self.netport_spin = QSpinBox()
        self.netport_spin.setRange(1, 65535)
        self.netport_spin.setValue(14550)
        v.addWidget(self.netport_spin)

        self.hint = QLabel("")
        self.hint.setStyleSheet("color:#8b96a5;font-size:11px")
        self.hint.setWordWrap(True)
        v.addWidget(self.hint)
        v.addStretch()
        self._mode_changed(self.mode_combo.currentText())
        return w

    def _mode_changed(self, label):
        _scheme, needs_ip = NET_MODES[label]
        self.ip_edit.setEnabled(needs_ip)
        self.ip_label.setEnabled(needs_ip)
        if needs_ip:
            self.hint.setText("This app connects out to the device at the "
                              "IP/port above.")
        else:
            self.hint.setText("This app listens on the port for data sent TO "
                              "this PC. In Mission Planner set the MAVLink "
                              "forward to UDP Client -> this PC's IP and the "
                              "same port. Do not enter an IP here.")

    # ---- serial port scan ----
    def _refresh(self):
        self.status.setText("Scanning ports...")
        self.drone.list_ports()

    def _on_result(self, name, data):
        if name != "PORTS":
            return
        self.port_combo.clear()
        for p in data.get("ports", []):
            self.port_combo.addItem(p["description"], p["device"])
        bauds = data.get("bauds")
        if bauds:
            self.baud_combo.clear()
            self.baud_combo.addItems([str(b) for b in bauds])
        self.status.setText("Select a connection and press Connect.")

    # ---- connect using whichever tab is active ----
    def _do_connect(self):
        if self.tabs.currentIndex() == 0:
            idx = self.port_combo.currentIndex()
            device = self.port_combo.itemData(idx) or self.port_combo.currentText().strip()
            if not device:
                self.status.setText("Choose a port first.")
                return
            baud = int(self.baud_combo.currentText())
            target, kwbaud = device, baud
        else:
            scheme, needs_ip = NET_MODES[self.mode_combo.currentText()]
            port = self.netport_spin.value()
            if scheme == "udp_listen":
                target = f"udp:0.0.0.0:{port}"
            else:
                ip = self.ip_edit.text().strip()
                if not ip:
                    self.status.setText("Enter the remote IP address.")
                    return
                target = f"{scheme}:{ip}:{port}"
            kwbaud = None

        self.status.setText(f"Connecting to {target} ...")
        self.connect_btn.setEnabled(False)
        self.drone.connect_drone(target, kwbaud)

    def _on_conn(self, ok):
        if ok:
            self.accept()
        else:
            self.status.setText("Connection failed - check the link type, "
                                "address/port, firewall, and that the drone "
                                "or MP forward is running.")
            self.connect_btn.setEnabled(True)
