import sys
import socket
import struct
import datetime

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QTextEdit, QTabWidget,
    QGroupBox, QGridLayout, QFileDialog, QSpinBox, QFrame, QComboBox
)
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QFont

# =============================================================================
# CRC16 CCITT-FALSE  (poly=0x1021, init=0xFFFF)
# Matches firmware prv_CalculateCRC16 exactly
# =============================================================================
def calculate_crc(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


# =============================================================================
# Protocol Builder  (original IOHandler TCP frame format)
#
# Frame layout (big-endian):
#   UID(4) | MsgType(2) | CompId(1) | DockId(1) | CmdId(1) | PayloadLen(1)
#   + payload + CRC16(2)
# =============================================================================
class Protocol:
    CMD_GPIO                 = 1
    CMD_ANALOG               = 2
    CMD_CHARGING             = 3
    CMD_BOOT                 = 5
    CMD_RESET                = 6
    CMD_COMMISSIONING_COMMAND = 0x07

    @staticmethod
    def build(uid: int, dock: int, comp: int, cmd: int, payload: bytes) -> bytes:
        header = struct.pack(">I H B B B B",
                             uid, 1, comp, dock, cmd, len(payload))
        raw = header + payload
        return raw + struct.pack(">H", calculate_crc(raw))


# =============================================================================
# TCP Receive Worker Thread
# =============================================================================
class TCPWorker(QThread):
    received     = pyqtSignal(bytes)
    disconnected = pyqtSignal()

    def __init__(self, sock: socket.socket):
        super().__init__()
        self.sock    = sock
        self.running = True

    def run(self):
        while self.running:
            try:
                data = self.sock.recv(2048)
                if not data:
                    self.disconnected.emit()
                    break
                self.received.emit(data)
            except Exception:
                self.disconnected.emit()
                break

    def stop(self):
        self.running = False


# =============================================================================
# Shared Stylesheet
# =============================================================================
STYLE = """
QWidget {
    background-color: #f5f6fa;
    font-size: 13px;
    font-family: "Segoe UI", Arial, sans-serif;
}
QLabel {
    color: #2f3640;
    font-weight: bold;
}
QLineEdit, QSpinBox {
    background: white;
    border: 1px solid #dcdde1;
    padding: 5px;
    border-radius: 4px;
    color: #2f3640;
}
QLineEdit:focus, QSpinBox:focus {
    border: 1px solid #4078c0;
}
QPushButton {
    background-color: #4078c0;
    color: white;
    border-radius: 4px;
    padding: 6px 14px;
    font-weight: bold;
}
QPushButton:hover  { background-color: #2d5fa4; }
QPushButton:pressed{ background-color: #1e4a8a; }
QPushButton#btnGreen  { background-color: #27ae60; }
QPushButton#btnGreen:hover { background-color: #1e8449; }
QPushButton#btnRed    { background-color: #c0392b; }
QPushButton#btnRed:hover   { background-color: #922b21; }
QPushButton#btnOrange { background-color: #d35400; }
QPushButton#btnOrange:hover{ background-color: #a04000; }
QTextEdit {
    background: white;
    border: 1px solid #dcdde1;
    color: #2f3640;
    font-family: "Consolas", "Courier New", monospace;
    font-size: 12px;
}
QTabWidget::pane {
    border: 1px solid #dcdde1;
    border-radius: 4px;
    background: #f5f6fa;
}
QTabBar::tab {
    background: #dfe6e9;
    color: #2f3640;
    padding: 7px 20px;
    border: 1px solid #dcdde1;
    border-bottom: none;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    font-weight: bold;
}
QTabBar::tab:selected {
    background: #4078c0;
    color: white;
}
QGroupBox {
    border: 1px solid #dcdde1;
    border-radius: 4px;
    margin-top: 10px;
    padding: 10px 8px 8px 8px;
    font-weight: bold;
    color: #2f3640;
    background: white;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 0 6px;
}
QFrame#hline {
    background-color: #dcdde1;
    max-height: 1px;
}
"""


# =============================================================================
# IOC Commissioning Panel
#
# Builds an IOC_Commisioning_t packed binary frame from 4 input fields.
#
# C struct (packed):
#   typedef struct __attribute__((packed)) {
#       uint8_t  msgType;          // fixed = 0x10
#       uint8_t  u8CompartmentId;
#       uint8_t  u8Maxdock;
#       char     cIpAddress[16];   // null-padded to exactly 16 bytes
#       char     cSubnetMask[16];  // null-padded to exactly 16 bytes
#       uint16_t u16Crc;           // CRC16-CCITT of all bytes before CRC
#   } IOC_Commisioning_t;          // total = 37 bytes
#
# =============================================================================
MSG_TYPE_IOC = 0x10          # fixed msgType for commissioning frames
IOC_STRUCT_SIZE = 37         # 1+1+1+16+16+2 = 37 bytes

class IOCCommissioningPanel(QWidget):
    """
    Self-contained panel that builds and displays an IOC_Commisioning_t frame.
    Caller can optionally pass a send_callback(bytes) to transmit the frame.
    """

    def __init__(self, send_callback=None, parent=None):
        super().__init__(parent)
        self._send_cb = send_callback   # optional: TCPClient.send_frame
        self._last_frame: bytes = b""
        self._build_ui()

    # ── UI Construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(12)

        # ── Title ─────────────────────────────────────────────────────────────
        title = QLabel("IOC Commissioning  —  IOC_Commisioning_t")
        title.setFont(QFont("Segoe UI", 11, QFont.Bold))
        title.setStyleSheet("color: #1e3a6e;")
        root.addWidget(title)

        root.addWidget(self._hline())

        # ── Input Fields ──────────────────────────────────────────────────────
        fields_grp = QGroupBox("Configuration Fields")
        grid = QGridLayout(fields_grp)
        grid.setSpacing(10)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)

        # Row 0 — Compartment ID
        grid.addWidget(self._lbl("Compartment ID  (uint8)"), 0, 0)
        self.sb_comp_id = QSpinBox()
        self.sb_comp_id.setRange(1, 255)
        self.sb_comp_id.setValue(1)
        self.sb_comp_id.setFixedWidth(100)
        self.sb_comp_id.setToolTip("u8CompartmentId — 1 byte")
        grid.addWidget(self.sb_comp_id, 0, 1)

        # Row 0 — Max Dock
        grid.addWidget(self._lbl("Max Dock  (uint8)"), 0, 2)
        self.sb_max_dock = QSpinBox()
        self.sb_max_dock.setRange(1, 6)
        self.sb_max_dock.setValue(3)
        self.sb_max_dock.setFixedWidth(100)
        self.sb_max_dock.setToolTip("u8Maxdock — 1 byte")
        grid.addWidget(self.sb_max_dock, 0, 3)

        # Row 1 — IP Address
        grid.addWidget(self._lbl("IP Address  (char[16])"), 1, 0)
        self.le_ip = QLineEdit("192.168.1.231")
        self.le_ip.setMaxLength(15)        # "xxx.xxx.xxx.xxx" = max 15 chars + null
        self.le_ip.setToolTip("cIpAddress — null-padded to 16 bytes in frame")
        grid.addWidget(self.le_ip, 1, 1)

        # Row 1 — Subnet Mask
        grid.addWidget(self._lbl("Subnet Mask  (char[16])"), 1, 2)
        self.le_subnet = QLineEdit("255.255.255.0")
        self.le_subnet.setMaxLength(15)
        self.le_subnet.setToolTip("cSubnetMask — null-padded to 16 bytes in frame")
        grid.addWidget(self.le_subnet, 1, 3)

        # IP length hint
        self._ip_hint = QLabel("")
        self._ip_hint.setStyleSheet("color: #c0392b; font-size: 11px; font-weight: normal;")
        grid.addWidget(self._ip_hint, 2, 0, 1, 4)

        self.le_ip.textChanged.connect(self._check_ip_len)
        self.le_subnet.textChanged.connect(self._check_ip_len)

        root.addWidget(fields_grp)

        # ── Struct Layout (read-only reference) ───────────────────────────────
        layout_grp = QGroupBox("Packed Struct Layout  (37 bytes total)")
        layout_lay = QVBoxLayout(layout_grp)
        layout_lay.setContentsMargins(10, 10, 10, 10)

        layout_info = QLabel(
            "  Offset  Size  Field\n"
            "  ──────  ────  ───────────────────────────\n"
            "     0      1   msgType      = 0x10 (fixed)\n"
            "     1      1   u8CompartmentId\n"
            "     2      1   u8Maxdock\n"
            "     3     16   cIpAddress[16]  (null-padded)\n"
            "    19     16   cSubnetMask[16] (null-padded)\n"
            "    35      2   u16Crc  (CRC16-CCITT of bytes 0–34)"
        )
        layout_info.setFont(QFont("Consolas", 10))
        layout_info.setStyleSheet("color: #2f3640; font-weight: normal;")
        layout_lay.addWidget(layout_info)
        root.addWidget(layout_grp)

        # ── Action Buttons ────────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)

        self.btn_generate = QPushButton("Generate Frame")
        self.btn_generate.setObjectName("btnOrange")
        self.btn_generate.setToolTip("Pack fields into IOC_Commisioning_t binary")
        self.btn_generate.clicked.connect(self._on_generate)
        btn_row.addWidget(self.btn_generate)

        self.btn_send = QPushButton("Send Frame")
        self.btn_send.setObjectName("btnGreen")
        self.btn_send.setToolTip("Transmit last generated frame over TCP")
        self.btn_send.setEnabled(False)
        self.btn_send.clicked.connect(self._on_send)
        btn_row.addWidget(self.btn_send)

        self.btn_receive = QPushButton("Receive Frame")
        self.btn_receive.setObjectName("btnGreen")
        self.btn_receive.setToolTip("Send a READ frame (msgType=0x00) over TCP")
        self.btn_receive.setEnabled(self._send_cb is not None)
        self.btn_receive.clicked.connect(self._on_receive_frame)
        btn_row.addWidget(self.btn_receive)

        btn_row.addWidget(self._lbl("Msg Type:"))
        self.cb_msg_type = QComboBox()
        self.cb_msg_type.addItem("Write (0x01)", 0x01)
        self.cb_msg_type.addItem("Read  (0x00)", 0x00)
        self.cb_msg_type.setCurrentIndex(0)   # default = Write
        self.cb_msg_type.setToolTip("msgType byte at offset 0 of the frame")
        self.cb_msg_type.setFixedWidth(120)
        btn_row.addWidget(self.cb_msg_type)

        self.btn_copy = QPushButton("Copy Hex")
        self.btn_copy.setToolTip("Copy hex string to clipboard")
        self.btn_copy.setEnabled(False)
        self.btn_copy.clicked.connect(self._on_copy)
        btn_row.addWidget(self.btn_copy)

        self.btn_clear = QPushButton("Clear")
        self.btn_clear.clicked.connect(self._on_clear)
        btn_row.addWidget(self.btn_clear)

        btn_row.addStretch()
        root.addLayout(btn_row)

        # ── Output Display ────────────────────────────────────────────────────
        out_grp = QGroupBox("Frame Output")
        out_lay = QVBoxLayout(out_grp)
        out_lay.setContentsMargins(8, 10, 8, 8)
        out_lay.setSpacing(6)

        out_lay.addWidget(self._lbl("Hex String  (space-separated bytes):"))
        self.out_hex = QLineEdit()
        self.out_hex.setReadOnly(True)
        self.out_hex.setFont(QFont("Consolas", 10))
        self.out_hex.setPlaceholderText("press Generate Frame")
        out_lay.addWidget(self.out_hex)

        out_lay.addWidget(self._lbl("Raw Bytes  (Python bytes literal):"))
        self.out_raw = QLineEdit()
        self.out_raw.setReadOnly(True)
        self.out_raw.setFont(QFont("Consolas", 10))
        self.out_raw.setPlaceholderText("press Generate Frame")
        out_lay.addWidget(self.out_raw)

        out_lay.addWidget(self._lbl("Field Decode (verification):"))
        self.out_decode = QTextEdit()
        self.out_decode.setReadOnly(True)
        self.out_decode.setMaximumHeight(130)
        self.out_decode.setFont(QFont("Consolas", 10))
        out_lay.addWidget(self.out_decode)

        root.addWidget(out_grp)
        root.addStretch()

    # ── Helpers ───────────────────────────────────────────────────────────────
    @staticmethod
    def _lbl(text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setStyleSheet("font-weight: bold; color: #2f3640;")
        return lbl

    @staticmethod
    def _hline() -> QFrame:
        f = QFrame()
        f.setObjectName("hline")
        f.setFrameShape(QFrame.HLine)
        f.setFixedHeight(1)
        return f

    def _check_ip_len(self):
        warn = []
        if len(self.le_ip.text()) > 15:
            warn.append("IP too long (max 15 chars)")
        if len(self.le_subnet.text()) > 15:
            warn.append("Subnet too long (max 15 chars)")
        self._ip_hint.setText("  ⚠ " + "  |  ".join(warn) if warn else "")

    # ── Frame Builder ─────────────────────────────────────────────────────────
    def build_frame(self, msg_type: int = None) -> bytes:
        """
        Pack IOC_Commisioning_t:
            uint8   msgType        — from dropdown (0x00=Read, 0x01=Write), or override
            uint8   u8CompartmentId
            uint8   u8Maxdock
            char[16] cIpAddress    (null-padded to exactly 16 bytes)
            char[16] cSubnetMask   (null-padded to exactly 16 bytes)
            uint16  u16Crc         (CRC16-CCITT over bytes 0..34)
        """
        if msg_type is None:
            msg_type = self.cb_msg_type.currentData()

        comp_id  = self.sb_comp_id.value()  & 0xFF
        max_dock = self.sb_max_dock.value() & 0xFF

        ip_raw  = self.le_ip.text()[:15].encode("ascii")
        sn_raw  = self.le_subnet.text()[:15].encode("ascii")

        # Pad to exactly 16 bytes (including null terminator)
        ip_field = ip_raw + b"\x00" * (16 - len(ip_raw))
        sn_field = sn_raw + b"\x00" * (16 - len(sn_raw))

        body = struct.pack("B B B 16s 16s",
                           msg_type, comp_id, max_dock,
                           ip_field, sn_field)

        crc   = calculate_crc(body)
        frame = body + struct.pack(">H", crc)

        assert len(frame) == IOC_STRUCT_SIZE, \
            f"Frame size mismatch: {len(frame)} != {IOC_STRUCT_SIZE}"
        return frame

    # ── Decode (verification) ─────────────────────────────────────────────────
    @staticmethod
    def decode_frame(frame: bytes) -> str:
        """Parse a raw IOC_Commisioning_t frame and return human-readable text."""
        if len(frame) < IOC_STRUCT_SIZE:
            return f"Frame too short: {len(frame)} bytes (expected {IOC_STRUCT_SIZE})"

        msg_type  = frame[0]
        comp_id   = frame[1]
        max_dock  = frame[2]
        ip_bytes  = frame[3:19]
        sn_bytes  = frame[19:35]
        crc_rx    = struct.unpack(">H", frame[35:37])[0]
        crc_calc  = calculate_crc(frame[:35])

        ip_str = ip_bytes.rstrip(b"\x00").decode("ascii", errors="replace")
        sn_str = sn_bytes.rstrip(b"\x00").decode("ascii", errors="replace")
        crc_ok = "✔ OK" if crc_rx == crc_calc else f"✘ MISMATCH (calc=0x{crc_calc:04X})"

        lines = [
            f"  msgType        = 0x{msg_type:02X}",
            f"  u8CompartmentId= {comp_id}",
            f"  u8Maxdock      = {max_dock}",
            f"  cIpAddress     = \"{ip_str}\"  (raw: {ip_bytes.hex(' ')})",
            f"  cSubnetMask    = \"{sn_str}\"  (raw: {sn_bytes.hex(' ')})",
            f"  u16Crc         = 0x{crc_rx:04X}  {crc_ok}",
            f"  Total size     = {len(frame)} bytes",
        ]
        return "\n".join(lines)

    # ── Button Handlers ───────────────────────────────────────────────────────
    def _on_generate(self):
        if len(self.le_ip.text()) > 15 or len(self.le_subnet.text()) > 15:
            self._ip_hint.setText("  ⚠ Fix field lengths before generating.")
            return

        try:
            frame = self.build_frame()
        except Exception as e:
            self.out_decode.setPlainText(f"Build error: {e}")
            return

        self._last_frame = frame

        hex_str = frame.hex(" ").upper()
        raw_str = repr(frame)

        self.out_hex.setText(hex_str)
        self.out_raw.setText(raw_str)
        self.out_decode.setPlainText(self.decode_frame(frame))

        self.btn_send.setEnabled(self._send_cb is not None)
        self.btn_copy.setEnabled(True)

    def _on_send(self):
        if not self._last_frame or self._send_cb is None:
            return
        self._send_cb(self._last_frame)

    def _on_receive_frame(self):
        """Build and send a READ frame (msgType=0x00) using current field values."""
        if self._send_cb is None:
            return
        if len(self.le_ip.text()) > 15 or len(self.le_subnet.text()) > 15:
            self._ip_hint.setText("  ⚠ Fix field lengths before sending.")
            return
        try:
            frame = self.build_frame(msg_type=0x00)
        except Exception as e:
            self.out_decode.setPlainText(f"Build error: {e}")
            return
        self._send_cb(frame)

    def _on_copy(self):
        if self._last_frame:
            QApplication.clipboard().setText(self._last_frame.hex(" ").upper())

    def _on_clear(self):
        self._last_frame = b""
        self.out_hex.clear()
        self.out_raw.clear()
        self.out_decode.clear()
        self.btn_send.setEnabled(False)
        self.btn_copy.setEnabled(False)


# =============================================================================
# Main TCP Client  (original functionality preserved + commissioning tab added)
# =============================================================================
class TCPClient(QWidget):

    def __init__(self):
        super().__init__()
        self.sock   = None
        self.worker = None
        self.uid    = 1
        self.history: list = []
        self._build_ui()

    # ── UI Construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        self.setWindowTitle("Battery Swapping TCP Client")
        self.resize(900, 680)

        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # ── Connection bar (always visible) ───────────────────────────────────
        conn_grp = QGroupBox("Connection")
        conn_lay = QHBoxLayout(conn_grp)
        conn_lay.setSpacing(8)

        self.le_ip   = QLineEdit("192.168.1.231")
        self.le_port = QLineEdit("8888")
        self.le_dock = QLineEdit("1")
        self.le_comp = QLineEdit("1")

        self.le_ip.setFixedWidth(140)
        self.le_port.setFixedWidth(70)
        self.le_dock.setFixedWidth(50)
        self.le_comp.setFixedWidth(50)

        self.btn_connect    = QPushButton("Connect")
        self.btn_connect.setObjectName("btnGreen")
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setObjectName("btnRed")

        for lbl, w in [("IP", self.le_ip), ("Port", self.le_port),
                        ("Dock", self.le_dock), ("Comp", self.le_comp)]:
            conn_lay.addWidget(QLabel(lbl))
            conn_lay.addWidget(w)

        conn_lay.addWidget(self.btn_connect)
        conn_lay.addWidget(self.btn_disconnect)
        conn_lay.addStretch()

        # Connection status dot
        self._dot = QLabel("●")
        self._dot.setStyleSheet("color: #c0392b; font-size: 18px;")
        conn_lay.addWidget(self._dot)

        root.addWidget(conn_grp)

        # ── Tab widget ────────────────────────────────────────────────────────
        self.tabs = QTabWidget()

        # Tab 1 — IO Control (original commands)
        self.tabs.addTab(self._build_io_tab(), "  IO Control  ")

        # Tab 2 — IOC Commissioning (new panel)
        self._comm_panel = IOCCommissioningPanel(
            send_callback=self.send_frame
        )
        self.tabs.addTab(self._comm_panel, "  IOC Commissioning  ")

        root.addWidget(self.tabs, 1)

        # ── Console (shared, always visible) ──────────────────────────────────
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumHeight(160)
        self.console.setFont(QFont("Consolas", 10))
        root.addWidget(self.console)

        # ── Log toolbar ───────────────────────────────────────────────────────
        log_row = QHBoxLayout()
        btn_save  = QPushButton("Save Log")
        btn_clear = QPushButton("Clear Log")
        btn_save.clicked.connect(self.save_log)
        btn_clear.clicked.connect(self.console.clear)
        log_row.addStretch()
        log_row.addWidget(btn_save)
        log_row.addWidget(btn_clear)
        root.addLayout(log_row)

        # ── Signal / button wiring ────────────────────────────────────────────
        self.btn_connect.clicked.connect(self.connect_server)
        self.btn_disconnect.clicked.connect(self.disconnect_server)

    # ── IO Control Tab ────────────────────────────────────────────────────────
    def _build_io_tab(self) -> QWidget:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(10)

        # GPIO
        gpio_grp = QGroupBox("GPIO")
        gpio_lay = QHBoxLayout(gpio_grp)
        self.gpio_port = QLineEdit()
        self.gpio_port.setPlaceholderText("Port number")
        self.gpio_val  = QLineEdit()
        self.gpio_val.setPlaceholderText("Write value (0/1)")
        btn_gpio_r = QPushButton("Read")
        btn_gpio_w = QPushButton("Write")
        for w in [QLabel("Port:"), self.gpio_port, QLabel("Value:"),
                  self.gpio_val, btn_gpio_r, btn_gpio_w]:
            gpio_lay.addWidget(w)
        gpio_lay.addStretch()
        lay.addWidget(gpio_grp)

        # Charging
        chg_grp = QGroupBox("Charging Control")
        chg_lay = QHBoxLayout(chg_grp)
        btn_start = QPushButton("Start Charging")
        btn_start.setObjectName("btnGreen")
        btn_stop  = QPushButton("Stop Charging")
        btn_stop.setObjectName("btnRed")
        chg_lay.addWidget(btn_start)
        chg_lay.addWidget(btn_stop)
        chg_lay.addStretch()
        lay.addWidget(chg_grp)

        # Analog
        ana_grp = QGroupBox("Analog Read")
        ana_lay = QHBoxLayout(ana_grp)
        self.le_channel = QLineEdit()
        self.le_channel.setPlaceholderText("Channel (1–20)")
        self.le_channel.setFixedWidth(140)
        btn_ana = QPushButton("Read Analog")
        ana_lay.addWidget(QLabel("Channel:"))
        ana_lay.addWidget(self.le_channel)
        ana_lay.addWidget(btn_ana)
        ana_lay.addStretch()
        lay.addWidget(ana_grp)

        # System
        sys_grp = QGroupBox("System")
        sys_lay = QHBoxLayout(sys_grp)
        btn_boot  = QPushButton("Boot Mode")
        btn_reset = QPushButton("Soft Reset")
        btn_comm  = QPushButton("Commissioning Mode")
        btn_boot.setObjectName("btnOrange")
        btn_reset.setObjectName("btnRed")
        btn_comm.setObjectName("btnOrange")
        sys_lay.addWidget(btn_boot)
        sys_lay.addWidget(btn_reset)
        sys_lay.addWidget(btn_comm)
        sys_lay.addStretch()
        lay.addWidget(sys_grp)

        lay.addStretch()

        # Wiring
        btn_gpio_r.clicked.connect(self.gpio_read)
        btn_gpio_w.clicked.connect(self.gpio_write)
        btn_start.clicked.connect(self.charging_start)
        btn_stop.clicked.connect(self.charging_stop)
        btn_ana.clicked.connect(self.analog_read)
        btn_boot.clicked.connect(self.boot_mode)
        btn_reset.clicked.connect(self.soft_reset)
        btn_comm.clicked.connect(self.commissioning_mode)

        return tab

    # ── Logging ───────────────────────────────────────────────────────────────
    def _ts(self) -> str:
        return datetime.datetime.now().strftime("%H:%M:%S")

    def log_tx(self, msg: str):
        self.console.append(
            f"<span style='color:#1565c0'>[{self._ts()}] TX: {msg}</span>")
        self.history.append(f"[{self._ts()}] TX {msg}")

    def log_rx(self, msg: str):
        self.console.append(
            f"<span style='color:#1b5e20'>[{self._ts()}] RX: {msg}</span>")
        self.history.append(f"[{self._ts()}] RX {msg}")

    def log_info(self, msg: str, color: str = "#c62828"):
        self.console.append(f"<span style='color:{color}'>{msg}</span>")

    # ── Connection ────────────────────────────────────────────────────────────
    def connect_server(self):
        try:
            ip   = self.le_ip.text().strip()
            port = int(self.le_port.text().strip())
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((ip, port))
            self.sock.settimeout(None)
            self._set_connected(True)
            self.log_info(f"Connected to {ip}:{port}", "#1b5e20")

            self.worker = TCPWorker(self.sock)
            self.worker.received.connect(self.on_receive)
            self.worker.disconnected.connect(self.on_disconnect)
            self.worker.start()

        except Exception as e:
            self.log_info(f"Connection error: {e}")

    def disconnect_server(self):
        try:
            if self.worker:
                self.worker.stop()
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        finally:
            self.sock = None
            self._set_connected(False)
            self.log_info("Disconnected")

    def on_disconnect(self):
        self.sock = None
        self._set_connected(False)
        self.log_info("Server disconnected")

    def _set_connected(self, connected: bool):
        color = "#27ae60" if connected else "#c0392b"
        self._dot.setStyleSheet(f"color: {color}; font-size: 18px;")

    # ── Send Frame ────────────────────────────────────────────────────────────
    def send_frame(self, frame: bytes):
        if not self.sock:
            self.log_info("Not connected — cannot send")
            return
        try:
            self.sock.sendall(frame)
            self.log_tx(frame.hex(" ").upper())
        except Exception as e:
            self.log_info(f"Send error: {e}")

    # ── Receive ───────────────────────────────────────────────────────────────
    def on_receive(self, data: bytes):
        self.log_rx(data.hex(" ").upper())
        self.log_info(self._parse_frame(data), "#4a4a4a")

    @staticmethod
    def _parse_frame(data: bytes) -> str:
        try:
            uid, msg, comp, dock, cmd, plen = struct.unpack(">I H B B B B", data[:10])
            payload  = data[10:10 + plen]
            crc_rx   = struct.unpack(">H", data[-2:])[0]
            crc_calc = calculate_crc(data[:-2])
            ok = "CRC OK" if crc_rx == crc_calc else f"CRC FAIL (calc=0x{crc_calc:04X})"
            return (f"  → UID={uid}  CMD={cmd}  DOCK={dock}  COMP={comp}"
                    f"  PAYLOAD={payload.hex()}  {ok}")
        except Exception:
            return "  → Parse error"

    # ── IO Commands ───────────────────────────────────────────────────────────
    def _dock(self) -> int:
        try:    return int(self.le_dock.text())
        except: return 1

    def _comp(self) -> int:
        try:    return int(self.le_comp.text())
        except: return 1

    def _next_uid(self) -> int:
        uid = self.uid
        self.uid = (self.uid + 1) & 0xFFFFFFFF
        return uid

    def gpio_read(self):
        try:
            port = int(self.gpio_port.text())
        except ValueError:
            self.log_info("GPIO: invalid port number"); return
        payload = struct.pack(">BH", 0, port)
        self.send_frame(Protocol.build(self._next_uid(), self._dock(),
                                       self._comp(), Protocol.CMD_GPIO, payload))

    def gpio_write(self):
        try:
            port  = int(self.gpio_port.text())
            value = int(self.gpio_val.text())
        except ValueError:
            self.log_info("GPIO: invalid port or value"); return
        payload = struct.pack(">BHB", 1, port, value)
        self.send_frame(Protocol.build(self._next_uid(), self._dock(),
                                       self._comp(), Protocol.CMD_GPIO, payload))

    def charging_start(self):
        self.send_frame(Protocol.build(self._next_uid(), self._dock(),
                                       self._comp(), Protocol.CMD_CHARGING, b'\x01'))

    def charging_stop(self):
        self.send_frame(Protocol.build(self._next_uid(), self._dock(),
                                       self._comp(), Protocol.CMD_CHARGING, b'\x00'))

    def analog_read(self):
        try:
            ch = int(self.le_channel.text())
        except ValueError:
            self.log_info("Analog: invalid channel"); return
        payload = struct.pack(">B", ch)
        self.send_frame(Protocol.build(self._next_uid(), self._dock(),
                                       self._comp(), Protocol.CMD_ANALOG, payload))

    def boot_mode(self):
        self.send_frame(Protocol.build(self._next_uid(), self._dock(),
                                       self._comp(), Protocol.CMD_BOOT, b'\x01'))

    def soft_reset(self):
        self.send_frame(Protocol.build(self._next_uid(), self._dock(),
                                       self._comp(), Protocol.CMD_RESET, b'\x01'))

    def commissioning_mode(self):
        self.send_frame(Protocol.build(self._next_uid(), self._dock(),
                                       self._comp(), Protocol.CMD_COMMISSIONING_COMMAND, b'\x01'))

    # ── Save Log ──────────────────────────────────────────────────────────────
    def save_log(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Log", "",
                                              "Text Files (*.txt)")
        if not path:
            return
        with open(path, "w") as f:
            f.writelines(line + "\n" for line in self.history)
        self.log_info("Log saved.", "#1b5e20")


# =============================================================================
# Entry Point
# =============================================================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    win = TCPClient()
    win.show()
    sys.exit(app.exec_())