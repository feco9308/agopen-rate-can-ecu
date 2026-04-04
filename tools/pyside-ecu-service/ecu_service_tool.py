import socket
import struct
import sys
from dataclasses import dataclass

from PySide6.QtCore import Qt, QDateTime, QTimer
from PySide6.QtNetwork import QHostAddress, QUdpSocket
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSpinBox,
    QDoubleSpinBox,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


UDP_SEND_PORT = 28888
UDP_LISTEN_PORT = 30001
DEFAULT_TARGET_IP = "192.168.1.255"
PACKET_LEN = 11


class Pgn:
    ECU_CFG_GET = 32800
    ECU_CFG_SET = 32801
    ECU_CFG_SAVE = 32802
    ECU_CFG_LOAD = 32803
    ECU_CFG_STATUS = 32804
    ECU_DIAG_CONTROL = 32805
    ECU_DIAG_STATUS = 32806
    ECU_DIAG_SENSOR = 32807
    ECU_DIAG_NODE_SUMMARY = 32808
    ECU_DIAG_NODE_DETAIL_A = 32809
    ECU_DIAG_NODE_DETAIL_B = 32810
    ECU_DIAG_NODE_DETAIL_REQ = 32811

    NODE_DISCOVER = 32900
    NODE_UID_A = 32901
    NODE_UID_B = 32902
    NODE_ASSIGN = 32903
    NODE_SAVE_CFG = 32904
    NODE_ACK = 32905
    NODE_TEST_SPIN = 32906
    NODE_DIAG_REQ = 32907
    NODE_DIAG_RESP_A = 32908
    NODE_DIAG_RESP_B = 32909
    NODE_REBOOT = 32910
    NODE_IDENTIFY = 32911
    NODE_CFG_READ = 32912
    NODE_CFG_RESP = 32913
    NODE_SET_CAN_SOURCE = 32914


class Block:
    MACHINE = 0
    DRIVE = 1
    DIAG = 2
    CHANNEL = 3
    NETWORK = 4


def packet_crc(packet_without_crc: bytes) -> int:
    return sum(bytes(packet_without_crc)) & 0xFF


def build_packet(pgn: int, payload: bytes) -> bytes:
    if len(payload) != 8:
        raise ValueError("Payload must be exactly 8 bytes")
    packet = bytearray(10)
    packet[0] = pgn & 0xFF
    packet[1] = (pgn >> 8) & 0xFF
    packet[2:10] = payload
    packet.append(packet_crc(packet))
    return bytes(packet)


def parse_u16(lo: int, hi: int) -> int:
    return lo | (hi << 8)


def parse_i16(lo: int, hi: int) -> int:
    return struct.unpack("<h", bytes([lo, hi]))[0]


def parse_u32(data: bytes) -> int:
    return struct.unpack("<I", data)[0]


@dataclass
class SensorDiag:
    mode: int = 0
    base_rpm: float = 0.0
    target_upm: float = 0.0
    online_nodes: int = 0
    avg_rpm: float = 0.0
    total_rpm: float = 0.0
    avg_pos: int = 0


@dataclass
class NodeDiag:
    sensor: int = 0
    node_id: int = 0
    status_flags: int = 0
    error_code: int = 0
    actual_rpm: float = 0.0
    actual_pos: int = 0
    bus_voltage: float = 0.0
    motor_current: float = 0.0
    controller_temp: int = 0
    motor_temp: int = 0
    warning_flags: int = 0
    fault_flags: int = 0


class EcuServiceTool(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Vetogep ECU Service Tool")
        self.resize(1200, 820)

        self.sensor_diags = {0: SensorDiag(), 1: SensorDiag(), 2: SensorDiag(), 3: SensorDiag()}
        self.node_diags: dict[tuple[int, int], NodeDiag] = {}
        self.current_target_ip = DEFAULT_TARGET_IP
        self.auto_detail_sensor = 0
        self.auto_detail_node = 1

        self.udp = QUdpSocket(self)
        self.udp.readyRead.connect(self.on_udp_ready_read)
        self.rx_bound = False

        self.auto_detail_timer = QTimer(self)
        self.auto_detail_timer.timeout.connect(self.request_next_node_details)

        central = QWidget()
        root = QVBoxLayout(central)

        root.addLayout(self.build_connection_bar())

        self.tabs = QTabWidget()
        self.tabs.addTab(self.build_config_tab(), "ECU Config")
        self.tabs.addTab(self.build_diag_tab(), "Diag")
        self.tabs.addTab(self.build_node_tab(), "Node Service")
        root.addWidget(self.tabs)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1000)
        root.addWidget(self.log, 1)

        self.setCentralWidget(central)
        self.try_bind_udp_listener()

    def build_connection_bar(self) -> QHBoxLayout:
        layout = QHBoxLayout()
        layout.addWidget(QLabel("ECU IP / Broadcast"))

        self.target_ip_edit = QLineEdit(DEFAULT_TARGET_IP)
        self.target_ip_edit.setMinimumWidth(180)
        layout.addWidget(self.target_ip_edit)

        self.apply_target_btn = QPushButton("Apply Target")
        self.apply_target_btn.clicked.connect(self.on_apply_target)
        layout.addWidget(self.apply_target_btn)

        layout.addSpacing(16)
        self.rx_status_label = QLabel("RX: not started")
        layout.addWidget(self.rx_status_label)

        self.retry_bind_btn = QPushButton("Retry RX Bind")
        self.retry_bind_btn.clicked.connect(self.try_bind_udp_listener)
        layout.addWidget(self.retry_bind_btn)

        layout.addStretch(1)
        return layout

    def build_config_tab(self) -> QWidget:
        page = QWidget()
        grid = QGridLayout(page)

        machine_box = QGroupBox("Machine Config")
        machine_form = QFormLayout(machine_box)

        self.active_sensor_spin = QSpinBox()
        self.active_sensor_spin.setRange(1, 4)
        self.active_sensor_spin.setValue(4)

        self.row_count_spin = QSpinBox()
        self.row_count_spin.setRange(1, 16)
        self.row_count_spin.setValue(6)

        self.holes_spin = QSpinBox()
        self.holes_spin.setRange(1, 200)
        self.holes_spin.setValue(26)

        self.upm_scale_spin = QDoubleSpinBox()
        self.upm_scale_spin.setDecimals(1)
        self.upm_scale_spin.setRange(0.1, 1000.0)
        self.upm_scale_spin.setValue(100.0)

        machine_form.addRow("Active Sensors", self.active_sensor_spin)
        machine_form.addRow("Configured Rows", self.row_count_spin)
        machine_form.addRow("Holes / Rev", self.holes_spin)
        machine_form.addRow("UPM Scale", self.upm_scale_spin)

        machine_buttons = QHBoxLayout()
        btn_get_machine = QPushButton("Get")
        btn_get_machine.clicked.connect(lambda: self.send_cfg_get(Block.MACHINE, 0))
        btn_set_machine = QPushButton("Set")
        btn_set_machine.clicked.connect(self.send_machine_config)
        machine_buttons.addWidget(btn_get_machine)
        machine_buttons.addWidget(btn_set_machine)
        machine_form.addRow(machine_buttons)

        drive_box = QGroupBox("Drive Config")
        drive_form = QFormLayout(drive_box)

        self.trim_limit_spin = QDoubleSpinBox()
        self.trim_limit_spin.setDecimals(1)
        self.trim_limit_spin.setRange(0.1, 1000.0)
        self.trim_limit_spin.setValue(200.0)

        self.position_kp_spin = QDoubleSpinBox()
        self.position_kp_spin.setDecimals(3)
        self.position_kp_spin.setRange(0.001, 10.0)
        self.position_kp_spin.setValue(0.5)

        drive_form.addRow("Trim Limit RPM", self.trim_limit_spin)
        drive_form.addRow("Position Kp", self.position_kp_spin)

        drive_buttons = QHBoxLayout()
        btn_get_drive = QPushButton("Get")
        btn_get_drive.clicked.connect(lambda: self.send_cfg_get(Block.DRIVE, 0))
        btn_set_drive = QPushButton("Set")
        btn_set_drive.clicked.connect(self.send_drive_config)
        drive_buttons.addWidget(btn_get_drive)
        drive_buttons.addWidget(btn_set_drive)
        drive_form.addRow(drive_buttons)

        channel_box = QGroupBox("Channel Ratio Config")
        channel_form = QFormLayout(channel_box)

        self.channel_cfg_spin = QSpinBox()
        self.channel_cfg_spin.setRange(0, 3)

        self.channel_drive_ratio_spin = QDoubleSpinBox()
        self.channel_drive_ratio_spin.setDecimals(2)
        self.channel_drive_ratio_spin.setRange(0.01, 100.0)
        self.channel_drive_ratio_spin.setValue(1.0)

        self.channel_motor_ratio_spin = QDoubleSpinBox()
        self.channel_motor_ratio_spin.setDecimals(2)
        self.channel_motor_ratio_spin.setRange(0.01, 100.0)
        self.channel_motor_ratio_spin.setValue(2.0)

        channel_form.addRow("Sensor Channel", self.channel_cfg_spin)
        channel_form.addRow("Drive Ratio", self.channel_drive_ratio_spin)
        channel_form.addRow("Motor Ratio", self.channel_motor_ratio_spin)

        channel_buttons = QHBoxLayout()
        btn_get_channel = QPushButton("Get")
        btn_get_channel.clicked.connect(lambda: self.send_cfg_get(Block.CHANNEL, self.channel_cfg_spin.value()))
        btn_set_channel = QPushButton("Set")
        btn_set_channel.clicked.connect(self.send_channel_config)
        channel_buttons.addWidget(btn_get_channel)
        channel_buttons.addWidget(btn_set_channel)
        channel_form.addRow(channel_buttons)

        diag_box = QGroupBox("Diag Config")
        diag_form = QFormLayout(diag_box)

        self.diag_enable_check = QCheckBox("Enable Diagnostics")
        self.diag_stream_check = QCheckBox("Enable Stream")

        self.diag_period_spin = QSpinBox()
        self.diag_period_spin.setRange(10, 5000)
        self.diag_period_spin.setSingleStep(10)
        self.diag_period_spin.setValue(200)

        self.diag_detail_combo = QComboBox()
        self.diag_detail_combo.addItems(["Basic", "Extended"])

        diag_form.addRow(self.diag_enable_check)
        diag_form.addRow(self.diag_stream_check)
        diag_form.addRow("Diag Period ms", self.diag_period_spin)
        diag_form.addRow("Diag Detail", self.diag_detail_combo)

        diag_buttons = QHBoxLayout()
        btn_get_diag = QPushButton("Get")
        btn_get_diag.clicked.connect(lambda: self.send_cfg_get(Block.DIAG, 0))
        btn_set_diag = QPushButton("Set")
        btn_set_diag.clicked.connect(self.send_diag_config)
        diag_buttons.addWidget(btn_get_diag)
        diag_buttons.addWidget(btn_set_diag)
        diag_form.addRow(diag_buttons)

        network_box = QGroupBox("Network Config")
        network_form = QFormLayout(network_box)

        self.ip_last_octet_spin = QSpinBox()
        self.ip_last_octet_spin.setRange(1, 254)
        self.ip_last_octet_spin.setValue(200)

        self.module_id_spin = QSpinBox()
        self.module_id_spin.setRange(0, 15)
        self.module_id_spin.setValue(0)

        network_form.addRow("IP Last Octet", self.ip_last_octet_spin)
        network_form.addRow("Module ID", self.module_id_spin)

        network_buttons = QHBoxLayout()
        btn_get_net = QPushButton("Get")
        btn_get_net.clicked.connect(lambda: self.send_cfg_get(Block.NETWORK, 0))
        btn_set_net = QPushButton("Set")
        btn_set_net.clicked.connect(self.send_network_config)
        network_buttons.addWidget(btn_get_net)
        network_buttons.addWidget(btn_set_net)
        network_form.addRow(network_buttons)

        persist_box = QGroupBox("Persistence")
        persist_layout = QHBoxLayout(persist_box)
        btn_save = QPushButton("Save To ECU")
        btn_save.clicked.connect(self.send_cfg_save)
        btn_load = QPushButton("Load From ECU")
        btn_load.clicked.connect(self.send_cfg_load)
        persist_layout.addWidget(btn_save)
        persist_layout.addWidget(btn_load)

        grid.addWidget(machine_box, 0, 0)
        grid.addWidget(drive_box, 0, 1)
        grid.addWidget(channel_box, 1, 0)
        grid.addWidget(diag_box, 1, 1)
        grid.addWidget(network_box, 2, 0)
        grid.addWidget(persist_box, 2, 1)
        return page

    def build_diag_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        controls_box = QGroupBox("Diag Control")
        controls = QGridLayout(controls_box)

        self.diag_ctrl_enable = QCheckBox("Enable")
        self.diag_ctrl_stream = QCheckBox("Stream")
        self.diag_sensor_checks = []
        for sensor in range(4):
            check = QCheckBox(f"S{sensor}")
            check.setChecked(True)
            self.diag_sensor_checks.append(check)
            controls.addWidget(check, 0, 2 + sensor)

        self.diag_node_mask_edit = QLineEdit("FFFF")
        self.diag_period_ctrl_spin = QSpinBox()
        self.diag_period_ctrl_spin.setRange(10, 5000)
        self.diag_period_ctrl_spin.setSingleStep(10)
        self.diag_period_ctrl_spin.setValue(200)
        self.diag_detail_ctrl_combo = QComboBox()
        self.diag_detail_ctrl_combo.addItems(["Basic", "Extended"])
        self.diag_ctrl_enable.toggled.connect(self.send_diag_control)
        self.diag_ctrl_stream.toggled.connect(self.send_diag_control)
        for check in self.diag_sensor_checks:
            check.toggled.connect(self.send_diag_control)
        self.diag_node_mask_edit.editingFinished.connect(self.send_diag_control)
        self.diag_period_ctrl_spin.valueChanged.connect(self.send_diag_control)
        self.diag_detail_ctrl_combo.currentIndexChanged.connect(self.send_diag_control)
        self.detail_sensor_spin = QSpinBox()
        self.detail_sensor_spin.setRange(0, 3)
        self.detail_node_spin = QSpinBox()
        self.detail_node_spin.setRange(1, 16)
        self.auto_detail_check = QCheckBox("Auto Node Details")
        self.auto_detail_check.toggled.connect(self.on_auto_detail_toggled)
        self.auto_detail_period_spin = QSpinBox()
        self.auto_detail_period_spin.setRange(200, 10000)
        self.auto_detail_period_spin.setSingleStep(100)
        self.auto_detail_period_spin.setValue(1000)

        btn_send_diag_ctrl = QPushButton("Send Diag Control")
        btn_send_diag_ctrl.clicked.connect(self.send_diag_control)
        btn_request_status = QPushButton("Request ECU Status")
        btn_request_status.clicked.connect(self.request_ecu_status)
        btn_request_node_detail = QPushButton("Request Node Details")
        btn_request_node_detail.clicked.connect(self.request_node_details)

        controls.addWidget(self.diag_ctrl_enable, 0, 0)
        controls.addWidget(self.diag_ctrl_stream, 0, 1)
        controls.addWidget(QLabel("Node Mask Hex"), 1, 0)
        controls.addWidget(self.diag_node_mask_edit, 1, 1)
        controls.addWidget(QLabel("Period ms"), 1, 2)
        controls.addWidget(self.diag_period_ctrl_spin, 1, 3)
        controls.addWidget(QLabel("Detail"), 1, 4)
        controls.addWidget(self.diag_detail_ctrl_combo, 1, 5)
        controls.addWidget(QLabel("Detail Sensor"), 2, 0)
        controls.addWidget(self.detail_sensor_spin, 2, 1)
        controls.addWidget(QLabel("Detail Node"), 2, 2)
        controls.addWidget(self.detail_node_spin, 2, 3)
        controls.addWidget(btn_request_node_detail, 2, 4, 1, 2)
        controls.addWidget(self.auto_detail_check, 3, 0, 1, 2)
        controls.addWidget(QLabel("Auto ms"), 3, 2)
        controls.addWidget(self.auto_detail_period_spin, 3, 3)
        controls.addWidget(btn_send_diag_ctrl, 4, 0, 1, 3)
        controls.addWidget(btn_request_status, 4, 3, 1, 3)

        self.sensor_table = QTableWidget(4, 7)
        self.sensor_table.setHorizontalHeaderLabels(
            ["Sensor", "Mode", "Base RPM", "Target UPM", "Online Nodes", "Avg RPM", "Total RPM"]
        )
        self.sensor_table.verticalHeader().setVisible(False)
        for row in range(4):
            self.sensor_table.setItem(row, 0, QTableWidgetItem(f"S{row}"))

        layout.addWidget(controls_box)
        layout.addWidget(self.sensor_table)

        self.node_table = QTableWidget(0, 12)
        self.node_table.setHorizontalHeaderLabels(
            [
                "Sensor",
                "Node",
                "Status",
                "Error",
                "Actual RPM",
                "Pos",
                "Bus V",
                "Current A",
                "Ctrl C",
                "Motor C",
                "Warnings",
                "Faults",
            ]
        )
        self.node_table.verticalHeader().setVisible(False)
        layout.addWidget(self.node_table)
        return page

    def build_node_tab(self) -> QWidget:
        page = QWidget()
        layout = QGridLayout(page)

        discover_box = QGroupBox("Discover")
        discover_form = QFormLayout(discover_box)
        self.discover_delay_spin = QSpinBox()
        self.discover_delay_spin.setRange(0, 255)
        self.discover_delay_spin.setValue(10)
        self.discover_uid_check = QCheckBox("Request UID")
        self.discover_uid_check.setChecked(True)
        self.discover_cfg_check = QCheckBox("Request CFG")
        btn_discover = QPushButton("Send Discover")
        btn_discover.clicked.connect(self.send_node_discover)
        discover_form.addRow("Delay Slots", self.discover_delay_spin)
        discover_form.addRow(self.discover_uid_check)
        discover_form.addRow(self.discover_cfg_check)
        discover_form.addRow(btn_discover)

        target_box = QGroupBox("Target Node")
        target_form = QFormLayout(target_box)
        self.uid_edit = QLineEdit("00000000")
        self.sensor_channel_spin = QSpinBox()
        self.sensor_channel_spin.setRange(0, 3)
        self.node_id_spin = QSpinBox()
        self.node_id_spin.setRange(1, 16)
        self.can_profile_spin = QSpinBox()
        self.can_profile_spin.setRange(0, 3)
        target_form.addRow("UID32 Hex", self.uid_edit)
        target_form.addRow("Sensor Channel", self.sensor_channel_spin)
        target_form.addRow("Node ID", self.node_id_spin)
        target_form.addRow("CAN Profile", self.can_profile_spin)

        assign_buttons = QHBoxLayout()
        btn_assign = QPushButton("Assign")
        btn_assign.clicked.connect(self.send_node_assign)
        btn_save_cfg = QPushButton("Save CFG")
        btn_save_cfg.clicked.connect(self.send_node_save)
        assign_buttons.addWidget(btn_assign)
        assign_buttons.addWidget(btn_save_cfg)
        target_form.addRow(assign_buttons)

        service_box = QGroupBox("Service Actions")
        service_form = QFormLayout(service_box)
        self.test_rpm_spin = QDoubleSpinBox()
        self.test_rpm_spin.setRange(-1000.0, 1000.0)
        self.test_rpm_spin.setDecimals(1)
        self.test_rpm_spin.setValue(50.0)
        self.test_duration_spin = QSpinBox()
        self.test_duration_spin.setRange(1, 60)
        self.test_duration_spin.setValue(3)
        self.identify_mode_combo = QComboBox()
        self.identify_mode_combo.addItems(["LED Blink", "Short Jog"])

        btn_test_spin = QPushButton("Test Spin")
        btn_test_spin.clicked.connect(self.send_node_test_spin)
        btn_diag_req = QPushButton("Diag Req")
        btn_diag_req.clicked.connect(self.send_node_diag_req)
        btn_cfg_read = QPushButton("CFG Read")
        btn_cfg_read.clicked.connect(self.send_node_cfg_read)
        btn_set_can_src = QPushButton("Set CAN Source")
        btn_set_can_src.clicked.connect(self.send_node_set_can_source)
        btn_identify = QPushButton("Identify")
        btn_identify.clicked.connect(self.send_node_identify)
        btn_reboot = QPushButton("Reboot")
        btn_reboot.clicked.connect(self.send_node_reboot)

        service_form.addRow("Test RPM", self.test_rpm_spin)
        service_form.addRow("Duration s", self.test_duration_spin)
        service_form.addRow("Identify Mode", self.identify_mode_combo)
        service_form.addRow(btn_test_spin)
        service_form.addRow(btn_diag_req)
        service_form.addRow(btn_cfg_read)
        service_form.addRow(btn_set_can_src)
        service_form.addRow(btn_identify)
        service_form.addRow(btn_reboot)

        layout.addWidget(discover_box, 0, 0)
        layout.addWidget(target_box, 0, 1)
        layout.addWidget(service_box, 1, 0, 1, 2)
        return page

    def send_packet(self, pgn: int, payload: bytes) -> None:
        try:
            packet = build_packet(pgn, payload)
        except Exception as exc:
            QMessageBox.critical(self, "Packet Error", str(exc))
            return

        self.current_target_ip = self.target_ip_edit.text().strip() or DEFAULT_TARGET_IP
        sent = self.udp.writeDatagram(packet, QHostAddress(self.current_target_ip), UDP_SEND_PORT)
        self.log_line(f"TX PGN {pgn} -> {self.current_target_ip}:{UDP_SEND_PORT} ({sent} bytes)")

    def send_cfg_get(self, block_id: int, index: int) -> None:
        self.send_packet(Pgn.ECU_CFG_GET, bytes([block_id, index, 0, 0, 0, 0, 0, 0]))

    def send_machine_config(self) -> None:
        payload = bytearray(8)
        payload[0] = Block.MACHINE
        payload[1] = 0
        payload[2] = self.active_sensor_spin.value()
        payload[3] = self.row_count_spin.value()
        holes = self.holes_spin.value()
        scale_x10 = int(round(self.upm_scale_spin.value() * 10.0))
        payload[4] = holes & 0xFF
        payload[5] = (holes >> 8) & 0xFF
        payload[6] = scale_x10 & 0xFF
        payload[7] = (scale_x10 >> 8) & 0xFF
        self.send_packet(Pgn.ECU_CFG_SET, bytes(payload))

    def send_drive_config(self) -> None:
        payload = bytearray(8)
        payload[0] = Block.DRIVE
        payload[1] = 0
        trim_x10 = int(round(self.trim_limit_spin.value() * 10.0))
        kp_x1000 = int(round(self.position_kp_spin.value() * 1000.0))
        payload[2] = trim_x10 & 0xFF
        payload[3] = (trim_x10 >> 8) & 0xFF
        payload[4] = kp_x1000 & 0xFF
        payload[5] = (kp_x1000 >> 8) & 0xFF
        self.send_packet(Pgn.ECU_CFG_SET, bytes(payload))

    def send_channel_config(self) -> None:
        payload = bytearray(8)
        payload[0] = Block.CHANNEL
        payload[1] = self.channel_cfg_spin.value()
        drive_x100 = int(round(self.channel_drive_ratio_spin.value() * 100.0))
        motor_x100 = int(round(self.channel_motor_ratio_spin.value() * 100.0))
        payload[2] = drive_x100 & 0xFF
        payload[3] = (drive_x100 >> 8) & 0xFF
        payload[4] = motor_x100 & 0xFF
        payload[5] = (motor_x100 >> 8) & 0xFF
        self.send_packet(Pgn.ECU_CFG_SET, bytes(payload))

    def send_diag_config(self) -> None:
        payload = bytes([
            Block.DIAG,
            0,
            1 if self.diag_enable_check.isChecked() else 0,
            1 if self.diag_stream_check.isChecked() else 0,
            max(1, self.diag_period_spin.value() // 10),
            self.diag_detail_combo.currentIndex(),
            0,
            0,
        ])
        self.send_packet(Pgn.ECU_CFG_SET, payload)

    def send_network_config(self) -> None:
        payload = bytes([
            Block.NETWORK,
            0,
            self.ip_last_octet_spin.value(),
            self.module_id_spin.value(),
            0,
            0,
            0,
            0,
        ])
        self.send_packet(Pgn.ECU_CFG_SET, payload)

    def send_cfg_save(self) -> None:
        self.send_packet(Pgn.ECU_CFG_SAVE, bytes([0xA5, 0, 0, 0, 0, 0, 0, 0]))

    def send_cfg_load(self) -> None:
        self.send_packet(Pgn.ECU_CFG_LOAD, bytes([0x5A, 0, 0, 0, 0, 0, 0, 0]))

    def send_diag_control(self) -> None:
        sensor_mask = 0
        for idx, check in enumerate(self.diag_sensor_checks):
            if check.isChecked():
                sensor_mask |= (1 << idx)
        try:
            node_mask = int(self.diag_node_mask_edit.text().strip(), 16) & 0xFFFF
        except ValueError:
            QMessageBox.warning(self, "Node Mask", "A node mask hexadecimal legyen, pl. FFFF")
            return

        payload = bytearray(8)
        payload[0] = 1 if self.diag_ctrl_enable.isChecked() else 0
        payload[1] = 1 if self.diag_ctrl_stream.isChecked() else 0
        payload[2] = sensor_mask
        payload[3] = node_mask & 0xFF
        payload[4] = (node_mask >> 8) & 0xFF
        payload[5] = max(1, self.diag_period_ctrl_spin.value() // 10)
        payload[6] = self.diag_detail_ctrl_combo.currentIndex()
        payload[7] = 0
        self.send_packet(Pgn.ECU_DIAG_CONTROL, bytes(payload))

    def request_ecu_status(self) -> None:
        self.send_cfg_get(Block.MACHINE, 0)
        self.send_cfg_get(Block.DRIVE, 0)
        self.send_cfg_get(Block.DIAG, 0)
        self.send_cfg_get(Block.NETWORK, 0)
        for sensor_index in range(4):
            self.send_cfg_get(Block.CHANNEL, sensor_index)
        self.send_diag_control()

    def request_node_details(self) -> None:
        self.send_diag_control()
        self._send_node_detail_request(self.detail_sensor_spin.value(), self.detail_node_spin.value())

    def _selected_sensor_list(self) -> list[int]:
        sensors = [idx for idx, check in enumerate(self.diag_sensor_checks) if check.isChecked()]
        if not sensors:
            sensors = [self.detail_sensor_spin.value()]
        return sensors

    def _configured_row_count(self) -> int:
        return max(1, min(16, self.row_count_spin.value()))

    def _selected_node_list(self) -> list[int]:
        try:
            node_mask = int(self.diag_node_mask_edit.text().strip(), 16) & 0xFFFF
        except ValueError:
            node_mask = 0

        max_node = self._configured_row_count()
        nodes = [node_id for node_id in range(1, max_node + 1) if node_mask & (1 << (node_id - 1))]
        if not nodes:
            nodes = [min(self.detail_node_spin.value(), max_node)]
        return nodes

    def _send_node_detail_request(self, sensor: int, node_id: int) -> None:
        node_id = max(1, min(node_id, self._configured_row_count()))
        sensor_mask = 1 << sensor
        node_mask = 1 << (node_id - 1)
        payload = bytes([
            sensor_mask,
            node_mask & 0xFF,
            (node_mask >> 8) & 0xFF,
            0,
            0,
            0,
            0,
            0,
        ])
        self.log_line(
            f"Request node details sensor_mask=0x{sensor_mask:02X} node_mask=0x{node_mask:04X}"
        )
        self.send_packet(Pgn.ECU_DIAG_NODE_DETAIL_REQ, payload)

    def on_auto_detail_toggled(self, enabled: bool) -> None:
        if enabled:
            self.auto_detail_sensor = self.detail_sensor_spin.value()
            self.auto_detail_node = self.detail_node_spin.value()
            self.auto_detail_timer.start(self.auto_detail_period_spin.value())
            self.log_line(f"Auto node detail polling started ({self.auto_detail_period_spin.value()} ms)")
        else:
            self.auto_detail_timer.stop()
            self.log_line("Auto node detail polling stopped")

    def request_next_node_details(self) -> None:
        sensors = self._selected_sensor_list()
        nodes = self._selected_node_list()
        if not sensors or not nodes:
            return

        if self.auto_detail_sensor not in sensors:
            self.auto_detail_sensor = sensors[0]
        if self.auto_detail_node not in nodes:
            self.auto_detail_node = nodes[0]

        self.detail_sensor_spin.setValue(self.auto_detail_sensor)
        self.detail_node_spin.setValue(self.auto_detail_node)
        self.send_diag_control()
        self._send_node_detail_request(self.auto_detail_sensor, self.auto_detail_node)

        sensor_idx = sensors.index(self.auto_detail_sensor)
        node_idx = nodes.index(self.auto_detail_node)
        node_idx += 1
        if node_idx >= len(nodes):
            node_idx = 0
            sensor_idx = (sensor_idx + 1) % len(sensors)

        self.auto_detail_sensor = sensors[sensor_idx]
        self.auto_detail_node = nodes[node_idx]

    def uid32_value(self) -> int:
        text = self.uid_edit.text().strip().lower().replace("0x", "")
        return int(text or "0", 16) & 0xFFFFFFFF

    def send_node_discover(self) -> None:
        payload = bytes([
            1,
            self.discover_delay_spin.value(),
            1 if self.discover_uid_check.isChecked() else 0,
            1 if self.discover_cfg_check.isChecked() else 0,
            0, 0, 0, 0,
        ])
        self.send_packet(Pgn.NODE_DISCOVER, payload)

    def send_node_assign(self) -> None:
        uid32 = self.uid32_value()
        payload = uid32.to_bytes(4, "little") + bytes([
            self.sensor_channel_spin.value(),
            self.node_id_spin.value(),
            1,
            1,
        ])
        self.send_packet(Pgn.NODE_ASSIGN, payload)

    def send_node_save(self) -> None:
        uid32 = self.uid32_value()
        payload = uid32.to_bytes(4, "little") + bytes([0, 0, 0, 0])
        self.send_packet(Pgn.NODE_SAVE_CFG, payload)

    def send_node_test_spin(self) -> None:
        uid32 = self.uid32_value()
        rpm_x10 = int(round(self.test_rpm_spin.value() * 10.0))
        payload = bytearray(8)
        payload[0:4] = uid32.to_bytes(4, "little")
        payload[4] = rpm_x10 & 0xFF
        payload[5] = (rpm_x10 >> 8) & 0xFF
        payload[6] = self.test_duration_spin.value()
        payload[7] = 0
        self.send_packet(Pgn.NODE_TEST_SPIN, bytes(payload))

    def send_node_diag_req(self) -> None:
        uid32 = self.uid32_value()
        payload = uid32.to_bytes(4, "little") + bytes([1, 1, 0, 0])
        self.send_packet(Pgn.NODE_DIAG_REQ, payload)

    def send_node_cfg_read(self) -> None:
        uid32 = self.uid32_value()
        payload = uid32.to_bytes(4, "little") + bytes([0, 0, 0, 0])
        self.send_packet(Pgn.NODE_CFG_READ, payload)

    def send_node_set_can_source(self) -> None:
        uid32 = self.uid32_value()
        payload = uid32.to_bytes(4, "little") + bytes([
            self.sensor_channel_spin.value(),
            self.node_id_spin.value(),
            self.can_profile_spin.value(),
            1,
        ])
        self.send_packet(Pgn.NODE_SET_CAN_SOURCE, payload)

    def send_node_identify(self) -> None:
        uid32 = self.uid32_value()
        payload = uid32.to_bytes(4, "little") + bytes([
            self.identify_mode_combo.currentIndex(),
            self.test_duration_spin.value(),
            0,
            0,
        ])
        self.send_packet(Pgn.NODE_IDENTIFY, payload)

    def send_node_reboot(self) -> None:
        uid32 = self.uid32_value()
        payload = uid32.to_bytes(4, "little") + bytes([0, 0, 0, 0])
        self.send_packet(Pgn.NODE_REBOOT, payload)

    def on_apply_target(self) -> None:
        text = self.target_ip_edit.text().strip()
        try:
            socket.inet_aton(text)
        except OSError:
            QMessageBox.warning(self, "IP Error", "Hibas IPv4 cim")
            return
        self.current_target_ip = text
        self.log_line(f"Target IP set to {text}")

    def try_bind_udp_listener(self) -> None:
        if self.rx_bound:
            self.rx_status_label.setText(f"RX: listening on *:{UDP_LISTEN_PORT}")
            return

        bind_ok = self.udp.bind(
            QHostAddress.AnyIPv4,
            UDP_LISTEN_PORT,
            QUdpSocket.ShareAddress | QUdpSocket.ReuseAddressHint,
        )

        if bind_ok:
            self.rx_bound = True
            self.rx_status_label.setText(f"RX: listening on *:{UDP_LISTEN_PORT}")
            self.log_line(f"UDP listener bound on *:{UDP_LISTEN_PORT}")
        else:
            error_text = self.udp.errorString()
            self.rx_status_label.setText(f"RX: bind failed on {UDP_LISTEN_PORT}")
            self.log_line(
                f"UDP listener bind failed on *:{UDP_LISTEN_PORT} ({error_text}). "
                "The app can still send commands, but RX packets will not appear until the port is free."
            )

    def on_udp_ready_read(self) -> None:
        while self.udp.hasPendingDatagrams():
            datagram, host, port = self.udp.readDatagram(self.udp.pendingDatagramSize())
            datagram = bytes(datagram)
            if len(datagram) != PACKET_LEN:
                self.log_line(f"RX {len(datagram)} bytes from {host.toString()}:{port} ignored")
                continue
            if packet_crc(datagram[:-1]) != datagram[-1]:
                self.log_line(f"RX bad CRC from {host.toString()}:{port}")
                continue

            pgn = datagram[0] | (datagram[1] << 8)
            payload = datagram[2:10]
            self.handle_packet(pgn, payload, host.toString(), port)

    def handle_packet(self, pgn: int, payload: bytes, host: str, port: int) -> None:
        if pgn == Pgn.ECU_CFG_STATUS:
            self.handle_cfg_status(payload)
        elif pgn == Pgn.ECU_DIAG_STATUS:
            self.handle_diag_status(payload)
        elif pgn == Pgn.ECU_DIAG_SENSOR:
            self.handle_diag_sensor(payload)
        elif pgn == Pgn.ECU_DIAG_NODE_SUMMARY:
            self.handle_diag_node_summary(payload)
        elif pgn == Pgn.ECU_DIAG_NODE_DETAIL_A:
            self.handle_diag_node_detail_a(payload)
        elif pgn == Pgn.ECU_DIAG_NODE_DETAIL_B:
            self.handle_diag_node_detail_b(payload)
        else:
            self.log_line(f"RX PGN {pgn} from {host}:{port} payload={payload.hex(' ')}")

    def handle_cfg_status(self, payload: bytes) -> None:
        block = payload[0]
        index = payload[1]
        if block == Block.MACHINE:
            self.active_sensor_spin.setValue(payload[2])
            self.row_count_spin.setValue(payload[3])
            self.detail_node_spin.setMaximum(max(1, payload[3]))
            if self.detail_node_spin.value() > payload[3]:
                self.detail_node_spin.setValue(max(1, payload[3]))
            self.holes_spin.setValue(parse_u16(payload[4], payload[5]))
            self.upm_scale_spin.setValue(parse_u16(payload[6], payload[7]) / 10.0)
            self.log_line("RX CFG_STATUS MACHINE")
        elif block == Block.DRIVE:
            self.trim_limit_spin.setValue(parse_i16(payload[2], payload[3]) / 10.0)
            self.position_kp_spin.setValue(parse_u16(payload[4], payload[5]) / 1000.0)
            self.log_line("RX CFG_STATUS DRIVE")
        elif block == Block.DIAG:
            self.diag_enable_check.setChecked(payload[2] != 0)
            self.diag_stream_check.setChecked(payload[3] != 0)
            self.diag_period_spin.setValue(max(10, payload[4] * 10))
            self.diag_detail_combo.setCurrentIndex(min(payload[5], 1))
            self.log_line("RX CFG_STATUS DIAG")
        elif block == Block.NETWORK:
            self.ip_last_octet_spin.setValue(payload[2])
            self.module_id_spin.setValue(payload[3])
            self.log_line("RX CFG_STATUS NETWORK")
        elif block == Block.CHANNEL:
            self.channel_cfg_spin.setValue(index)
            self.channel_drive_ratio_spin.setValue(parse_u16(payload[2], payload[3]) / 100.0)
            self.channel_motor_ratio_spin.setValue(parse_u16(payload[4], payload[5]) / 100.0)
            self.log_line(
                f"RX CFG_STATUS CHANNEL index={index} drive={self.channel_drive_ratio_spin.value():.2f} "
                f"motor={self.channel_motor_ratio_spin.value():.2f}"
            )
        else:
            self.log_line(f"RX CFG_STATUS block={block} payload={payload.hex(' ')}")

    def handle_diag_status(self, payload: bytes) -> None:
        self.log_line(
            f"RX ECU_DIAG_STATUS sensors={payload[0]} rows={payload[1]} eth={payload[2]} can={payload[3]} diag={payload[4]} timeout={payload[5]}"
        )

    def handle_diag_sensor(self, payload: bytes) -> None:
        sensor = payload[0]
        if sensor not in self.sensor_diags:
            return
        diag = self.sensor_diags[sensor]
        diag.mode = payload[1]
        diag.base_rpm = parse_u16(payload[2], payload[3])
        diag.target_upm = parse_u32(payload[4:8]) / 1000.0
        self.refresh_sensor_row(sensor)
        self.log_line(f"RX ECU_DIAG_SENSOR s{sensor} base={diag.base_rpm:.1f} target={diag.target_upm:.3f}")

    def handle_diag_node_summary(self, payload: bytes) -> None:
        sensor = payload[0]
        if sensor not in self.sensor_diags:
            return
        diag = self.sensor_diags[sensor]
        diag.online_nodes = payload[1]
        diag.avg_rpm = parse_u16(payload[2], payload[3])
        diag.total_rpm = parse_u16(payload[4], payload[5])
        diag.avg_pos = parse_u16(payload[6], payload[7])
        self.refresh_sensor_row(sensor)
        self.log_line(f"RX ECU_DIAG_NODE_SUMMARY s{sensor} online={diag.online_nodes} total={diag.total_rpm:.1f}")

    def handle_diag_node_detail_a(self, payload: bytes) -> None:
        sensor = payload[0]
        node_id = payload[1]
        key = (sensor, node_id)
        diag = self.node_diags.get(key, NodeDiag(sensor=sensor, node_id=node_id))
        diag.status_flags = payload[2]
        diag.error_code = payload[3]
        diag.actual_rpm = parse_u16(payload[4], payload[5])
        diag.actual_pos = parse_u16(payload[6], payload[7])
        self.node_diags[key] = diag
        self.refresh_node_table()
        self.log_line(
            f"RX ECU_DIAG_NODE_DETAIL_A s{sensor} n{node_id} rpm={diag.actual_rpm:.1f} pos={diag.actual_pos}"
        )

    def handle_diag_node_detail_b(self, payload: bytes) -> None:
        sensor = payload[0]
        node_id = payload[1]
        key = (sensor, node_id)
        diag = self.node_diags.get(key, NodeDiag(sensor=sensor, node_id=node_id))
        diag.bus_voltage = payload[2] / 10.0
        diag.motor_current = payload[3] / 10.0
        diag.controller_temp = payload[4]
        diag.motor_temp = payload[5]
        diag.warning_flags = payload[6]
        diag.fault_flags = payload[7]
        self.node_diags[key] = diag
        self.refresh_node_table()
        self.log_line(
            f"RX ECU_DIAG_NODE_DETAIL_B s{sensor} n{node_id} bus={diag.bus_voltage:.1f} current={diag.motor_current:.1f}"
        )

    def refresh_sensor_row(self, sensor: int) -> None:
        diag = self.sensor_diags[sensor]
        values = [
            f"S{sensor}",
            str(diag.mode),
            f"{diag.base_rpm:.1f}",
            f"{diag.target_upm:.3f}",
            str(diag.online_nodes),
            f"{diag.avg_rpm:.1f}",
            f"{diag.total_rpm:.1f}",
        ]
        for col, value in enumerate(values):
            item = self.sensor_table.item(sensor, col)
            if item is None:
                item = QTableWidgetItem()
                self.sensor_table.setItem(sensor, col, item)
            item.setText(value)
            if col == 0:
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)

    def refresh_node_table(self) -> None:
        rows = sorted(self.node_diags.values(), key=lambda item: (item.sensor, item.node_id))
        self.node_table.setRowCount(len(rows))

        for row, diag in enumerate(rows):
            values = [
                f"S{diag.sensor}",
                str(diag.node_id),
                f"0x{diag.status_flags:02X}",
                str(diag.error_code),
                f"{diag.actual_rpm:.1f}",
                str(diag.actual_pos),
                f"{diag.bus_voltage:.1f}",
                f"{diag.motor_current:.1f}",
                str(diag.controller_temp),
                str(diag.motor_temp),
                f"0x{diag.warning_flags:02X}",
                f"0x{diag.fault_flags:02X}",
            ]

            for col, value in enumerate(values):
                item = self.node_table.item(row, col)
                if item is None:
                    item = QTableWidgetItem()
                    self.node_table.setItem(row, col, item)
                item.setText(value)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)

    def log_line(self, text: str) -> None:
        timestamp = QDateTime.currentDateTime().toString("HH:mm:ss")
        self.log.appendPlainText(f"[{timestamp}] {text}")


def main() -> int:
    app = QApplication(sys.argv)
    window = EcuServiceTool()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
