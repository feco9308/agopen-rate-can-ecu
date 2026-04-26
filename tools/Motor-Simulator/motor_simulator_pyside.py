import threading
import time
import uuid
from dataclasses import dataclass
from typing import Dict, List, Optional

import can
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QStatusBar,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


CHANNEL = "PCAN_USBBUS1"
BITRATE = 250000
NODES_PER_SENSOR = 6
SENSOR_TAB_COUNT = 4
CONTROL_TIMEOUT_S = 0.15
STATUS_PERIOD_S = 0.02
DIAG_PERIOD_S = 0.10
PRESENCE_PERIOD_S = 1.0

SERVICE_ID_DISCOVER = 0x500
SERVICE_ID_UID_A = 0x501
SERVICE_ID_UID_B = 0x502
SERVICE_ID_ASSIGN = 0x503
SERVICE_ID_SAVE_CFG = 0x504
SERVICE_ID_ACK = 0x505
SERVICE_ID_TEST_SPIN = 0x506
SERVICE_ID_DIAG_REQ = 0x507
SERVICE_ID_DIAG_RESP_A = 0x508
SERVICE_ID_DIAG_RESP_B = 0x509
SERVICE_ID_REBOOT = 0x50A
SERVICE_ID_IDENTIFY = 0x50B
SERVICE_ID_CFG_READ = 0x50C
SERVICE_ID_CFG_RESP = 0x50D
SERVICE_ID_SET_CAN_SOURCE = 0x50E

MODE_OFF = 0
MODE_MANUAL = 1
MODE_AUTO = 2
MODE_CALIBRATION = 3
MODE_FAULT_HOLD = 4
MODE_NAMES = {
    MODE_OFF: "OFF",
    MODE_MANUAL: "MANUAL",
    MODE_AUTO: "AUTO",
    MODE_CALIBRATION: "CAL",
    MODE_FAULT_HOLD: "FAULT",
}

CMD_ENABLE = 1
CMD_DISABLE = 2
CMD_ZERO_POS = 3
CMD_CLEAR_FAULT = 4

CTRL_DRIVE_ENABLE = 1 << 0
CTRL_SYNC_ENABLE = 1 << 1
CTRL_ESTOP = 1 << 2
CTRL_DIAG_ENABLE = 1 << 3

NODE_FLAG_ALLOW_RUN = 1 << 0
NODE_FLAG_INVERT_DIR = 1 << 1
NODE_FLAG_USE_LOCAL_SENSOR = 1 << 2

ST_READY = 1 << 0
ST_ENABLED = 1 << 1
ST_RUNNING = 1 << 2
ST_SYNC_LOCKED = 1 << 3
ST_WARNING_ACTIVE = 1 << 4
ST_FAULT_ACTIVE = 1 << 5
ST_POSITION_VALID = 1 << 6
ST_CAN_TIMEOUT = 1 << 7

WRN_RPM_DEV = 1 << 0
WRN_SYNC_DEV = 1 << 1
WRN_COMM_LATE = 1 << 4

FLT_DRIVER = 1 << 7

SEED_VALID = 1 << 0
SEED_BLOCKED = 1 << 1
SEED_SLOWED = 1 << 2
SEED_SENSOR_FAULT = 1 << 3


@dataclass(frozen=True)
class CanProfile:
    global_control: int
    global_timebase: int
    global_estop: int
    node_cmd_base: int
    node_presence_base: int
    node_status_base: int
    node_diag_base: int


PROFILES: List[CanProfile] = [
    CanProfile(0x080, 0x081, 0x082, 0x100, 0x140, 0x180, 0x1C0),
    CanProfile(0x200, 0x201, 0x202, 0x210, 0x240, 0x280, 0x2C0),
    CanProfile(0x300, 0x301, 0x302, 0x310, 0x340, 0x380, 0x3C0),
    CanProfile(0x400, 0x401, 0x402, 0x410, 0x440, 0x480, 0x4C0),
]


def le_u16(value: int) -> List[int]:
    value &= 0xFFFF
    return [value & 0xFF, (value >> 8) & 0xFF]


def le_i16(value: int) -> List[int]:
    value &= 0xFFFF
    return [value & 0xFF, (value >> 8) & 0xFF]


def u16_from_le(data: List[int], idx: int) -> int:
    return data[idx] | (data[idx + 1] << 8)


def i16_from_le(data: List[int], idx: int) -> int:
    value = data[idx] | (data[idx + 1] << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def u32_from_le(data: List[int], idx: int = 0) -> int:
    return data[idx] | (data[idx + 1] << 8) | (data[idx + 2] << 16) | (data[idx + 3] << 24)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap_rev(value: float) -> float:
    while value >= 1.0:
        value -= 1.0
    while value < 0.0:
        value += 1.0
    return value


def wrap_pos_error(ref_u16: int, act_u16: int) -> int:
    err = int(ref_u16) - int(act_u16)
    if err > 32767:
        err -= 65536
    if err < -32768:
        err += 65536
    return err


def set_or_clear(flags: int, bit: int, enabled: bool) -> int:
    return (flags | bit) if enabled else (flags & ~bit)


@dataclass
class GlobalControlState:
    system_mode: int = MODE_OFF
    control_flags: int = 0
    base_rpm_u16: int = 0
    sync_pos_u16: int = 0
    sequence: int = 0
    last_rx: float = 0.0
    valid: bool = False
    estop: bool = False
    estop_reason: int = 0


@dataclass
class SimNode:
    uid32: int
    uid32_hi: int
    hw_rev: int
    fw_major: int
    fw_minor: int
    sensor_channel: int
    node_id: int
    can_profile_index: int
    config_saved: bool = False
    identify_until: float = 0.0
    test_spin_until: float = 0.0
    test_spin_rpm: float = 0.0
    enabled: bool = False
    allow_run: bool = True
    invert_dir: bool = False
    use_local_sensor: bool = False
    trim_rpm_x10: int = 0
    section_mask: int = 0
    section_active: bool = False
    should_run: bool = False
    actual_rpm: float = 0.0
    target_rpm: float = 0.0
    pos_rev: float = 0.0
    alive_counter: int = 0
    warning_flags: int = 0
    fault_flags: int = 0
    error_code: int = 0
    bus_voltage: float = 12.4
    motor_current: float = 0.0
    controller_temp: float = 30.0
    motor_temp: float = 30.0
    last_cmd_seq: int = 0
    last_cmd_rx: float = 0.0
    slowdown_pct: int = 0
    blockage_pct: int = 0

    def profile(self) -> CanProfile:
        return PROFILES[self.can_profile_index]

    def status_id(self) -> int:
        return self.profile().node_status_base + self.node_id

    def diag_id(self) -> int:
        return self.profile().node_diag_base + self.node_id

    def presence_id(self) -> int:
        return self.profile().node_presence_base + self.node_id


class MotorSimulatorEngine:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.bus: Optional[can.Bus] = None
        self.running = False
        self.rx_thread: Optional[threading.Thread] = None
        self.loop_thread: Optional[threading.Thread] = None
        self.channel = CHANNEL
        self.bitrate = BITRATE
        self.log_lines: List[str] = []
        self.last_error = ""
        self.last_rx = "-"
        self.last_tx = "-"
        self.tx_blocked_until = 0.0
        self.publish_status = True
        self.publish_diag = True
        self.publish_presence = False
        self.allow_diag_tx = True
        self.sensor_enabled: Dict[int, bool] = {i: True for i in range(SENSOR_TAB_COUNT)}
        self.globals: Dict[int, GlobalControlState] = {i: GlobalControlState() for i in range(SENSOR_TAB_COUNT)}
        self.nodes: List[SimNode] = []
        self._create_default_nodes()

    def set_sensor_enabled(self, sensor: int, enabled: bool) -> None:
        with self.lock:
            self.sensor_enabled[sensor] = enabled
            if not enabled:
                self.globals[sensor] = GlobalControlState()
                for node in self.nodes:
                    if node.sensor_channel != sensor:
                        continue
                    node.enabled = False
                    node.section_active = False
                    node.should_run = False
                    node.target_rpm = 0.0
                    node.actual_rpm = 0.0
                    node.warning_flags = 0
                    node.fault_flags = 0
                    node.error_code = 0
        self.log(f"Sensor {sensor} {'enabled' if enabled else 'disabled'}")

    def set_node_impairment(self, sensor: int, node_id: int, slowdown_pct: int, blockage_pct: int) -> None:
        with self.lock:
            for node in self.nodes:
                if node.sensor_channel == sensor and node.node_id == node_id:
                    node.slowdown_pct = max(0, min(100, slowdown_pct))
                    node.blockage_pct = max(0, min(100, blockage_pct))
                    break
            else:
                return
        self.log(f"Impairment S{sensor} N{node_id}: slowdown={slowdown_pct}% blockage={blockage_pct}%")

    def clear_node_impairment(self, sensor: int, node_id: int) -> None:
        self.set_node_impairment(sensor, node_id, 0, 0)

    def _node_should_publish(self, node: SimNode, now: float) -> bool:
        if not self.sensor_enabled.get(node.sensor_channel, True):
            return False
        cmd_fresh = (now - node.last_cmd_rx) <= (CONTROL_TIMEOUT_S * 2.0)
        spinning = abs(node.actual_rpm) > 0.2 or now < node.test_spin_until
        active_flags = node.last_cmd_rx > 0.0 and (node.enabled or node.should_run or node.warning_flags != 0 or node.fault_flags != 0)
        identifying = now < node.identify_until
        return cmd_fresh or spinning or active_flags or identifying

    def _node_should_publish_diag(self, node: SimNode, now: float) -> bool:
        if not self.allow_diag_tx:
            return False
        if not self.sensor_enabled.get(node.sensor_channel, True):
            return False
        g = self.globals[node.can_profile_index]
        diag_enable = bool(g.control_flags & CTRL_DIAG_ENABLE)
        return diag_enable and self._node_should_publish(node, now)

    def _create_default_nodes(self) -> None:
        self.nodes.clear()
        for sensor in range(SENSOR_TAB_COUNT):
            for node_id in range(1, NODES_PER_SENSOR + 1):
                raw = uuid.uuid4().int
                self.nodes.append(
                    SimNode(
                        uid32=raw & 0xFFFFFFFF,
                        uid32_hi=(raw >> 32) & 0xFFFFFFFF,
                        hw_rev=1,
                        fw_major=1,
                        fw_minor=0,
                        sensor_channel=sensor,
                        node_id=node_id,
                        can_profile_index=sensor,
                    )
                )

    def log(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        with self.lock:
            self.log_lines.append(f"[{stamp}] {text}")
            self.log_lines = self.log_lines[-800:]

    def connect(self, channel: str, bitrate: int) -> None:
        if self.bus is not None:
            self.disconnect()
        self.channel = channel
        self.bitrate = bitrate
        self.bus = can.Bus(interface="pcan", channel=channel, bitrate=bitrate)
        self.log(f"Connected to {channel} @ {bitrate}")

    def disconnect(self) -> None:
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
            self.log("CAN disconnected")

    def start(self) -> None:
        if self.bus is None:
            raise RuntimeError("No CAN bus connected")
        if self.running:
            return
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.loop_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.rx_thread.start()
        self.loop_thread.start()
        self.log("Simulation started")

    def stop(self) -> None:
        self.running = False
        self.log("Simulation stopped")

    def shutdown(self) -> None:
        self.stop()
        time.sleep(0.05)
        self.disconnect()

    def send_msg(self, arbitration_id: int, data: List[int]) -> None:
        if self.bus is None:
            return
        now = time.time()
        if now < self.tx_blocked_until:
            return
        msg = can.Message(arbitration_id=arbitration_id, is_extended_id=False, data=data)
        try:
            self.bus.send(msg)
            with self.lock:
                self.last_tx = f"0x{arbitration_id:03X} {' '.join(f'{b:02X}' for b in data)}"
                self.last_error = ""
        except Exception as exc:
            err = str(exc)
            with self.lock:
                self.last_error = err
                self.tx_blocked_until = time.time() + 1.0
            self.log(f"TX paused: {err}")

    def _find_node_by_uid(self, uid32: int) -> Optional[SimNode]:
        with self.lock:
            for node in self.nodes:
                if node.uid32 == uid32:
                    return node
        return None

    def _find_assigned_node(self, sensor_channel: int, node_id: int) -> Optional[SimNode]:
        with self.lock:
            for node in self.nodes:
                if node.sensor_channel == sensor_channel and node.node_id == node_id:
                    return node
        return None

    def _rx_loop(self) -> None:
        while self.running:
            if self.bus is None:
                time.sleep(0.05)
                continue
            try:
                msg = self.bus.recv(timeout=0.05)
                if msg is None or msg.is_extended_id:
                    continue
                self._handle_rx(msg)
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
                self.log(f"RX error: {exc}")
                time.sleep(0.1)

    def _handle_rx(self, msg: can.Message) -> None:
        data = list(msg.data)
        with self.lock:
            self.last_rx = f"0x{msg.arbitration_id:03X} {' '.join(f'{b:02X}' for b in data)}"

        if self._handle_service(msg.arbitration_id, data):
            return

        for sensor_index, profile in enumerate(PROFILES):
            if not self.sensor_enabled.get(sensor_index, True):
                continue
            if msg.arbitration_id == profile.global_control and len(data) >= 8:
                with self.lock:
                    g = self.globals[sensor_index]
                    g.system_mode = data[0]
                    g.control_flags = data[1]
                    g.base_rpm_u16 = u16_from_le(data, 2)
                    g.sync_pos_u16 = u16_from_le(data, 4)
                    g.sequence = data[6]
                    g.last_rx = time.time()
                    g.valid = True
                self.log(
                    f"RX GLOBAL_CONTROL s{sensor_index}: mode={data[0]} flags=0x{data[1]:02X} rpm={u16_from_le(data, 2):.1f}"
                )
                return

            if msg.arbitration_id == profile.global_estop and len(data) >= 2:
                with self.lock:
                    g = self.globals[sensor_index]
                    if data[0] == 0xA5:
                        g.estop = True
                        g.estop_reason = data[1]
                self.log(f"RX GLOBAL_ESTOP s{sensor_index} reason={data[1]}")
                return

            if profile.node_cmd_base < msg.arbitration_id <= profile.node_cmd_base + NODES_PER_SENSOR and len(data) >= 8:
                node_id = msg.arbitration_id - profile.node_cmd_base
                node = self._find_assigned_node(sensor_index, node_id)
                if node is not None:
                    self._apply_node_cmd(node, data)
                return

    def _handle_service(self, arbitration_id: int, data: List[int]) -> bool:
        if arbitration_id == SERVICE_ID_DISCOVER and len(data) >= 4:
            with self.lock:
                nodes = [node for node in self.nodes if self.sensor_enabled.get(node.sensor_channel, True)]
            for node in nodes:
                self._send_uid(node)
            self.log("RX SRV_DISCOVER -> UID responses sent")
            return True

        if arbitration_id == SERVICE_ID_ASSIGN and len(data) >= 8:
            uid32 = u32_from_le(data)
            node = self._find_node_by_uid(uid32)
            if node is not None:
                with self.lock:
                    node.sensor_channel = int(data[4]) % SENSOR_TAB_COUNT
                    node.node_id = max(1, min(NODES_PER_SENSOR, int(data[5])))
                    node.can_profile_index = node.sensor_channel
                    node.config_saved = bool(data[7])
                self._send_ack(node, SERVICE_ID_ASSIGN, 0)
                self.log(f"RX SRV_ASSIGN uid=0x{uid32:08X} -> s{data[4]} node{data[5]}")
            return True

        if arbitration_id == SERVICE_ID_SAVE_CFG and len(data) >= 6:
            uid32 = u32_from_le(data)
            node = self._find_node_by_uid(uid32)
            if node is not None:
                with self.lock:
                    node.config_saved = True
                self._send_ack(node, SERVICE_ID_SAVE_CFG, 0)
                self.log(f"RX SRV_SAVE_CFG uid=0x{uid32:08X}")
            return True

        if arbitration_id == SERVICE_ID_TEST_SPIN and len(data) >= 8:
            uid32 = u32_from_le(data)
            node = self._find_node_by_uid(uid32)
            if node is not None:
                rpm = i16_from_le(data, 4) / 10.0
                duration = max(1, data[6])
                with self.lock:
                    node.test_spin_rpm = rpm
                    node.test_spin_until = time.time() + duration
                self._send_ack(node, SERVICE_ID_TEST_SPIN, 0)
                self.log(f"RX SRV_TEST_SPIN uid=0x{uid32:08X} rpm={rpm:.1f} duration={duration}s")
            return True

        if arbitration_id == SERVICE_ID_DIAG_REQ and len(data) >= 7:
            uid32 = u32_from_le(data)
            node = self._find_node_by_uid(uid32)
            if node is not None:
                self._send_diag_response(node)
                self.log(f"RX SRV_DIAG_REQ uid=0x{uid32:08X}")
            return True

        if arbitration_id == SERVICE_ID_REBOOT and len(data) >= 5:
            uid32 = u32_from_le(data)
            node = self._find_node_by_uid(uid32)
            if node is not None:
                with self.lock:
                    node.enabled = False
                    node.actual_rpm = 0.0
                    node.target_rpm = 0.0
                self._send_ack(node, SERVICE_ID_REBOOT, 0)
                self.log(f"RX SRV_REBOOT uid=0x{uid32:08X}")
            return True

        if arbitration_id == SERVICE_ID_IDENTIFY and len(data) >= 6:
            uid32 = u32_from_le(data)
            node = self._find_node_by_uid(uid32)
            if node is not None:
                duration = max(1, data[5])
                with self.lock:
                    node.identify_until = time.time() + duration
                self._send_ack(node, SERVICE_ID_IDENTIFY, 0)
                self.log(f"RX SRV_IDENTIFY uid=0x{uid32:08X} duration={duration}s")
            return True

        if arbitration_id == SERVICE_ID_CFG_READ and len(data) >= 5:
            uid32 = u32_from_le(data)
            node = self._find_node_by_uid(uid32)
            if node is not None:
                self._send_cfg_response(node, data[4])
                self.log(f"RX SRV_CFG_READ uid=0x{uid32:08X} block={data[4]}")
            return True

        if arbitration_id == SERVICE_ID_SET_CAN_SOURCE and len(data) >= 8:
            uid32 = u32_from_le(data)
            node = self._find_node_by_uid(uid32)
            if node is not None:
                with self.lock:
                    node.sensor_channel = int(data[4]) % SENSOR_TAB_COUNT
                    node.node_id = max(1, min(NODES_PER_SENSOR, int(data[5])))
                    node.can_profile_index = int(data[6]) % SENSOR_TAB_COUNT
                    node.config_saved = bool(data[7])
                self._send_ack(node, SERVICE_ID_SET_CAN_SOURCE, 0)
                self.log(
                    f"RX SRV_SET_CAN_SOURCE uid=0x{uid32:08X} sensor={data[4]} node={data[5]} profile={data[6]}"
                )
            return True

        return False

    def _apply_node_cmd(self, node: SimNode, data: List[int]) -> None:
        with self.lock:
            node_cmd = data[0]
            node_flags = data[1]
            node.allow_run = bool(node_flags & NODE_FLAG_ALLOW_RUN)
            node.invert_dir = bool(node_flags & NODE_FLAG_INVERT_DIR)
            node.use_local_sensor = bool(node_flags & NODE_FLAG_USE_LOCAL_SENSOR)
            node.trim_rpm_x10 = i16_from_le(data, 2)
            node.section_mask = u16_from_le(data, 4)
            node.last_cmd_seq = data[6]
            node.last_cmd_rx = time.time()
            node.section_active = bool(node.section_mask & (1 << (node.node_id - 1)))

            if node_cmd == CMD_ENABLE:
                node.enabled = True
            elif node_cmd == CMD_DISABLE:
                node.enabled = False
                node.target_rpm = 0.0
            elif node_cmd == CMD_ZERO_POS:
                node.pos_rev = 0.0
            elif node_cmd == CMD_CLEAR_FAULT:
                node.fault_flags = 0
                node.warning_flags = 0
                node.error_code = 0

        self.log(
            f"RX NODE_CMD s{node.sensor_channel} node{node.node_id}: cmd={data[0]} flags=0x{data[1]:02X} trim={i16_from_le(data, 2)/10.0:.1f} mask=0x{u16_from_le(data, 4):04X}"
        )

    def _main_loop(self) -> None:
        last = time.time()
        last_status = 0.0
        last_diag = 0.0
        last_presence = 0.0

        while self.running:
            now = time.time()
            dt = min(now - last, 0.1)
            last = now

            try:
                with self.lock:
                    for node in self.nodes:
                        self._update_node_locked(node, dt, now)

                if self.publish_status and now - last_status >= STATUS_PERIOD_S:
                    last_status = now
                    with self.lock:
                        nodes = list(self.nodes)
                    for node in nodes:
                        if self._node_should_publish(node, now):
                            self._send_status(node)

                if self.publish_diag and now - last_diag >= DIAG_PERIOD_S:
                    last_diag = now
                    with self.lock:
                        nodes = list(self.nodes)
                    for node in nodes:
                        if self._node_should_publish_diag(node, now):
                            self._send_diag(node)

                if self.publish_presence and now - last_presence >= PRESENCE_PERIOD_S:
                    last_presence = now
                    with self.lock:
                        nodes = list(self.nodes)
                    for node in nodes:
                        if self._node_should_publish(node, now):
                            self._send_presence(node)
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
                self.log(f"Loop error: {exc}")

            time.sleep(0.005)

    def _update_node_locked(self, node: SimNode, dt: float, now: float) -> None:
        if not self.sensor_enabled.get(node.sensor_channel, True):
            node.enabled = False
            node.section_active = False
            node.should_run = False
            node.target_rpm = 0.0
            node.actual_rpm = 0.0
            node.warning_flags = 0
            node.fault_flags = 0
            node.error_code = 0
            return

        g = self.globals[node.can_profile_index]
        has_seen_cmd = node.last_cmd_rx > 0.0
        can_timeout = (not g.valid) or ((now - g.last_rx) > CONTROL_TIMEOUT_S)
        drive_enable = bool(g.control_flags & CTRL_DRIVE_ENABLE)
        sync_enable = bool(g.control_flags & CTRL_SYNC_ENABLE)
        estop_flag = bool(g.control_flags & CTRL_ESTOP) or g.estop
        diag_enable = bool(g.control_flags & CTRL_DIAG_ENABLE)
        mode_allows_run = g.system_mode in (MODE_MANUAL, MODE_AUTO, MODE_CALIBRATION)

        actual_pos_u16 = int(wrap_rev(node.pos_rev) * 65535.0) & 0xFFFF
        pos_err = wrap_pos_error(g.sync_pos_u16, actual_pos_u16)
        pos_err_rev = pos_err / 65536.0

        sync_corr = 60.0 * pos_err_rev if sync_enable else 0.0
        cmd_rpm = float(g.base_rpm_u16) + (node.trim_rpm_x10 / 10.0) + sync_corr
        if node.invert_dir:
            cmd_rpm = -cmd_rpm
        cmd_rpm = clamp(cmd_rpm, -10000.0, 10000.0)

        slowdown_scale = (100.0 - float(node.slowdown_pct)) / 100.0
        blockage_scale = (100.0 - float(node.blockage_pct)) / 100.0
        cmd_rpm *= slowdown_scale * blockage_scale

        should_run = (
            has_seen_cmd
            and node.enabled
            and node.allow_run
            and node.section_active
            and drive_enable
            and mode_allows_run
            and not can_timeout
            and not estop_flag
            and node.fault_flags == 0
        )
        node.should_run = should_run

        if now < node.test_spin_until:
            cmd_rpm = node.test_spin_rpm
            should_run = True

        node.target_rpm = cmd_rpm if should_run else 0.0

        error = node.target_rpm - node.actual_rpm
        response = clamp(error * 2.0, -4000.0, 4000.0)
        node.actual_rpm += response * dt
        if not should_run and now >= node.test_spin_until:
            node.actual_rpm *= 0.90
            if abs(node.actual_rpm) < 0.05:
                node.actual_rpm = 0.0
        node.actual_rpm = clamp(node.actual_rpm, -10000.0, 10000.0)
        node.pos_rev = wrap_rev(node.pos_rev + (node.actual_rpm / 60.0) * dt)

        node.warning_flags = 0
        node.error_code = 0
        if has_seen_cmd or now < node.test_spin_until:
            if abs(node.target_rpm - node.actual_rpm) > 5.0:
                node.warning_flags |= WRN_RPM_DEV
            if abs(pos_err_rev) > 0.03:
                node.warning_flags |= WRN_SYNC_DEV
            if can_timeout:
                node.warning_flags |= WRN_COMM_LATE
            if node.slowdown_pct > 0 or node.blockage_pct > 0:
                node.warning_flags |= WRN_RPM_DEV
            if estop_flag:
                node.fault_flags = FLT_DRIVER
                node.error_code = 2
            else:
                node.fault_flags = 0
        else:
            node.fault_flags = 0

        node.motor_current = clamp(0.3 + abs(node.actual_rpm) * 0.03, 0.0, 20.0)
        node.bus_voltage = clamp(12.4 - node.motor_current * 0.08, 7.0, 16.0)
        node.controller_temp = clamp(30.0 + abs(node.actual_rpm) * 0.05, 20.0, 90.0)
        node.motor_temp = clamp(30.0 + abs(node.actual_rpm) * 0.04, 20.0, 95.0)

    def _send_presence(self, node: SimNode) -> None:
        seed_flags = 0
        if node.section_active or node.enabled or node.should_run:
            seed_flags |= SEED_VALID
        if node.blockage_pct >= 80:
            seed_flags |= SEED_BLOCKED
        if node.slowdown_pct > 0:
            seed_flags |= SEED_SLOWED

        skip_pct = int(clamp(float(node.blockage_pct), 0.0, 100.0))
        double_pct = 0
        singulation_pct = int(clamp(100.0 - skip_pct - double_pct, 0.0, 100.0))
        population_x1k = 0
        data = [
            seed_flags & 0xFF,
            node.blockage_pct & 0xFF,
            node.slowdown_pct & 0xFF,
            skip_pct & 0xFF,
            double_pct & 0xFF,
            singulation_pct & 0xFF,
            *le_u16(population_x1k),
        ]
        self.send_msg(node.presence_id(), data)

    def _send_status(self, node: SimNode) -> None:
        g = self.globals[node.can_profile_index]
        can_timeout = (not g.valid) or ((time.time() - g.last_rx) > CONTROL_TIMEOUT_S)
        status_flags = ST_READY | ST_POSITION_VALID
        status_flags = set_or_clear(status_flags, ST_ENABLED, node.enabled)
        status_flags = set_or_clear(status_flags, ST_RUNNING, abs(node.actual_rpm) > 1.0)
        status_flags = set_or_clear(status_flags, ST_SYNC_LOCKED, not bool(node.warning_flags & WRN_SYNC_DEV))
        status_flags = set_or_clear(status_flags, ST_WARNING_ACTIVE, node.warning_flags != 0)
        status_flags = set_or_clear(status_flags, ST_FAULT_ACTIVE, node.fault_flags != 0)
        status_flags = set_or_clear(status_flags, ST_CAN_TIMEOUT, can_timeout)

        rpm_u16 = int(round(clamp(node.actual_rpm, 0.0, 65535.0)))
        pos_u16 = int(wrap_rev(node.pos_rev) * 65535.0) & 0xFFFF
        pos_err = wrap_pos_error(g.sync_pos_u16, pos_u16)
        sync_err = int(clamp(pos_err / 256.0, -128.0, 127.0)) & 0xFF
        data = [
            status_flags & 0xFF,
            node.error_code & 0xFF,
            *le_u16(rpm_u16),
            *le_u16(pos_u16),
            node.alive_counter & 0xFF,
            sync_err,
        ]
        node.alive_counter = (node.alive_counter + 1) & 0xFF
        self.send_msg(node.status_id(), data)

    def _send_diag(self, node: SimNode) -> None:
        bus_x10 = int(round(node.bus_voltage * 10.0))
        current_x10 = int(round(node.motor_current * 10.0))
        data = [
            *le_u16(bus_x10),
            *le_i16(current_x10),
            int(round(node.controller_temp)) & 0xFF,
            int(round(node.motor_temp)) & 0xFF,
            node.fault_flags & 0xFF,
            node.warning_flags & 0xFF,
        ]
        self.send_msg(node.diag_id(), data)

    def _send_uid(self, node: SimNode) -> None:
        self.send_msg(
            SERVICE_ID_UID_A,
            [
                node.uid32 & 0xFF,
                (node.uid32 >> 8) & 0xFF,
                (node.uid32 >> 16) & 0xFF,
                (node.uid32 >> 24) & 0xFF,
                node.sensor_channel & 0xFF,
                node.node_id & 0xFF,
                node.fw_major & 0xFF,
                node.fw_minor & 0xFF,
            ],
        )
        self.send_msg(
            SERVICE_ID_UID_B,
            [
                node.uid32_hi & 0xFF,
                (node.uid32_hi >> 8) & 0xFF,
                (node.uid32_hi >> 16) & 0xFF,
                (node.uid32_hi >> 24) & 0xFF,
                node.hw_rev & 0xFF,
                0x1F,
                0x01 if node.config_saved else 0x00,
                0,
            ],
        )

    def _send_ack(self, node: SimNode, ack_cmd: int, result: int) -> None:
        self.send_msg(
            SERVICE_ID_ACK,
            [
                node.uid32 & 0xFF,
                (node.uid32 >> 8) & 0xFF,
                (node.uid32 >> 16) & 0xFF,
                (node.uid32 >> 24) & 0xFF,
                ack_cmd & 0xFF,
                result & 0xFF,
                node.sensor_channel & 0xFF,
                node.node_id & 0xFF,
            ],
        )

    def _send_diag_response(self, node: SimNode) -> None:
        actual_pos_u16 = int(wrap_rev(node.pos_rev) * 65535.0) & 0xFFFF
        self.send_msg(
            SERVICE_ID_DIAG_RESP_A,
            [
                node.sensor_channel & 0xFF,
                node.node_id & 0xFF,
                *le_i16(int(round(node.actual_rpm * 10.0))),
                *le_u16(actual_pos_u16),
                node.error_code & 0xFF,
                (0x01 if node.enabled else 0x00) | (0x02 if node.should_run else 0x00),
            ],
        )
        self.send_msg(
            SERVICE_ID_DIAG_RESP_B,
            [
                node.uid32 & 0xFF,
                (node.uid32 >> 8) & 0xFF,
                (node.uid32 >> 16) & 0xFF,
                (node.uid32 >> 24) & 0xFF,
                0,
                node.alive_counter & 0xFF,
                int(round(node.bus_voltage)) & 0xFF,
                0,
            ],
        )

    def _send_cfg_response(self, node: SimNode, block_id: int) -> None:
        if block_id == 0:
            payload = [block_id, node.node_id, node.sensor_channel, node.node_id, node.fw_major, node.fw_minor, node.hw_rev, 1 if node.config_saved else 0]
        elif block_id == 1:
            payload = [block_id, node.node_id, node.sensor_channel, node.node_id, node.can_profile_index, 1, 0, 0]
        else:
            payload = [block_id, node.node_id, 0, 0, 0, 0, 0, 0]
        self.send_msg(SERVICE_ID_CFG_RESP, payload)

    def snapshot(self) -> Dict:
        with self.lock:
            return {
                "sensor_enabled": dict(self.sensor_enabled),
                "globals": {k: GlobalControlState(**v.__dict__) for k, v in self.globals.items()},
                "nodes": [SimNode(**node.__dict__) for node in self.nodes],
                "running": self.running,
                "channel": self.channel,
                "bitrate": self.bitrate,
                "last_error": self.last_error,
                "last_rx": self.last_rx,
                "last_tx": self.last_tx,
                "log_lines": list(self.log_lines[-200:]),
            }


class MotorSimulatorWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Motor Node Simulator - PySide")
        self.resize(1600, 980)
        self.engine = MotorSimulatorEngine()
        self.sensor_tables: Dict[int, QTableWidget] = {}
        self.summary_labels: Dict[int, Dict[str, QLabel]] = {}
        self.sensor_enable_checks: Dict[int, QCheckBox] = {}

        central = QWidget()
        root = QVBoxLayout(central)
        root.addLayout(self._build_top_bar())
        root.addLayout(self._build_service_bar())

        self.tabs = QTabWidget()
        for sensor in range(SENSOR_TAB_COUNT):
            self.tabs.addTab(self._build_sensor_tab(sensor), f"Sensor {sensor}")
        root.addWidget(self.tabs, 1)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1500)
        root.addWidget(self.log_view, 1)

        self.setCentralWidget(central)
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready")

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_ui)
        self.refresh_timer.start(100)

    def _build_top_bar(self) -> QHBoxLayout:
        layout = QHBoxLayout()
        self.channel_edit = QLineEdit(CHANNEL)
        self.bitrate_edit = QLineEdit(str(BITRATE))
        btn_connect = QPushButton("Connect")
        btn_connect.clicked.connect(self.on_connect)
        btn_start = QPushButton("Start")
        btn_start.clicked.connect(self.on_start)
        btn_stop = QPushButton("Stop")
        btn_stop.clicked.connect(self.on_stop)
        self.status_check = QCheckBox("Status TX")
        self.status_check.setChecked(True)
        self.status_check.toggled.connect(lambda v: setattr(self.engine, "publish_status", v))
        self.diag_check = QCheckBox("Diag TX")
        self.diag_check.setChecked(True)
        self.diag_check.toggled.connect(self.on_diag_toggle)
        self.presence_check = QCheckBox("Presence TX")
        self.presence_check.setChecked(False)
        self.presence_check.toggled.connect(lambda v: setattr(self.engine, "publish_presence", v))
        layout.addWidget(QLabel("Channel"))
        layout.addWidget(self.channel_edit)
        layout.addWidget(QLabel("Bitrate"))
        layout.addWidget(self.bitrate_edit)
        layout.addWidget(btn_connect)
        layout.addWidget(btn_start)
        layout.addWidget(btn_stop)
        layout.addWidget(self.status_check)
        layout.addWidget(self.diag_check)
        layout.addWidget(self.presence_check)
        self.engine.publish_diag = self.diag_check.isChecked()
        for sensor in range(SENSOR_TAB_COUNT):
            check = QCheckBox(f"S{sensor}")
            check.setChecked(True)
            check.toggled.connect(lambda enabled, s=sensor: self.engine.set_sensor_enabled(s, enabled))
            self.sensor_enable_checks[sensor] = check
            layout.addWidget(check)
        layout.addStretch(1)
        return layout

    def _build_service_bar(self) -> QHBoxLayout:
        layout = QHBoxLayout()
        service_box = QGroupBox("Service / Provisioning")
        form = QGridLayout(service_box)

        self.uid_edit = QLineEdit("00000000")
        self.assign_sensor_spin = QSpinBox()
        self.assign_sensor_spin.setRange(0, SENSOR_TAB_COUNT - 1)
        self.assign_node_spin = QSpinBox()
        self.assign_node_spin.setRange(1, NODES_PER_SENSOR)
        self.profile_spin = QSpinBox()
        self.profile_spin.setRange(0, SENSOR_TAB_COUNT - 1)
        self.test_rpm_spin = QDoubleSpinBox()
        self.test_rpm_spin.setRange(-1000.0, 1000.0)
        self.test_rpm_spin.setValue(60.0)
        self.test_rpm_spin.setDecimals(1)
        self.test_duration_spin = QSpinBox()
        self.test_duration_spin.setRange(1, 30)
        self.test_duration_spin.setValue(3)
        self.slowdown_spin = QSpinBox()
        self.slowdown_spin.setRange(0, 100)
        self.slowdown_spin.setSuffix("%")
        self.blockage_spin = QSpinBox()
        self.blockage_spin.setRange(0, 100)
        self.blockage_spin.setSuffix("%")

        btn_discover = QPushButton("Discover")
        btn_discover.clicked.connect(self.send_discover)
        btn_assign = QPushButton("Assign")
        btn_assign.clicked.connect(self.send_assign)
        btn_save = QPushButton("Save")
        btn_save.clicked.connect(self.send_save)
        btn_cfg = QPushButton("Cfg Read")
        btn_cfg.clicked.connect(self.send_cfg_read)
        btn_diag = QPushButton("Diag Req")
        btn_diag.clicked.connect(self.send_diag_req)
        btn_identify = QPushButton("Identify")
        btn_identify.clicked.connect(self.send_identify)
        btn_test = QPushButton("Test Spin")
        btn_test.clicked.connect(self.send_test_spin)
        btn_can_src = QPushButton("Set CAN Source")
        btn_can_src.clicked.connect(self.send_set_can_source)
        btn_apply_impairment = QPushButton("Apply Impairment")
        btn_apply_impairment.clicked.connect(self.apply_impairment)
        btn_clear_impairment = QPushButton("Clear Impairment")
        btn_clear_impairment.clicked.connect(self.clear_impairment)

        form.addWidget(QLabel("UID32 Hex"), 0, 0)
        form.addWidget(self.uid_edit, 0, 1)
        form.addWidget(QLabel("Sensor"), 0, 2)
        form.addWidget(self.assign_sensor_spin, 0, 3)
        form.addWidget(QLabel("Node"), 0, 4)
        form.addWidget(self.assign_node_spin, 0, 5)
        form.addWidget(QLabel("Profile"), 0, 6)
        form.addWidget(self.profile_spin, 0, 7)
        form.addWidget(QLabel("Test RPM"), 1, 0)
        form.addWidget(self.test_rpm_spin, 1, 1)
        form.addWidget(QLabel("Duration"), 1, 2)
        form.addWidget(self.test_duration_spin, 1, 3)
        form.addWidget(QLabel("Slowdown"), 1, 4)
        form.addWidget(self.slowdown_spin, 1, 5)
        form.addWidget(QLabel("Blockage"), 1, 6)
        form.addWidget(self.blockage_spin, 1, 7)
        form.addWidget(btn_discover, 2, 0)
        form.addWidget(btn_assign, 2, 1)
        form.addWidget(btn_save, 2, 2)
        form.addWidget(btn_cfg, 2, 3)
        form.addWidget(btn_diag, 2, 4)
        form.addWidget(btn_identify, 2, 5)
        form.addWidget(btn_test, 2, 6)
        form.addWidget(btn_can_src, 2, 7)
        form.addWidget(btn_apply_impairment, 3, 4, 1, 2)
        form.addWidget(btn_clear_impairment, 3, 6, 1, 2)

        layout.addWidget(service_box)
        return layout

    def _build_sensor_tab(self, sensor: int) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        summary_box = QGroupBox(f"Sensor {sensor} Summary")
        summary_form = QFormLayout(summary_box)
        labels = {
            "mode": QLabel("-"),
            "flags": QLabel("-"),
            "base_rpm": QLabel("-"),
            "sync_pos": QLabel("-"),
            "sequence": QLabel("-"),
            "last_rx": QLabel("-"),
        }
        self.summary_labels[sensor] = labels
        summary_form.addRow("Mode", labels["mode"])
        summary_form.addRow("Flags", labels["flags"])
        summary_form.addRow("Base RPM", labels["base_rpm"])
        summary_form.addRow("Sync Pos", labels["sync_pos"])
        summary_form.addRow("Sequence", labels["sequence"])
        summary_form.addRow("Last RX age", labels["last_rx"])
        layout.addWidget(summary_box)

        table = QTableWidget(0, 18)
        table.setHorizontalHeaderLabels([
            "Node",
            "UID32",
            "Profile",
            "Enabled",
            "Section",
            "Target RPM",
            "Actual RPM",
            "Trim RPM",
            "Pos",
            "Bus V",
            "Current A",
            "Ctrl C",
            "Motor C",
            "Slow %",
            "Block %",
            "Warnings",
            "Faults",
            "Identify",
        ])
        table.verticalHeader().setVisible(False)
        self.sensor_tables[sensor] = table
        layout.addWidget(table, 1)
        return page

    def _uid_value(self) -> int:
        text = self.uid_edit.text().strip().lower().replace("0x", "")
        return int(text or "0", 16) & 0xFFFFFFFF

    def on_connect(self) -> None:
        try:
            self.engine.connect(self.channel_edit.text().strip(), int(self.bitrate_edit.text().strip()))
            self.statusBar().showMessage("CAN connected")
        except Exception as exc:
            QMessageBox.critical(self, "Connect error", str(exc))

    def on_start(self) -> None:
        try:
            self.engine.start()
            self.statusBar().showMessage("Simulation running")
        except Exception as exc:
            QMessageBox.critical(self, "Start error", str(exc))

    def on_stop(self) -> None:
        self.engine.stop()
        self.statusBar().showMessage("Simulation stopped")

    def on_diag_toggle(self, enabled: bool) -> None:
        self.engine.allow_diag_tx = enabled

    def send_discover(self) -> None:
        self.engine.send_msg(SERVICE_ID_DISCOVER, [1, 10, 1, 1, 0, 0, 0, 0])

    def send_assign(self) -> None:
        uid32 = self._uid_value()
        self.engine.send_msg(SERVICE_ID_ASSIGN, [uid32 & 0xFF, (uid32 >> 8) & 0xFF, (uid32 >> 16) & 0xFF, (uid32 >> 24) & 0xFF, self.assign_sensor_spin.value(), self.assign_node_spin.value(), 1, 1])

    def send_save(self) -> None:
        uid32 = self._uid_value()
        self.engine.send_msg(SERVICE_ID_SAVE_CFG, [uid32 & 0xFF, (uid32 >> 8) & 0xFF, (uid32 >> 16) & 0xFF, (uid32 >> 24) & 0xFF, 0xA5, 0, 0, 0])

    def send_cfg_read(self) -> None:
        uid32 = self._uid_value()
        self.engine.send_msg(SERVICE_ID_CFG_READ, [uid32 & 0xFF, (uid32 >> 8) & 0xFF, (uid32 >> 16) & 0xFF, (uid32 >> 24) & 0xFF, 0, 0, 0, 0])

    def send_diag_req(self) -> None:
        uid32 = self._uid_value()
        self.engine.send_msg(SERVICE_ID_DIAG_REQ, [uid32 & 0xFF, (uid32 >> 8) & 0xFF, (uid32 >> 16) & 0xFF, (uid32 >> 24) & 0xFF, 1, 1, 0, 0])

    def send_identify(self) -> None:
        uid32 = self._uid_value()
        self.engine.send_msg(SERVICE_ID_IDENTIFY, [uid32 & 0xFF, (uid32 >> 8) & 0xFF, (uid32 >> 16) & 0xFF, (uid32 >> 24) & 0xFF, 0, self.test_duration_spin.value(), 0, 0])

    def send_test_spin(self) -> None:
        uid32 = self._uid_value()
        rpm_x10 = int(round(self.test_rpm_spin.value() * 10.0))
        self.engine.send_msg(SERVICE_ID_TEST_SPIN, [uid32 & 0xFF, (uid32 >> 8) & 0xFF, (uid32 >> 16) & 0xFF, (uid32 >> 24) & 0xFF, rpm_x10 & 0xFF, (rpm_x10 >> 8) & 0xFF, self.test_duration_spin.value(), 0])

    def send_set_can_source(self) -> None:
        uid32 = self._uid_value()
        self.engine.send_msg(SERVICE_ID_SET_CAN_SOURCE, [uid32 & 0xFF, (uid32 >> 8) & 0xFF, (uid32 >> 16) & 0xFF, (uid32 >> 24) & 0xFF, self.assign_sensor_spin.value(), self.assign_node_spin.value(), self.profile_spin.value(), 1])

    def apply_impairment(self) -> None:
        self.engine.set_node_impairment(
            self.assign_sensor_spin.value(),
            self.assign_node_spin.value(),
            self.slowdown_spin.value(),
            self.blockage_spin.value(),
        )

    def clear_impairment(self) -> None:
        self.slowdown_spin.setValue(0)
        self.blockage_spin.setValue(0)
        self.engine.clear_node_impairment(
            self.assign_sensor_spin.value(),
            self.assign_node_spin.value(),
        )

    def refresh_ui(self) -> None:
        snap = self.engine.snapshot()
        now = time.time()
        for sensor in range(SENSOR_TAB_COUNT):
            g = snap["globals"][sensor]
            labels = self.summary_labels[sensor]
            enabled = snap["sensor_enabled"].get(sensor, True)
            labels["mode"].setText(f"{g.system_mode} ({MODE_NAMES.get(g.system_mode, '?')})")
            labels["flags"].setText(f"0x{g.control_flags:02X}")
            labels["base_rpm"].setText(f"{float(g.base_rpm_u16):.1f}")
            labels["sync_pos"].setText(f"{g.sync_pos_u16 / 65535.0:.4f}")
            labels["sequence"].setText(str(g.sequence))
            labels["last_rx"].setText("Disabled" if not enabled else ("-" if not g.valid else f"{max(0.0, now - g.last_rx):.2f}s"))
            self._fill_sensor_table(sensor, snap["nodes"], now)

        self.log_view.setPlainText("\n".join(snap["log_lines"]))
        self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())
        self.statusBar().showMessage(f"Running={snap['running']}  RX={snap['last_rx']}  TX={snap['last_tx']}  ERR={snap['last_error'] or '-'}")

    def _fill_sensor_table(self, sensor: int, nodes: List[SimNode], now: float) -> None:
        table = self.sensor_tables[sensor]
        sensor_nodes = [node for node in nodes if node.sensor_channel == sensor]
        sensor_nodes.sort(key=lambda n: n.node_id)
        table.setRowCount(len(sensor_nodes))
        for row, node in enumerate(sensor_nodes):
            values = [
                str(node.node_id),
                f"{node.uid32:08X}",
                str(node.can_profile_index),
                "Yes" if node.enabled else "No",
                "On" if node.section_active else "Off",
                f"{node.target_rpm:.1f}",
                f"{node.actual_rpm:.1f}",
                f"{node.trim_rpm_x10 / 10.0:.1f}",
                f"{node.pos_rev:.4f}",
                f"{node.bus_voltage:.1f}",
                f"{node.motor_current:.1f}",
                f"{node.controller_temp:.0f}",
                f"{node.motor_temp:.0f}",
                f"{node.slowdown_pct}",
                f"{node.blockage_pct}",
                f"0x{node.warning_flags:02X}",
                f"0x{node.fault_flags:02X}",
                "Blink" if now < node.identify_until else "",
            ]
            for col, value in enumerate(values):
                item = table.item(row, col)
                if item is None:
                    item = QTableWidgetItem()
                    table.setItem(row, col, item)
                item.setText(value)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)

            if not self.engine.sensor_enabled.get(sensor, True):
                color = QColor("#e0e0e0")
            elif now < node.identify_until:
                color = QColor("#ffd54f")
            elif node.fault_flags:
                color = QColor("#ff8a80")
            elif node.warning_flags:
                color = QColor("#fff59d")
            elif node.should_run:
                color = QColor("#c8e6c9")
            else:
                color = QColor("white")

            for col in range(table.columnCount()):
                table.item(row, col).setBackground(color)

    def closeEvent(self, event) -> None:
        try:
            self.engine.shutdown()
        finally:
            super().closeEvent(event)


def main() -> int:
    app = QApplication([])
    window = MotorSimulatorWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
