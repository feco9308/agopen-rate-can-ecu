import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from dataclasses import dataclass, field
from typing import Optional, Dict

import can

CHANNEL = "PCAN_USBBUS1"
BITRATE = 250000
NODE_COUNT = 6
HOLES_PER_REV = 26.0

ID_GLOBAL_CONTROL = 0x080
ID_GLOBAL_TIMEBASE = 0x081
ID_GLOBAL_ESTOP = 0x082
ID_NODE_CMD_BASE = 0x100
ID_NODE_PRESENCE_BASE = 0x140
ID_NODE_STATUS_BASE = 0x180
ID_NODE_DIAG_BASE = 0x1C0

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
    MODE_FAULT_HOLD: "FAULT_HOLD",
}

CMD_NOP = 0
CMD_ENABLE = 1
CMD_DISABLE = 2
CMD_ZERO_POS = 3
CMD_CLEAR_FAULT = 4

CTRL_DRIVE_ENABLE = 1 << 0
CTRL_SYNC_ENABLE = 1 << 1
CTRL_MANUAL_ACTIVE = 1 << 2
CTRL_SECTION_CTRL_ACTIVE = 1 << 3
CTRL_ESTOP = 1 << 4

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
WRN_TEMP_HIGH = 1 << 2
WRN_CURRENT_HIGH = 1 << 3
WRN_COMM_LATE = 1 << 4

FLT_OVERCURRENT = 1 << 0
FLT_UNDERVOLTAGE = 1 << 1
FLT_OVERVOLTAGE = 1 << 2
FLT_OVERTEMP = 1 << 3
FLT_ENCODER = 1 << 4
FLT_HALL = 1 << 5
FLT_STALL = 1 << 6
FLT_DRIVER = 1 << 7

STATUS_PERIOD_S = 0.02
DIAG_PERIOD_S = 0.10
PRESENCE_PERIOD_S = 1.0
CONTROL_TIMEOUT_S = 0.15


def i16_from_le(data, idx):
    value = data[idx] | (data[idx + 1] << 8)
    if value >= 0x8000:
        value -= 0x10000
    return value


def u16_from_le(data, idx):
    return data[idx] | (data[idx + 1] << 8)


def i16_to_le(value):
    value &= 0xFFFF
    return [value & 0xFF, (value >> 8) & 0xFF]


def u16_to_le(value):
    value &= 0xFFFF
    return [value & 0xFF, (value >> 8) & 0xFF]


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def wrap_rev(value):
    while value >= 1.0:
        value -= 1.0
    while value < 0.0:
        value += 1.0
    return value


def wrap_pos_error(ref_u16, act_u16):
    err = int(ref_u16) - int(act_u16)
    if err > 32767:
        err -= 65536
    if err < -32768:
        err += 65536
    return err


def set_or_clear_bit(flags: int, bit: int, enabled: bool) -> int:
    return (flags | bit) if enabled else (flags & ~bit)


def count_bits(v: int) -> int:
    return bin(v & 0xFF).count("1")


@dataclass
class GlobalControlState:
    system_mode: int = MODE_OFF
    control_flags: int = 0
    base_rpm_x10: int = 0
    sync_pos_u16: int = 0
    sequence: int = 0
    last_rx: float = 0.0
    valid: bool = False
    estop: bool = False
    estop_reason: int = 0
    section_mask: int = 0x3F
    active_sections: int = 6
    last_upm: float = 0.0
    upm_per_section: float = 0.0
    last_disc_rpm: float = 0.0


@dataclass
class NodeState:
    node_id: int
    cmd_id: int
    presence_id: int
    status_id: int
    diag_id: int

    enabled: bool = False
    allow_run: bool = True
    invert_dir: bool = False
    use_local_sensor: bool = False
    trim_rpm_x10: int = 0
    pos_offset_i16: int = 0
    section_active: bool = True
    should_run: bool = False

    control_mode: str = "AUTO"  # AUTO / MANUAL
    manual_rpm_enabled: bool = False
    manual_pos_enabled: bool = False
    manual_rpm: float = 0.0
    manual_pos_rev: float = 0.0

    actual_rpm: float = 0.0
    target_rpm: float = 0.0
    pos_rev: float = 0.0
    alive_counter: int = 0

    bus_voltage: float = 12.4
    motor_current: float = 0.0
    controller_temp: float = 35.0
    motor_temp: float = 32.0
    fault_flags: int = 0
    warning_flags: int = 0
    error_code: int = 0

    presence_enabled: bool = True
    status_enabled: bool = True
    diag_enabled: bool = True

    force_fault_driver: bool = False
    force_fault_overtemp: bool = False
    force_warning_sync: bool = False
    force_warning_rpm: bool = False

    load_pct: float = 0.0
    drag_pct: float = 0.0
    jammed: bool = False
    freeze: bool = False

    kp: float = 2.0
    ki: float = 0.8
    response_limit: float = 180.0
    sync_kp: float = 60.0
    current_gain: float = 0.03
    ramp_error_gain: float = 0.01

    integral: float = 0.0
    last_error_rpm: float = 0.0
    last_cmd_seq: int = 0


class CanSimulator:
    def __init__(self):
        self.lock = threading.Lock()
        self.bus: Optional[can.Bus] = None
        self.running = False
        self.rx_thread: Optional[threading.Thread] = None
        self.loop_thread: Optional[threading.Thread] = None

        self.g = GlobalControlState()
        self.nodes: Dict[int, NodeState] = {
            i: NodeState(
                i,
                ID_NODE_CMD_BASE + i,
                ID_NODE_PRESENCE_BASE + i,
                ID_NODE_STATUS_BASE + i,
                ID_NODE_DIAG_BASE + i,
            )
            for i in range(1, NODE_COUNT + 1)
        }

        self.last_error = ""
        self.last_rx_info = "-"
        self.last_tx_info = "-"
        self.log_lines: list[str] = []

    def log(self, line: str):
        timestamp = time.strftime("%H:%M:%S")
        entry = f"[{timestamp}] {line}"
        with self.lock:
            self.log_lines.append(entry)
            self.log_lines = self.log_lines[-500:]

    def connect(self, channel: str, bitrate: int):
        if self.bus is not None:
            self.disconnect()
        self.bus = can.Bus(interface="pcan", channel=channel, bitrate=bitrate)
        self.log(f"Kapcsolódva: {channel}, {bitrate} bit/s")

    def disconnect(self):
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
            self.log("CAN kapcsolat lezárva")

    def start(self):
        if self.bus is None:
            raise RuntimeError("Nincs megnyitott CAN busz.")
        if self.running:
            return
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.loop_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.rx_thread.start()
        self.loop_thread.start()
        self.log("Szimuláció elindult")

    def stop(self):
        self.running = False
        self.log("Szimuláció leállt")

    def shutdown(self):
        self.stop()
        time.sleep(0.05)
        self.disconnect()

    def send_msg(self, arbitration_id: int, data: list[int]):
        if self.bus is None:
            return
        msg = can.Message(arbitration_id=arbitration_id, is_extended_id=False, data=data)
        self.bus.send(msg)
        with self.lock:
            self.last_tx_info = f"0x{arbitration_id:03X}  {' '.join(f'{b:02X}' for b in data)}"

    def send_global_control_from_ui(self):
        with self.lock:
            data = [
                self.g.system_mode & 0xFF,
                self.g.control_flags & 0xFF,
                *i16_to_le(self.g.base_rpm_x10),
                *u16_to_le(self.g.sync_pos_u16),
                self.g.sequence & 0xFF,
                0,
            ]
            self.g.sequence = (self.g.sequence + 1) & 0xFF
        self.send_msg(ID_GLOBAL_CONTROL, data)
        self.log("Kézi GLOBAL_CONTROL küldve")

    def send_node_cmd_from_ui(self, node: NodeState, command: int):
        with self.lock:
            flags = 0
            flags = set_or_clear_bit(flags, NODE_FLAG_ALLOW_RUN, node.allow_run)
            flags = set_or_clear_bit(flags, NODE_FLAG_INVERT_DIR, node.invert_dir)
            flags = set_or_clear_bit(flags, NODE_FLAG_USE_LOCAL_SENSOR, node.use_local_sensor)
            node.last_cmd_seq = (node.last_cmd_seq + 1) & 0xFF
            data = [
                command & 0xFF,
                flags & 0xFF,
                *i16_to_le(node.trim_rpm_x10),
                *i16_to_le(node.pos_offset_i16),
                node.last_cmd_seq & 0xFF,
                0,
            ]
        self.send_msg(node.cmd_id, data)
        self.log(f"NODE_CMD küldve node{node.node_id}: cmd={command}")

    def trigger_estop(self, reason: int = 0):
        with self.lock:
            self.g.estop = True
            self.g.estop_reason = reason
        self.send_msg(ID_GLOBAL_ESTOP, [0xA5, reason & 0xFF, 0, 0, 0, 0, 0, 0])
        self.log("GLOBAL_ESTOP küldve")


    def _rx_loop(self):
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
                self.log(f"RX hiba: {exc}")
                time.sleep(0.1)

    def _handle_rx(self, msg: can.Message):
        data = list(msg.data)
        with self.lock:
            self.last_rx_info = f"0x{msg.arbitration_id:03X}  {' '.join(f'{b:02X}' for b in data)}"

        if msg.arbitration_id == ID_GLOBAL_CONTROL and len(data) >= 8:
            with self.lock:
                self.g.system_mode = data[0]
                self.g.control_flags = data[1]
                self.g.base_rpm_x10 = i16_from_le(data, 2)
                self.g.sync_pos_u16 = u16_from_le(data, 4)
                self.g.sequence = data[6]
                self.g.last_rx = time.time()
                self.g.valid = True
                self.g.active_sections = max(1, sum(1 for n in self.nodes.values() if n.section_active))
                self.g.section_mask = sum((1 << (i - 1)) for i, n in self.nodes.items() if n.section_active) & 0x3F
                self.g.last_disc_rpm = self.g.base_rpm_x10 / 10.0
                self.g.upm_per_section = self.g.last_disc_rpm * HOLES_PER_REV
                self.g.last_upm = self.g.upm_per_section * self.g.active_sections
            self.log(
                f"RX GLOBAL_CONTROL: mode={data[0]} flags=0x{data[1]:02X} rpm={i16_from_le(data, 2)/10.0:.1f}"
            )
            return

        if msg.arbitration_id == ID_GLOBAL_ESTOP and len(data) >= 2:
            with self.lock:
                if data[0] == 0xA5:
                    self.g.estop = True
                    self.g.estop_reason = data[1]
            self.log(f"RX GLOBAL_ESTOP reason={data[1]}")
            return

        if ID_NODE_CMD_BASE < msg.arbitration_id <= ID_NODE_CMD_BASE + NODE_COUNT and len(data) >= 8:
            node_id = msg.arbitration_id - ID_NODE_CMD_BASE
            self._apply_node_cmd(self.nodes[node_id], data)
            return

    def _apply_node_cmd(self, node: NodeState, data: list[int]):
        with self.lock:
            node_cmd = data[0]
            node_flags = data[1]
            node.allow_run = bool(node_flags & NODE_FLAG_ALLOW_RUN)
            node.invert_dir = bool(node_flags & NODE_FLAG_INVERT_DIR)
            node.use_local_sensor = bool(node_flags & NODE_FLAG_USE_LOCAL_SENSOR)
            node.trim_rpm_x10 = i16_from_le(data, 2)
            node.pos_offset_i16 = i16_from_le(data, 4)
            node.last_cmd_seq = data[6]

            if node_cmd == CMD_ENABLE:
                node.enabled = True
                node.section_active = True
            elif node_cmd == CMD_DISABLE:
                node.enabled = False
                node.section_active = False
                node.target_rpm = 0.0
            elif node_cmd == CMD_ZERO_POS:
                node.pos_rev = 0.0
            elif node_cmd == CMD_CLEAR_FAULT:
                node.fault_flags = 0
                node.warning_flags = 0
                node.error_code = 0
                node.force_fault_driver = False
                node.force_fault_overtemp = False
                node.jammed = False

        with self.lock:
            self.g.section_mask = sum((1 << (i - 1)) for i, n in self.nodes.items() if n.section_active) & 0x3F
            self.g.active_sections = max(1, sum(1 for n in self.nodes.values() if n.section_active))
            self.g.last_disc_rpm = self.g.base_rpm_x10 / 10.0
            self.g.upm_per_section = self.g.last_disc_rpm * HOLES_PER_REV
            self.g.last_upm = self.g.upm_per_section * self.g.active_sections
        self.log(f"RX NODE_CMD node{node.node_id}: cmd={data[0]} flags=0x{data[1]:02X}")

    def _main_loop(self):
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
                    for node in self.nodes.values():
                        self._update_node(node, dt)

                if now - last_status >= STATUS_PERIOD_S:
                    last_status = now
                    for node in self.nodes.values():
                        self._send_status(node)

                if now - last_diag >= DIAG_PERIOD_S:
                    last_diag = now
                    for node in self.nodes.values():
                        self._send_diag(node)

                if now - last_presence >= PRESENCE_PERIOD_S:
                    last_presence = now
                    for node in self.nodes.values():
                        self._send_presence(node)
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
                self.log(f"Loop hiba: {exc}")

            time.sleep(0.005)


    def _update_node(self, node: NodeState, dt: float):
        now = time.time()
        can_timeout = (not self.g.valid) or ((now - self.g.last_rx) > CONTROL_TIMEOUT_S)
        drive_enable = bool(self.g.control_flags & CTRL_DRIVE_ENABLE)
        sync_enable = bool(self.g.control_flags & CTRL_SYNC_ENABLE)
        estop_flag = bool(self.g.control_flags & CTRL_ESTOP) or self.g.estop
        mode_allows_run = self.g.system_mode in (MODE_MANUAL, MODE_AUTO, MODE_CALIBRATION)

        actual_pos_u16 = int(wrap_rev(node.pos_rev) * 65535.0) & 0xFFFF
        actual_pos_u16 = (actual_pos_u16 + (node.pos_offset_i16 & 0xFFFF)) & 0xFFFF
        pos_err = wrap_pos_error(self.g.sync_pos_u16, actual_pos_u16)
        pos_err_rev = pos_err / 65536.0

        sync_corr = node.sync_kp * pos_err_rev if sync_enable and node.control_mode == "AUTO" else 0.0
        cmd_rpm = (self.g.base_rpm_x10 / 10.0) + (node.trim_rpm_x10 / 10.0) + sync_corr
        if node.invert_dir:
            cmd_rpm = -cmd_rpm
        cmd_rpm = clamp(cmd_rpm, -250.0, 250.0)

        should_run = (
            node.enabled
            and node.allow_run
            and node.section_active
            and drive_enable
            and mode_allows_run
            and not can_timeout
            and not estop_flag
            and node.fault_flags == 0
        )
        node.should_run = should_run

        if not should_run:
            cmd_rpm = 0.0

        node.target_rpm = cmd_rpm

        if node.control_mode == "MANUAL":
            if node.manual_rpm_enabled:
                node.actual_rpm = clamp(node.manual_rpm, -250.0, 250.0)
            else:
                # kézi mód, de nincs rpm override: finom lecsengés
                node.actual_rpm *= 0.95
            if node.manual_pos_enabled:
                node.pos_rev = wrap_rev(node.manual_pos_rev)
            elif not node.freeze:
                node.pos_rev = wrap_rev(node.pos_rev + (node.actual_rpm / 60.0) * dt)
            node.integral = 0.0
        else:
            error = node.target_rpm - node.actual_rpm
            node.integral = clamp(node.integral + error * dt, -30.0, 30.0)
            accel_cmd = node.kp * error + node.ki * node.integral

            load_brake = abs(node.target_rpm) * (node.load_pct / 100.0) * 0.25
            drag_brake = (node.drag_pct / 100.0) * 40.0
            if node.actual_rpm > 0:
                accel_cmd -= (load_brake + drag_brake)
            elif node.actual_rpm < 0:
                accel_cmd += (load_brake + drag_brake)

            accel_cmd = clamp(accel_cmd, -abs(node.response_limit), abs(node.response_limit))

            if node.jammed:
                node.actual_rpm *= 0.6
                if abs(node.actual_rpm) < 0.2:
                    node.actual_rpm = 0.0
            else:
                node.actual_rpm += accel_cmd * dt

            if not should_run:
                node.integral = 0.0
                node.actual_rpm *= 0.85
                if abs(node.actual_rpm) < 0.05:
                    node.actual_rpm = 0.0

            node.actual_rpm = clamp(node.actual_rpm, -300.0, 300.0)

            if abs(node.actual_rpm) < 0.03 and abs(node.target_rpm) < 0.03:
                node.actual_rpm = 0.0

            if not node.freeze:
                node.pos_rev = wrap_rev(node.pos_rev + (node.actual_rpm / 60.0) * dt)

        node.motor_current = 0.4 + abs(node.actual_rpm) * node.current_gain + abs(node.target_rpm - node.actual_rpm) * node.ramp_error_gain
        node.motor_current = clamp(node.motor_current, 0.0, 20.0)
        node.bus_voltage = clamp(12.4 - node.motor_current * 0.08, 7.0, 16.0)
        node.controller_temp = clamp(32.0 + abs(node.actual_rpm) * 0.08 + node.load_pct * 0.15, 20.0, 95.0)
        node.motor_temp = clamp(29.0 + abs(node.actual_rpm) * 0.07 + node.drag_pct * 0.18, 20.0, 110.0)

        node.warning_flags = 0
        node.error_code = 0
        if abs(node.target_rpm - node.actual_rpm) > 5.0 or node.force_warning_rpm:
            node.warning_flags |= WRN_RPM_DEV
        if abs(pos_err_rev) > 0.03 or node.force_warning_sync:
            node.warning_flags |= WRN_SYNC_DEV
        if node.controller_temp > 65 or node.motor_temp > 80:
            node.warning_flags |= WRN_TEMP_HIGH
        if node.motor_current > 8.0:
            node.warning_flags |= WRN_CURRENT_HIGH
        if can_timeout:
            node.warning_flags |= WRN_COMM_LATE

        fault_flags = 0
        if estop_flag or node.force_fault_driver:
            fault_flags |= FLT_DRIVER
            node.error_code = 2
        if node.force_fault_overtemp:
            fault_flags |= FLT_OVERTEMP
            node.error_code = 4
        if node.jammed:
            fault_flags |= FLT_STALL
            node.error_code = 6
        node.fault_flags = fault_flags
    def _send_presence(self, node: NodeState):
        if not node.presence_enabled:
            return
        self.send_msg(node.presence_id, [node.node_id, 1, 0, 1, 0, 0x03, 0, 0])

    def _send_status(self, node: NodeState):
        if not node.status_enabled:
            return
        with self.lock:
            can_timeout = (not self.g.valid) or ((time.time() - self.g.last_rx) > CONTROL_TIMEOUT_S)
            status_flags = ST_READY | ST_POSITION_VALID
            if node.enabled:
                status_flags |= ST_ENABLED
            if abs(node.actual_rpm) > 1.0:
                status_flags |= ST_RUNNING
            if not (node.warning_flags & WRN_SYNC_DEV):
                status_flags |= ST_SYNC_LOCKED
            if node.warning_flags:
                status_flags |= ST_WARNING_ACTIVE
            if node.fault_flags:
                status_flags |= ST_FAULT_ACTIVE
            if can_timeout:
                status_flags |= ST_CAN_TIMEOUT

            rpm_x10 = int(round(node.actual_rpm * 10.0))
            pos_u16 = int(wrap_rev(node.pos_rev) * 65535.0) & 0xFFFF
            pos_u16 = (pos_u16 + (node.pos_offset_i16 & 0xFFFF)) & 0xFFFF
            pos_err = wrap_pos_error(self.g.sync_pos_u16, pos_u16)
            sync_err = int(clamp(pos_err / 256, -128, 127)) & 0xFF
            data = [
                status_flags,
                node.error_code & 0xFF,
                *i16_to_le(rpm_x10),
                *u16_to_le(pos_u16),
                node.alive_counter & 0xFF,
                sync_err,
            ]
            node.alive_counter = (node.alive_counter + 1) & 0xFF
        self.send_msg(node.status_id, data)

    def _send_diag(self, node: NodeState):
        if not node.diag_enabled:
            return
        with self.lock:
            bus_x10 = int(round(node.bus_voltage * 10.0))
            current_x10 = int(round(node.motor_current * 10.0))
            data = [
                *u16_to_le(bus_x10),
                *i16_to_le(current_x10),
                int(round(node.controller_temp)) & 0xFF,
                int(round(node.motor_temp)) & 0xFF,
                node.fault_flags & 0xFF,
                node.warning_flags & 0xFF,
            ]
        self.send_msg(node.diag_id, data)

    def get_snapshot(self):
        with self.lock:
            return {
                "global": GlobalControlState(**self.g.__dict__),
                "nodes": {nid: NodeState(**n.__dict__) for nid, n in self.nodes.items()},
                "last_error": self.last_error,
                "last_rx_info": self.last_rx_info,
                "last_tx_info": self.last_tx_info,
                "running": self.running,
                "log_lines": list(self.log_lines[-120:]),
            }


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PCAN 6-node diag szimulátor - node only")
        self.geometry("1680x980")
        self.minsize(1450, 820)

        self.sim = CanSimulator()
        self.channel_var = tk.StringVar(value=CHANNEL)
        self.bitrate_var = tk.StringVar(value=str(BITRATE))


        self.node_ui = {}
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._periodic_update)

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="Channel:").pack(side="left")
        ttk.Entry(top, textvariable=self.channel_var, width=16).pack(side="left", padx=(4, 10))
        ttk.Label(top, text="Bitrate:").pack(side="left")
        ttk.Entry(top, textvariable=self.bitrate_var, width=10).pack(side="left", padx=(4, 10))
        ttk.Button(top, text="Kapcsolódás", command=self.connect_bus).pack(side="left", padx=4)
        ttk.Button(top, text="Start", command=self.start_sim).pack(side="left", padx=4)
        ttk.Button(top, text="Stop", command=self.stop_sim).pack(side="left", padx=4)

        self.status_label = ttk.Label(top, text="Nincs kapcsolat", foreground="darkred")
        self.status_label.pack(side="right")

        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True, padx=8, pady=8)

        ecu_tab = ttk.Frame(notebook, padding=8)
        node_tab = ttk.Frame(notebook, padding=8)
        log_tab = ttk.Frame(notebook, padding=8)
        notebook.add(ecu_tab, text="Bus / Debug")
        notebook.add(node_tab, text="6 Node / Diag")
        notebook.add(log_tab, text="Log")

        self._build_bus_tab(ecu_tab)
        self._build_nodes_tab(node_tab)
        self._build_log_tab(log_tab)


    def _build_bus_tab(self, parent):
        info = ttk.LabelFrame(parent, text="Node-only mód", padding=12)
        info.pack(fill="x", pady=(0, 10))
        ttk.Label(
            info,
            text="A szimulátor ebben a verzióban nem küld master/ECU frame-eket. Csak figyeli a GLOBAL_CONTROL-t és a node commandokat, majd visszaküldi a node státusz/diag frame-eket.",
            justify="left",
        ).pack(anchor="w")

        summary = ttk.LabelFrame(parent, text="Busz állapot", padding=12)
        summary.pack(fill="x")
        self.run_state_var = tk.StringVar(value="stopped")
        self.last_rx_var = tk.StringVar(value="-")
        self.last_tx_var = tk.StringVar(value="-")
        self.last_err_var = tk.StringVar(value="-")
        self._kv(summary, 0, "Futás", self.run_state_var)
        self._kv(summary, 1, "Utolsó RX", self.last_rx_var)
        self._kv(summary, 2, "Utolsó TX", self.last_tx_var)
        self._kv(summary, 3, "Utolsó hiba", self.last_err_var)

        live = ttk.LabelFrame(parent, text="Élő busz / debug értékek", padding=12)
        live.pack(fill="x", pady=(10, 0))
        self.rx_mode_var = tk.StringVar(value="-")
        self.rx_flags_var = tk.StringVar(value="-")
        self.rx_rpm_var = tk.StringVar(value="-")
        self.rx_sync_var = tk.StringVar(value="-")
        self.rx_seq_var = tk.StringVar(value="-")
        self.rx_estop_var = tk.StringVar(value="-")
        self.rx_section_mask_var = tk.StringVar(value="-")
        self.rx_active_sections_var = tk.StringVar(value="-")
        self.rx_upm_var = tk.StringVar(value="-")
        self.rx_upm_per_section_var = tk.StringVar(value="-")
        self.rx_disc_rpm_var = tk.StringVar(value="-")
        self._kv(live, 0, "mode", self.rx_mode_var)
        self._kv(live, 1, "flags", self.rx_flags_var)
        self._kv(live, 2, "base_rpm", self.rx_rpm_var)
        self._kv(live, 3, "sync_pos", self.rx_sync_var)
        self._kv(live, 4, "sequence", self.rx_seq_var)
        self._kv(live, 5, "estop", self.rx_estop_var)
        self._kv(live, 6, "node sectionMask", self.rx_section_mask_var)
        self._kv(live, 7, "activeSections", self.rx_active_sections_var)
        self._kv(live, 8, "UPM", self.rx_upm_var)
        self._kv(live, 9, "UPM / section", self.rx_upm_per_section_var)
        self._kv(live, 10, "discRpm", self.rx_disc_rpm_var)
    def _build_nodes_tab(self, parent):
        container = ttk.Frame(parent)
        container.pack(fill="both", expand=True)
        canvas = tk.Canvas(container)
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)
        scroll_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        for idx in range(1, NODE_COUNT + 1):
            frame = ttk.LabelFrame(scroll_frame, text=f"Node {idx}", padding=8)
            r = (idx - 1) // 2
            c = (idx - 1) % 2
            frame.grid(row=r, column=c, sticky="nsew", padx=6, pady=6)
            self.node_ui[idx] = self._build_single_node_ui(frame, idx)
        for c in range(2):
            scroll_frame.columnconfigure(c, weight=1)

    def _build_single_node_ui(self, frame, node_id: int):
        ui = {}
        ui["control_mode_var"] = tk.StringVar(value="AUTO")
        ui["enabled_var"] = tk.BooleanVar(value=True)
        ui["allow_run_var"] = tk.BooleanVar(value=True)
        ui["invert_var"] = tk.BooleanVar(value=False)
        ui["local_sensor_var"] = tk.BooleanVar(value=False)
        ui["trim_var"] = tk.DoubleVar(value=0.0)
        ui["pos_offset_var"] = tk.DoubleVar(value=0.0)
        ui["manual_rpm_enabled_var"] = tk.BooleanVar(value=False)
        ui["manual_pos_enabled_var"] = tk.BooleanVar(value=False)
        ui["manual_rpm_var"] = tk.DoubleVar(value=0.0)
        ui["manual_pos_var"] = tk.DoubleVar(value=0.0)
        ui["presence_var"] = tk.BooleanVar(value=True)
        ui["status_var"] = tk.BooleanVar(value=True)
        ui["diag_var"] = tk.BooleanVar(value=True)
        ui["fault_driver_var"] = tk.BooleanVar(value=False)
        ui["fault_overtemp_var"] = tk.BooleanVar(value=False)
        ui["warn_sync_var"] = tk.BooleanVar(value=False)
        ui["warn_rpm_var"] = tk.BooleanVar(value=False)
        ui["load_var"] = tk.DoubleVar(value=0.0)
        ui["drag_var"] = tk.DoubleVar(value=0.0)
        ui["jam_var"] = tk.BooleanVar(value=False)
        ui["freeze_var"] = tk.BooleanVar(value=False)
        ui["kp_var"] = tk.DoubleVar(value=2.0)
        ui["ki_var"] = tk.DoubleVar(value=0.8)
        ui["response_var"] = tk.DoubleVar(value=180.0)
        ui["sync_kp_var"] = tk.DoubleVar(value=60.0)
        ui["actual_rpm_var"] = tk.StringVar(value="0.0")
        ui["target_rpm_var"] = tk.StringVar(value="0.0")
        ui["pos_var"] = tk.StringVar(value="0.0000")
        ui["alive_var"] = tk.StringVar(value="0")
        ui["faults_var"] = tk.StringVar(value="0x00")
        ui["warnings_var"] = tk.StringVar(value="0x00")
        ui["busv_var"] = tk.StringVar(value="12.4 V")
        ui["current_var"] = tk.StringVar(value="0.0 A")
        ui["ctemp_var"] = tk.StringVar(value="0 C")
        ui["mtemp_var"] = tk.StringVar(value="0 C")
        ui["section_state_var"] = tk.StringVar(value="ON")
        ui["should_run_var"] = tk.StringVar(value="NO")
        ui["running_var"] = tk.StringVar(value="NO")
        ui["error_var"] = tk.StringVar(value="0.0")
        ui["mode_state_var"] = tk.StringVar(value="AUTO")

        row = 0
        ttk.Label(frame, text="Mode").grid(row=row, column=0, sticky="w")
        mode_combo = ttk.Combobox(frame, textvariable=ui["control_mode_var"], values=["AUTO", "MANUAL"], width=10, state="readonly")
        mode_combo.grid(row=row, column=1, sticky="w")
        mode_combo.bind("<<ComboboxSelected>>", lambda _e, nid=node_id: self.push_node_ui(nid))
        ttk.Checkbutton(frame, text="enabled", variable=ui["enabled_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=2, sticky="w")
        ttk.Checkbutton(frame, text="allow_run", variable=ui["allow_run_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=3, sticky="w")
        row += 1

        ttk.Checkbutton(frame, text="invert_dir", variable=ui["invert_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=0, sticky="w")
        ttk.Checkbutton(frame, text="use_local_sensor", variable=ui["local_sensor_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=1, sticky="w")
        ttk.Checkbutton(frame, text="manual RPM", variable=ui["manual_rpm_enabled_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=2, sticky="w")
        ttk.Checkbutton(frame, text="manual POS", variable=ui["manual_pos_enabled_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=3, sticky="w")
        row += 1

        ttk.Label(frame, text="trim_rpm").grid(row=row, column=0, sticky="w")
        ttk.Scale(frame, from_=-100, to=100, variable=ui["trim_var"], orient="horizontal", length=160, command=lambda _v, nid=node_id: self.push_node_ui(nid)).grid(row=row, column=1, sticky="ew")
        ttk.Label(frame, textvariable=ui["trim_var"], width=8).grid(row=row, column=2, sticky="w")
        row += 1

        ttk.Label(frame, text="manual_rpm").grid(row=row, column=0, sticky="w")
        ttk.Scale(frame, from_=-300, to=300, variable=ui["manual_rpm_var"], orient="horizontal", length=160, command=lambda _v, nid=node_id: self.push_node_ui(nid)).grid(row=row, column=1, sticky="ew")
        ttk.Label(frame, textvariable=ui["manual_rpm_var"], width=8).grid(row=row, column=2, sticky="w")
        row += 1

        ttk.Label(frame, text="manual_pos [rev]").grid(row=row, column=0, sticky="w")
        ttk.Scale(frame, from_=0.0, to=1.0, variable=ui["manual_pos_var"], orient="horizontal", length=160, command=lambda _v, nid=node_id: self.push_node_ui(nid)).grid(row=row, column=1, sticky="ew")
        ttk.Label(frame, textvariable=ui["manual_pos_var"], width=8).grid(row=row, column=2, sticky="w")
        row += 1

        ttk.Label(frame, text="pos_offset [rev]").grid(row=row, column=0, sticky="w")
        ttk.Scale(frame, from_=-0.5, to=0.5, variable=ui["pos_offset_var"], orient="horizontal", length=160, command=lambda _v, nid=node_id: self.push_node_ui(nid)).grid(row=row, column=1, sticky="ew")
        ttk.Label(frame, textvariable=ui["pos_offset_var"], width=8).grid(row=row, column=2, sticky="w")
        row += 1

        ttk.Separator(frame, orient="horizontal").grid(row=row, column=0, columnspan=4, sticky="ew", pady=4)
        row += 1

        ttk.Checkbutton(frame, text="send presence", variable=ui["presence_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=0, sticky="w")
        ttk.Checkbutton(frame, text="send status", variable=ui["status_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=1, sticky="w")
        ttk.Checkbutton(frame, text="send diag", variable=ui["diag_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=2, sticky="w")
        row += 1

        ttk.Checkbutton(frame, text="fault_driver", variable=ui["fault_driver_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=0, sticky="w")
        ttk.Checkbutton(frame, text="fault_overtemp", variable=ui["fault_overtemp_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=1, sticky="w")
        ttk.Checkbutton(frame, text="warn_sync", variable=ui["warn_sync_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=2, sticky="w")
        ttk.Checkbutton(frame, text="warn_rpm", variable=ui["warn_rpm_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=3, sticky="w")
        row += 1

        ttk.Label(frame, text="Load %").grid(row=row, column=0, sticky="w")
        ttk.Scale(frame, from_=0, to=100, variable=ui["load_var"], orient="horizontal", length=140, command=lambda _v, nid=node_id: self.push_node_ui(nid)).grid(row=row, column=1, sticky="ew")
        ttk.Label(frame, textvariable=ui["load_var"], width=7).grid(row=row, column=2, sticky="w")
        row += 1

        ttk.Label(frame, text="Drag %").grid(row=row, column=0, sticky="w")
        ttk.Scale(frame, from_=0, to=100, variable=ui["drag_var"], orient="horizontal", length=140, command=lambda _v, nid=node_id: self.push_node_ui(nid)).grid(row=row, column=1, sticky="ew")
        ttk.Label(frame, textvariable=ui["drag_var"], width=7).grid(row=row, column=2, sticky="w")
        ttk.Checkbutton(frame, text="Jammed", variable=ui["jam_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=3, sticky="w")
        row += 1

        ttk.Checkbutton(frame, text="Freeze", variable=ui["freeze_var"], command=lambda nid=node_id: self.push_node_ui(nid)).grid(row=row, column=0, sticky="w")
        ttk.Label(frame, text="Kp").grid(row=row, column=1, sticky="e")
        ttk.Entry(frame, textvariable=ui["kp_var"], width=8).grid(row=row, column=2, sticky="w")
        ttk.Label(frame, text="Ki").grid(row=row, column=3, sticky="e")
        ttk.Entry(frame, textvariable=ui["ki_var"], width=8).grid(row=row, column=4, sticky="w")
        row += 1

        ttk.Label(frame, text="Response").grid(row=row, column=0, sticky="w")
        ttk.Entry(frame, textvariable=ui["response_var"], width=8).grid(row=row, column=1, sticky="w")
        ttk.Label(frame, text="Sync Kp").grid(row=row, column=2, sticky="e")
        ttk.Entry(frame, textvariable=ui["sync_kp_var"], width=8).grid(row=row, column=3, sticky="w")
        row += 1

        btns = ttk.Frame(frame)
        btns.grid(row=row, column=0, columnspan=5, sticky="ew", pady=4)
        ttk.Button(btns, text="CMD_ENABLE", command=lambda nid=node_id: self.send_node_cmd(nid, CMD_ENABLE)).pack(side="left", padx=2)
        ttk.Button(btns, text="CMD_DISABLE", command=lambda nid=node_id: self.send_node_cmd(nid, CMD_DISABLE)).pack(side="left", padx=2)
        ttk.Button(btns, text="ZERO_POS", command=lambda nid=node_id: self.send_node_cmd(nid, CMD_ZERO_POS)).pack(side="left", padx=2)
        ttk.Button(btns, text="CLEAR_FAULT", command=lambda nid=node_id: self.send_node_cmd(nid, CMD_CLEAR_FAULT)).pack(side="left", padx=2)
        row += 1

        live = ttk.LabelFrame(frame, text="Élő állapot / diag", padding=6)
        live.grid(row=row, column=0, columnspan=5, sticky="nsew")
        self._kv(live, 0, "Mode", ui["mode_state_var"])
        self._kv(live, 1, "Section Active", ui["section_state_var"])
        self._kv(live, 2, "Should Run", ui["should_run_var"])
        self._kv(live, 3, "Actual Running", ui["running_var"])
        self._kv(live, 4, "actual_rpm", ui["actual_rpm_var"])
        self._kv(live, 5, "target_rpm", ui["target_rpm_var"])
        self._kv(live, 6, "rpm_error", ui["error_var"])
        self._kv(live, 7, "position_rev", ui["pos_var"])
        self._kv(live, 8, "alive", ui["alive_var"])
        self._kv(live, 9, "faults", ui["faults_var"])
        self._kv(live, 10, "warnings", ui["warnings_var"])
        self._kv(live, 11, "bus_voltage", ui["busv_var"])
        self._kv(live, 12, "motor_current", ui["current_var"])
        self._kv(live, 13, "ctrl_temp", ui["ctemp_var"])
        self._kv(live, 14, "motor_temp", ui["mtemp_var"])

        for col in range(5):
            frame.columnconfigure(col, weight=1)
        frame.rowconfigure(row, weight=1)
        return ui

    def _build_log_tab(self, parent):
        self.log_text = tk.Text(parent, wrap="none", height=30)
        self.log_text.pack(fill="both", expand=True)
        self.log_text.configure(state="disabled")

    def _kv(self, parent, row, key, value_var):
        ttk.Label(parent, text=key + ":").grid(row=row, column=0, sticky="w", padx=(0, 8), pady=1)
        ttk.Label(parent, textvariable=value_var).grid(row=row, column=1, sticky="w", pady=1)

    def connect_bus(self):
        try:
            bitrate = int(self.bitrate_var.get())
            self.sim.connect(self.channel_var.get().strip(), bitrate)
            self.status_label.config(text="Kapcsolódva", foreground="darkgreen")
        except Exception as exc:
            messagebox.showerror("Kapcsolódási hiba", str(exc))
            self.status_label.config(text="Kapcsolódási hiba", foreground="darkred")

    def start_sim(self):
        try:
            for nid in range(1, NODE_COUNT + 1):
                self.push_node_ui(nid)
            self.sim.start()
            self.status_label.config(text="Szimuláció fut", foreground="darkgreen")
        except Exception as exc:
            messagebox.showerror("Indítási hiba", str(exc))

    def stop_sim(self):
        self.sim.stop()
        self.status_label.config(text="Szimuláció áll", foreground="darkorange")


    def send_node_cmd(self, node_id: int, command: int):
        self.push_node_ui(node_id)
        node = self.sim.nodes[node_id]
        self.sim.send_node_cmd_from_ui(node, command)


    def clear_estop(self):
        self.sim.clear_estop()


    def push_node_ui(self, node_id: int):
        ui = self.node_ui[node_id]
        node = self.sim.nodes[node_id]
        with self.sim.lock:
            node.control_mode = ui["control_mode_var"].get()
            node.enabled = ui["enabled_var"].get()
            node.allow_run = ui["allow_run_var"].get()
            node.invert_dir = ui["invert_var"].get()
            node.use_local_sensor = ui["local_sensor_var"].get()
            node.trim_rpm_x10 = int(round(ui["trim_var"].get() * 10.0))
            node.pos_offset_i16 = int(round(ui["pos_offset_var"].get() * 65536.0))
            node.manual_rpm_enabled = ui["manual_rpm_enabled_var"].get()
            node.manual_pos_enabled = ui["manual_pos_enabled_var"].get()
            node.manual_rpm = float(ui["manual_rpm_var"].get())
            node.manual_pos_rev = float(ui["manual_pos_var"].get())
            node.presence_enabled = ui["presence_var"].get()
            node.status_enabled = ui["status_var"].get()
            node.diag_enabled = ui["diag_var"].get()
            node.force_fault_driver = ui["fault_driver_var"].get()
            node.force_fault_overtemp = ui["fault_overtemp_var"].get()
            node.force_warning_sync = ui["warn_sync_var"].get()
            node.force_warning_rpm = ui["warn_rpm_var"].get()
            node.load_pct = float(ui["load_var"].get())
            node.drag_pct = float(ui["drag_var"].get())
            node.jammed = ui["jam_var"].get()
            node.freeze = ui["freeze_var"].get()
            node.kp = float(ui["kp_var"].get())
            node.ki = float(ui["ki_var"].get())
            node.response_limit = float(ui["response_var"].get())
            node.sync_kp = float(ui["sync_kp_var"].get())


    def _periodic_update(self):
        snapshot = self.sim.get_snapshot()
        g = snapshot["global"]
        nodes = snapshot["nodes"]
        self.run_state_var.set("running" if snapshot["running"] else "stopped")
        self.last_rx_var.set(snapshot["last_rx_info"])
        self.last_tx_var.set(snapshot["last_tx_info"])
        self.last_err_var.set(snapshot["last_error"] or "-")
        self.rx_mode_var.set(f"{g.system_mode} ({MODE_NAMES.get(g.system_mode, '?')})")
        self.rx_flags_var.set(f"0x{g.control_flags:02X}")
        self.rx_rpm_var.set(f"{g.base_rpm_x10 / 10.0:.2f}")
        self.rx_sync_var.set(f"{g.sync_pos_u16 / 65535.0:.4f}")
        self.rx_seq_var.set(str(g.sequence))
        self.rx_estop_var.set(f"{g.estop} (reason={g.estop_reason})")
        self.rx_section_mask_var.set(f"0b{g.section_mask:06b}")
        self.rx_active_sections_var.set(str(g.active_sections))
        self.rx_upm_var.set(f"{g.last_upm:.3f}")
        self.rx_upm_per_section_var.set(f"{g.upm_per_section:.3f}")
        self.rx_disc_rpm_var.set(f"{g.last_disc_rpm:.3f}")
        for nid, node in nodes.items():
            self._fill_node_live(nid, node)
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.insert("end", "\n".join(snapshot["log_lines"]))
        self.log_text.see("end")
        self.log_text.configure(state="disabled")
        self.after(100, self._periodic_update)

    def _fill_node_live(self, node_id: int, node: NodeState):
        ui = self.node_ui[node_id]
        ui["mode_state_var"].set(node.control_mode)
        ui["section_state_var"].set("ON" if node.section_active else "OFF")
        ui["should_run_var"].set("YES" if node.should_run else "NO")
        ui["running_var"].set("YES" if (node.should_run and abs(node.actual_rpm) > 1.0) else "NO")
        ui["actual_rpm_var"].set(f"{node.actual_rpm:.2f}")
        ui["target_rpm_var"].set(f"{node.target_rpm:.2f}")
        ui["error_var"].set(f"{node.target_rpm - node.actual_rpm:+.2f}")
        ui["pos_var"].set(f"{node.pos_rev:.4f}")
        ui["alive_var"].set(str(node.alive_counter))
        ui["faults_var"].set(f"0x{node.fault_flags:02X}")
        ui["warnings_var"].set(f"0x{node.warning_flags:02X}")
        ui["busv_var"].set(f"{node.bus_voltage:.1f} V")
        ui["current_var"].set(f"{node.motor_current:.1f} A")
        ui["ctemp_var"].set(f"{node.controller_temp:.0f} C")
        ui["mtemp_var"].set(f"{node.motor_temp:.0f} C")

    def on_close(self):
        try:
            self.sim.shutdown()
        finally:
            self.destroy()


if __name__ == "__main__":
    app = App()
    app.mainloop()
