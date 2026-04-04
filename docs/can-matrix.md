# CAN Matrix

## Overview

This project uses one shared 11-bit standard CAN bus between the Teensy ECU and the motor nodes.

Current runtime settings from firmware:
- Bitrate: `250 kbit/s`
- Sensor channels compiled in: `4`
- Runtime-active sensor channels: `1..4`
- Sections / node range per channel: `1..16`

The ECU publishes one CAN bank per sensor channel:
- `S0` bank starts at `0x080`
- `S1` bank starts at `0x200`
- `S2` bank starts at `0x300`
- `S3` bank starts at `0x400`

There is also a dedicated service CAN range:
- `0x500..0x50E`

## Bank Layout

Each sensor channel uses the same message layout, only the base ID changes.

| Channel | GLOBAL_CONTROL | GLOBAL_TIMEBASE | GLOBAL_ESTOP | NODE_CMD base | NODE_PRESENCE base | NODE_CFG_ACK base | NODE_STATUS base | NODE_DIAG base |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `S0` | `0x080` | `0x081` | `0x082` | `0x100` | `0x140` | `0x160` | `0x180` | `0x1C0` |
| `S1` | `0x200` | `0x201` | `0x202` | `0x210` | `0x240` | `0x260` | `0x280` | `0x2C0` |
| `S2` | `0x300` | `0x301` | `0x302` | `0x310` | `0x340` | `0x360` | `0x380` | `0x3C0` |
| `S3` | `0x400` | `0x401` | `0x402` | `0x410` | `0x440` | `0x460` | `0x480` | `0x4C0` |

Node-specific messages are formed as:
- `NODE_CMD = node_cmd_base + node_id`
- `NODE_PRESENCE = node_pres_base + node_id`
- `NODE_CFG_ACK = node_cfg_ack_base + node_id`
- `NODE_STATUS = node_status_base + node_id`
- `NODE_DIAG = node_diag_base + node_id`

Example:
- `S1 node 4 NODE_CMD = 0x210 + 4 = 0x214`
- `S2 node 10 NODE_STATUS = 0x380 + 10 = 0x38A`

## Runtime Messages

### GLOBAL_CONTROL

Purpose:
- Common operating mode and reference for one sensor channel

Message IDs:
- `S0_GLOBAL_CONTROL = 0x080`
- `S1_GLOBAL_CONTROL = 0x200`
- `S2_GLOBAL_CONTROL = 0x300`
- `S3_GLOBAL_CONTROL = 0x400`

Payload:

| Byte | Signal | Type | Unit | Description |
|---|---|---|---|---|
| 0 | `system_mode` | `uint8` | - | `OFF`, `MANUAL`, `AUTO`, `CALIBRATION`, `FAULT_HOLD` |
| 1 | `control_flags` | `uint8` | bitfield | Runtime control flags |
| 2-3 | `base_rpm_x10` | `int16` | rpm | Common target motor RPM for this channel |
| 4-5 | `sync_pos_u16` | `uint16` | rev | Sync reference position |
| 6 | `sequence` | `uint8` | - | Rolling sequence counter |
| 7 | `reserved` | `uint8` | - | Reserved |

`control_flags` bits:
- `bit0` `CTRL_DRIVE_ENABLE`
- `bit1` `CTRL_SYNC_ENABLE`
- `bit2` `CTRL_ESTOP`
- `bit3` `CTRL_DIAG_ENABLE`

`CTRL_DIAG_ENABLE` meaning:
- `0`: node periodic `NODE_DIAG` traffic may stay disabled
- `1`: node periodic `NODE_DIAG` traffic is allowed

### GLOBAL_TIMEBASE

Purpose:
- Reserved channel-wide timing payload

Message IDs:
- `0x081`, `0x201`, `0x301`, `0x401`

Payload:
- Raw 8-byte payload

### GLOBAL_ESTOP

Purpose:
- Emergency stop command

Message IDs:
- `0x082`, `0x202`, `0x302`, `0x402`

Payload:

| Byte | Signal | Type | Description |
|---|---|---|---|
| 0 | `estop_cmd` | `uint8` | `0xA5` means emergency stop |
| 1 | `reason` | `uint8` | `manual`, `comm_fault`, `master_fault` |
| 2-7 | reserved | - | Reserved |

### NODE_CMD

Purpose:
- Per-node runtime command

Message IDs:
- `S0`: `0x101..0x110`
- `S1`: `0x211..0x220`
- `S2`: `0x311..0x320`
- `S3`: `0x411..0x420`

Payload:

| Byte | Signal | Type | Unit | Description |
|---|---|---|---|---|
| 0 | `node_command` | `uint8` | - | `NOP`, `ENABLE`, `DISABLE`, `ZERO_POS`, `CLEAR_FAULT` |
| 1 | `node_flags` | `uint8` | bitfield | Per-node flags |
| 2-3 | `trim_rpm_x10` | `int16` | rpm | Signed trim RPM correction |
| 4-5 | `section_mask` | `uint16` | bitmask | Full 16-bit section mask |
| 6 | `sequence` | `uint8` | - | Rolling sequence counter |
| 7 | `reserved` | `uint8` | - | Reserved |

`node_flags` bits:
- `bit0` `NODE_FLAG_ALLOW_RUN`
- `bit1` `NODE_FLAG_INVERT_DIR`
- `bit2` `NODE_FLAG_USE_LOCAL_SENSOR`

Important:
- Bytes `4-5` are no longer a position offset in current firmware
- They carry the complete 16-bit `section_mask`
- Each node decides whether its section is active based on its own `node_id`

### NODE_PRESENCE

Purpose:
- Optional low-rate presence / heartbeat message from node

Message IDs:
- `S0`: `0x141..0x150`
- `S1`: `0x241..0x250`
- `S2`: `0x341..0x350`
- `S3`: `0x441..0x450`

Payload:
- Raw 8-byte payload

### NODE_CFG_ACK

Purpose:
- Optional per-node config acknowledgement channel

Message IDs:
- `S0`: `0x161..0x170`
- `S1`: `0x261..0x270`
- `S2`: `0x361..0x370`
- `S3`: `0x461..0x470`

Payload:
- Raw 8-byte payload

### NODE_STATUS_FAST

Purpose:
- Fast runtime node feedback

Message IDs:
- `S0`: `0x181..0x190`
- `S1`: `0x281..0x290`
- `S2`: `0x381..0x390`
- `S3`: `0x481..0x490`

Payload:

| Byte | Signal | Type | Unit | Description |
|---|---|---|---|---|
| 0 | `status_flags` | `uint8` | bitfield | Node runtime state |
| 1 | `error_code` | `uint8` | - | Main error code |
| 2-3 | `actual_rpm_x10` | `int16` | rpm | Actual motor RPM |
| 4-5 | `actual_pos_u16` | `uint16` | rev | Actual node position |
| 6 | `alive_counter` | `uint8` | - | Rolling heartbeat counter |
| 7 | `sync_error_x256rev` | `int8` | rev | Signed sync error |

### NODE_DIAG

Purpose:
- Detailed node diagnostics

Message IDs:
- `S0`: `0x1C1..0x1D0`
- `S1`: `0x2C1..0x2D0`
- `S2`: `0x3C1..0x3D0`
- `S3`: `0x4C1..0x4D0`

Payload:

| Byte | Signal | Type | Unit | Description |
|---|---|---|---|---|
| 0-1 | `bus_voltage_x10` | `uint16` | V | DC bus voltage |
| 2-3 | `motor_current_x10` | `int16` | A | Motor current |
| 4 | `controller_temp_c` | `uint8` | degC | Controller temperature |
| 5 | `motor_temp_c` | `uint8` | degC | Motor temperature |
| 6 | `fault_flags` | `uint8` | bitfield | Fault bits |
| 7 | `warning_flags` | `uint8` | bitfield | Warning bits |

## Service CAN

Service CAN is separate from normal runtime traffic and uses fixed IDs:

| ID | Name |
|---:|---|
| `0x500` | `SRV_DISCOVER` |
| `0x501` | `SRV_UID_A` |
| `0x502` | `SRV_UID_B` |
| `0x503` | `SRV_ASSIGN` |
| `0x504` | `SRV_SAVE_CFG` |
| `0x505` | `SRV_ACK` |
| `0x506` | `SRV_TEST_SPIN` |
| `0x507` | `SRV_DIAG_REQ` |
| `0x508` | `SRV_DIAG_RESP_A` |
| `0x509` | `SRV_DIAG_RESP_B` |
| `0x50A` | `SRV_REBOOT` |
| `0x50B` | `SRV_IDENTIFY` |
| `0x50C` | `SRV_CFG_READ` |
| `0x50D` | `SRV_CFG_RESP` |
| `0x50E` | `SRV_SET_CAN_SOURCE` |

Use cases:
- Node discovery
- UID query
- Assign node to sensor channel / node ID
- Save config
- Test spin
- Request node diagnostics

## Notes

- The ECU currently runs at `250 kbit/s`, not `500 kbit/s`
- Full 16-bit section masks are supported
- Four sensor banks are compiled in
- Detailed node diag is available on CAN and can also be forwarded by the ECU over UDP
