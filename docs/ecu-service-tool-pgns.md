# ECU Service Tool PGNs

## Overview

The PySide ECU Service Tool talks to the Teensy ECU over UDP using custom 11-byte packets:

- bytes `0-1`: `PGN` little-endian
- bytes `2-9`: 8-byte payload
- byte `10`: simple checksum

Default ports:
- ECU listens on `UDP 28888`
- Tool listens for ECU replies on `UDP 30001`

## Packet Format

| Byte | Meaning |
|---|---|
| 0 | PGN low byte |
| 1 | PGN high byte |
| 2-9 | Payload |
| 10 | CRC = sum of previous bytes mod 256 |

## ECU Config PGNs

| PGN | Name | Direction | Purpose |
|---:|---|---|---|
| `32800` | `PGN_ECU_CFG_GET` | Tool -> ECU | Request one config block |
| `32801` | `PGN_ECU_CFG_SET` | Tool -> ECU | Write one config block |
| `32802` | `PGN_ECU_CFG_SAVE` | Tool -> ECU | Save runtime config to non-volatile storage |
| `32803` | `PGN_ECU_CFG_LOAD` | Tool -> ECU | Load config from non-volatile storage |
| `32804` | `PGN_ECU_CFG_STATUS` | ECU -> Tool | Return current config block values |

### Config block IDs

| ID | Name |
|---:|---|
| `0` | `CFG_BLOCK_MACHINE` |
| `1` | `CFG_BLOCK_DRIVE` |
| `2` | `CFG_BLOCK_DIAG` |
| `3` | `CFG_BLOCK_CHANNEL` |
| `4` | `CFG_BLOCK_NETWORK` |

### `32800` `PGN_ECU_CFG_GET`

Payload:

| Byte | Meaning |
|---|---|
| 0 | `block_id` |
| 1 | `index` |
| 2-7 | Reserved |

### `32801` `PGN_ECU_CFG_SET`

Payload:

| Byte | Meaning |
|---|---|
| 0 | `block_id` |
| 1 | `index` |
| 2-7 | Block-specific data |

#### Block `0` Machine Config

| Byte | Meaning |
|---|---|
| 2 | `active_sensor_count` |
| 3 | `configured_row_count` |
| 4-5 | `holes_per_rev` |
| 6-7 | `upm_scale_x10` |

#### Block `1` Drive Config

| Byte | Meaning |
|---|---|
| 2-3 | `gear_ratio_x100` |
| 4-5 | `trim_limit_rpm_x10` |
| 6-7 | `position_kp_x1000` |

#### Block `2` Diag Config

| Byte | Meaning |
|---|---|
| 2 | `diag_enable` |
| 3 | `diag_stream_enable` |
| 4 | `diag_period_div10_ms` |
| 5 | `diag_detail_level` |
| 6-7 | Reserved |

#### Block `3` Channel Config

| Byte | Meaning |
|---|---|
| 2 | `channel_enable` |
| 3 | `can_profile_index` |
| 4 | `status_bit_mode` |
| 5-7 | Reserved |

#### Block `4` Network Config

| Byte | Meaning |
|---|---|
| 2 | `ip_last_octet` |
| 3 | `module_id` |
| 4-7 | Reserved |

### `32802` `PGN_ECU_CFG_SAVE`

Payload:

| Byte | Meaning |
|---|---|
| 0 | `magic = 0xA5` |
| 1 | `scope` |
| 2-7 | Reserved |

### `32803` `PGN_ECU_CFG_LOAD`

Payload:

| Byte | Meaning |
|---|---|
| 0 | `magic = 0x5A` |
| 1 | `scope` |
| 2-7 | Reserved |

### `32804` `PGN_ECU_CFG_STATUS`

Payload shape matches the requested config block:

| Byte | Meaning |
|---|---|
| 0 | `block_id` |
| 1 | `index` |
| 2-7 | Block-specific payload |

## ECU Diag PGNs

| PGN | Name | Direction | Purpose |
|---:|---|---|---|
| `32805` | `PGN_ECU_DIAG_CONTROL` | Tool -> ECU | Configure UDP diag stream |
| `32806` | `PGN_ECU_DIAG_STATUS` | ECU -> Tool | Global ECU state |
| `32807` | `PGN_ECU_DIAG_SENSOR` | ECU -> Tool | Per-sensor summary |
| `32808` | `PGN_ECU_DIAG_NODE_SUMMARY` | ECU -> Tool | Per-sensor node aggregate |
| `32809` | `PGN_ECU_DIAG_NODE_DETAIL_A` | ECU -> Tool | Node detail part A |
| `32810` | `PGN_ECU_DIAG_NODE_DETAIL_B` | ECU -> Tool | Node detail part B |
| `32811` | `PGN_ECU_DIAG_NODE_DETAIL_REQ` | Tool -> ECU | Request one node detail sample |

### `32805` `PGN_ECU_DIAG_CONTROL`

Payload:

| Byte | Meaning |
|---|---|
| 0 | `diag_enable` |
| 1 | `stream_enable` |
| 2 | `sensor_mask` |
| 3 | `node_mask_lo` |
| 4 | `node_mask_hi` |
| 5 | `period_div10_ms` |
| 6 | `detail_level` |
| 7 | Reserved |

`sensor_mask` bits:
- `bit0` `S0`
- `bit1` `S1`
- `bit2` `S2`
- `bit3` `S3`

### `32806` `PGN_ECU_DIAG_STATUS`

Payload:

| Byte | Meaning |
|---|---|
| 0 | `active_sensor_count` |
| 1 | `configured_row_count` |
| 2 | `eth_link` |
| 3 | `can_link` |
| 4 | `diag_enable` |
| 5 | `rc_timeout` |
| 6-7 | `uptime_s` |

### `32807` `PGN_ECU_DIAG_SENSOR`

Payload:

| Byte | Meaning |
|---|---|
| 0 | `sensor_channel` |
| 1 | `mode` |
| 2-3 | `base_rpm_x10` |
| 4-7 | `target_upm_x1000` |

### `32808` `PGN_ECU_DIAG_NODE_SUMMARY`

Payload:

| Byte | Meaning |
|---|---|
| 0 | `sensor_channel` |
| 1 | `online_nodes` |
| 2-3 | `avg_rpm_x10` |
| 4-5 | `total_rpm_x10` |
| 6-7 | `avg_pos_u16` |

### `32809` `PGN_ECU_DIAG_NODE_DETAIL_A`

Payload:

| Byte | Meaning |
|---|---|
| 0 | `sensor_channel` |
| 1 | `node_id` |
| 2 | `status_flags` |
| 3 | `error_code` |
| 4-5 | `actual_rpm_x10` |
| 6-7 | `actual_pos_u16` |

### `32810` `PGN_ECU_DIAG_NODE_DETAIL_B`

Payload:

| Byte | Meaning |
|---|---|
| 0 | `sensor_channel` |
| 1 | `node_id` |
| 2-3 | `bus_voltage_x10` |
| 4-5 | `motor_current_x10` |
| 6 | `controller_temp_c` |
| 7 | `motor_temp_c` |

Current implementation also keeps:
- `warning_flags`
- `fault_flags`

If needed, these can later move into an additional detail packet.

### `32811` `PGN_ECU_DIAG_NODE_DETAIL_REQ`

Payload:

| Byte | Meaning |
|---|---|
| 0 | `sensor_channel` |
| 1 | `node_id` |
| 2-7 | Reserved |

## Node Service PGNs

These are tool-facing UDP requests that the ECU converts into service CAN traffic.

| PGN | Name | Direction | Purpose |
|---:|---|---|---|
| `32900` | `PGN_NODE_DISCOVER` | Tool -> ECU | Start CAN discover |
| `32901` | `PGN_NODE_UID_A` | ECU -> Tool | Forward UID part A |
| `32902` | `PGN_NODE_UID_B` | ECU -> Tool | Forward UID part B |
| `32903` | `PGN_NODE_ASSIGN` | Tool -> ECU | Assign node to sensor / node id |
| `32904` | `PGN_NODE_SAVE_CFG` | Tool -> ECU | Save node config |
| `32905` | `PGN_NODE_ACK` | ECU -> Tool | Forward node ACK |
| `32906` | `PGN_NODE_TEST_SPIN` | Tool -> ECU | Start test spin |
| `32907` | `PGN_NODE_DIAG_REQ` | Tool -> ECU | Request node service diag |
| `32908` | `PGN_NODE_DIAG_RESP_A` | ECU -> Tool | Forward service diag part A |
| `32909` | `PGN_NODE_DIAG_RESP_B` | ECU -> Tool | Forward service diag part B |
| `32910` | `PGN_NODE_REBOOT` | Tool -> ECU | Reboot target node |
| `32911` | `PGN_NODE_IDENTIFY` | Tool -> ECU | Identify target node |
| `32912` | `PGN_NODE_CFG_READ` | Tool -> ECU | Read node config |
| `32913` | `PGN_NODE_CFG_RESP` | ECU -> Tool | Forward node config response |
| `32914` | `PGN_NODE_SET_CAN_SOURCE` | Tool -> ECU | Set node CAN source / profile |

## Recommended Usage

### Normal ECU setup

1. Read machine config
2. Set active sensor count, rows, holes, UPM scale
3. Set drive config
4. Save to ECU

### Slow diagnostic workflow

1. Enable diag
2. Enable summary stream
3. Read `32806`, `32807`, `32808`
4. Use `32811` for slow node detail polling

### Node provisioning workflow

1. Send discover
2. Inspect returned UID frames
3. Assign sensor channel and node id
4. Save node config
5. Optionally test spin and identify
