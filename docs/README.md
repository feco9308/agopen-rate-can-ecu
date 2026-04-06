# Documentation

This folder contains the main technical documentation for the CAN-based seeder ECU project.

## Contents

- `architecture.md`
  System structure, components, and high-level behavior.

- `can-matrix.md`
  Current runtime CAN message layout for sensor banks `S0..S3`, service CAN, and node feedback.

- `ecu-service-tool-pgns.md`
  Custom UDP PGNs used by the PySide ECU Service Tool and the Teensy ECU.

- `monitor-output.md`
  Optional external monitor output modes (`Off / Planter / Blockage`) and their data mapping.

- `session-notes.md`
  Rolling project notes and recent development context.

## Scope

The documented system contains:
- Teensy-based central ECU
- CAN motor nodes
- Ethernet / UDP interface toward PC tools and Rate App

The project replaces PWM motor control with CAN-based reference control using:
- RPM reference
- sync position reference
- node status and diagnostics

## Notes

- Current CAN bitrate in firmware is `250 kbit/s`
- Current firmware supports `4` sensor channels and `16` sections per channel
- Rate calculation logic lives in the ECU; the documentation here focuses on communication and control interfaces
