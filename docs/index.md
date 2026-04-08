# ESP32-CRSF-Relay

A high-performance CRSF relay node that forwards control frames from an ELRS receiver to a high-power TX module to extend usable radio range.

## Why This Project Exists

This relay acts as a transparent bridge:

1. ELRS receiver decodes incoming CRSF channel frames.
2. ESP32 repackages and forwards them.
3. TX module retransmits them toward the aircraft.

In field testing, this architecture approximately doubled effective communication range versus a direct single-hop setup, while keeping added latency very low.

## Key Capabilities

- Dual-core task split on ESP32 for deterministic radio forwarding.
- Real-time Wi-Fi dashboard at `http://192.168.4.1`.
- Telemetry visibility (RSSI, LQ, SNR, channel values).
- Configurable throttle limiter for training and safety.
- Custom half-duplex interface PCB for JR-bay style signal line.

## Documentation Map

Use the left navigation to move through:

- System architecture and timing model.
- Hardware pin mapping and UART configuration.
- CRSF framing, packing, and CRC behavior.
- Firmware module/function reference.
- Dashboard API, status model, and troubleshooting.

## Tested Hardware

### Input Receivers

- HGLRC ELRS 2.4G
- Radiomaster XR1 Nano ELRS (tested at 868 MHz)

### Output Module

- Emax Aeris Link Nano 900 MHz

!!! warning "Power guidance"
    Do not power high-output modules from USB or ESP32 board rails. Use an external supply (for example, 2S LiPo) sized for peak current.
