# Build and Flash

## Prerequisites

- Arduino IDE 1.8+ or 2.x
- ESP32 board package installed
- Board target: `ESP32 Dev Module` (or compatible DevKit)

## Build Settings

| Setting | Value |
| --- | --- |
| Board | ESP32 Dev Module |
| Upload speed | 921600 |
| CPU frequency | 240 MHz |
| Flash frequency | 80 MHz |
| Flash size | 4 MB |
| Partition scheme | Default 4MB with spiffs |

## Flash Procedure

1. Open `code/esp32code.ino` in Arduino IDE.
2. Select board and COM port.
3. Upload firmware.
4. After boot, connect to `Comm_Relay_Manager` and open `http://192.168.4.1`.

## ELRS Pairing Notes

- Ensure bind phrase matches on each communicating ELRS pair.
- Receiver and corresponding transmitter endpoints must be configured for CRSF.
