# Firmware Reference

## Runtime Roles

| Function | Role | Core |
| --- | --- | --- |
| `setup()` | Initializes GPIO/UART/RTOS task/Wi-Fi/AP server | Core 0 |
| `loop()` | Handles HTTP clients | Core 0 |
| `RadioTask(void*)` | Main radio processing loop | Core 1 |
| `processReceiverInput()` | Parses receiver CRSF channel frames | Core 1 |
| `processJrModule()` | Handles module telemetry and heartbeat | Core 1 |
| `sendStickPacket()` | Builds and transmits outbound stick frame | Core 1 |
| `packCrsfChannels()` | Bit-packs channels into payload | Core 1 |
| `calc_crc8()` | CRSF CRC computation | Core 1 |
| `handleRoot()` | Serves dashboard HTML | Core 0 |
| `handleData()` | Serves live JSON telemetry | Core 0 |
| `handleLimit()` | Updates throttle scale value | Core 0 |

## Important Global State

| Variable | Type | Volatile | Meaning |
| --- | --- | --- | --- |
| `currentChannels[16]` | `uint16_t[]` | Yes | Current decoded channel values |
| `disp_rssi` | `int` | Yes | Display RSSI |
| `disp_lq` | `int` | Yes | Display link quality |
| `disp_snr` | `int` | Yes | Display SNR |
| `throttleScale` | `int` | Yes | Throttle limiter percentage |
| `lastPacketTime` | `unsigned long` | No | Last TX timestamp |
| `lastRcInputTime` | `unsigned long` | No | Last receiver frame timestamp |
| `lastTelemTime` | `unsigned long` | No | Last telemetry timestamp |

## Timing Thresholds

| Event | Threshold | Outcome |
| --- | --- | --- |
| Keepalive TX | 20 ms | Force stick packet if heartbeat did not trigger send |
| RC timeout | 100 ms | `RC LOST` |
| Module packet timeout | 100 ms | `MODULE OFF` |
| Telemetry timeout | 1000 ms | `DRONE LOST` |
| Dashboard polling | 100 ms | Frontend update rate |
