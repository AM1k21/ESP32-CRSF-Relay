# Web Dashboard and API

## Access Point

| Parameter | Value |
| --- | --- |
| SSID | `Comm_Relay_Manager` |
| Password | Open network (empty) |
| Gateway IP | `192.168.4.1` |
| Mode | ESP32 SoftAP |

## Endpoints

| Method | Path | Description |
| --- | --- | --- |
| GET | `/` | Dashboard UI (HTML/CSS/JS from firmware memory) |
| GET | `/data` | Live telemetry JSON |
| GET | `/set_limit?val=N` | Updates throttle limiter (`0` to `100`) |

## `/data` Response Shape

```json
{
  "rssi": -65,
  "lq": 98,
  "status": 3,
  "channels": [992, 992, 172, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992]
}
```

## Status Codes

| Code | Label | Condition |
| --- | --- | --- |
| 0 | `RC LOST` | No receiver input for 100 ms or more |
| 1 | `MODULE OFF` | No packet sent to module for 100 ms or more |
| 2 | `DRONE LOST` | No telemetry for 1000 ms or more |
| 3 | `ONLINE` | All links healthy |

Evaluation order in firmware is 0, then 1, then 2, else 3.

## UI Behavior

Dashboard polls `/data` every 100 ms and renders:

- Link stats (RSSI, LQ, SNR)
- Scrolling signal history graph
- 16 live channel bars
- System status badge

A separate limits tab controls throttle scaling via `set_limit`.
