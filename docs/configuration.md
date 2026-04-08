# Configuration Reference

Configuration is defined at the top of `code/esp32code.ino` through constants and macros.

## Wi-Fi Settings

```cpp
const char* ssid = "Comm_Relay_Manager";
const char* password = "";
```

Set a password before field deployment if you need protected access.

## Pin Mapping

```cpp
#define BUFFER_EN_PIN 25
#define JR_TX_PIN     17
#define JR_RX_PIN     16
#define RE_TX_PIN     32
#define RE_RX_PIN     33
```

## Protocol Constants

```cpp
#define BUF_SIZE  1024
#define CRC_POLY  0xD5
```

## UART Speeds

| UART | Baud | Notes |
| --- | --- | --- |
| Receiver (`RE_UART`) | 420000 | ELRS receiver CRSF side |
| Module (`JR_UART`) | 400000 | JR side CRSF with inversion |
| USB debug serial | 115200 | Optional diagnostics |

## Safe Tuning Checklist

- Keep CRSF baud settings unchanged unless all endpoints are validated.
- Verify inversion behavior on module UART after wiring changes.
- Re-test status transitions (`RC LOST`, `MODULE OFF`, `DRONE LOST`) after edits.
