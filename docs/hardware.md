# Hardware Interface

## GPIO Mapping

| Macro | GPIO | Direction | UART | Function |
| --- | --- | --- | --- | --- |
| `RE_TX_PIN` | 32 | Output to receiver RX | UART2 | Data to ELRS receiver |
| `RE_RX_PIN` | 33 | Input from receiver TX | UART2 | CRSF data from ELRS receiver |
| `JR_TX_PIN` | 17 | Output to module data line | UART1 | Stick packet TX |
| `JR_RX_PIN` | 16 | Input from module data line | UART1 | Telemetry and heartbeat RX |
| `BUFFER_EN_PIN` | 25 | Output | - | SN74HC125 direction enable |

## UART Configuration

| Parameter | Receiver UART | Module UART |
| --- | --- | --- |
| Baud | 420000 | 400000 |
| Data bits | 8 | 8 |
| Parity | None | None |
| Stop bits | 1 | 1 |
| Flow control | None | None |
| Signal inversion | No | TX and RX inverted |
| RX buffer | 2048 bytes | 2048 bytes |

!!! note "Why inversion is required"
    JR-style CRSF signaling on module side is inverted. The firmware applies line inversion for that UART.

## Power Topology

Recommended setup:

- 2S LiPo feeds module power directly.
- ESP32 is fed via a regulated path (buck converter if needed).

!!! warning "Brownout prevention"
    High-power module operation can exceed current available from USB. Use external power with sufficient current headroom.

## Half-Duplex Direction Control

Direction is controlled through `BUFFER_EN_PIN` and the SN74HC125 buffer:

- `LOW`: transmit mode, buffer drives line.
- `HIGH`: receive mode, buffer high-impedance.

Transmission sequence:

1. Drive `BUFFER_EN_PIN` low.
2. Wait 2 microseconds for settling.
3. Send 26-byte frame.
4. Wait about 680 microseconds for full transfer.
5. Drive `BUFFER_EN_PIN` high.
