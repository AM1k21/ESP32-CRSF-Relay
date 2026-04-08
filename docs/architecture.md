# System Architecture

## High-Level Data Path

```text
ELRS Receiver (CRSF/UART, 420k)
        |
        v
ESP32 Relay Node
  - decode / pack / forward
  - half-duplex direction control
        |
        v
ELRS TX Module (CRSF, 400k)
```

The ESP32 also runs a SoftAP web dashboard on `192.168.4.1` for local monitoring and control.

## Core Assignment

| Core | Workload | Priority | Purpose |
| --- | --- | --- | --- |
| Core 1 | `RadioTask` | High (5) | UART RX/TX handling, packet forwarding, half-duplex control |
| Core 0 | `loop()` + HTTP handling | Normal (1) | Wi-Fi stack and web server |

This separation protects radio forwarding from Wi-Fi-induced jitter.

## Radio Loop Behavior

`RadioTask` runs continuously:

1. `processReceiverInput()` decodes incoming stick frames.
2. `processJrModule()` handles heartbeat and telemetry from module UART.
3. `sendStickPacket()` transmits keepalive if no TX event for 20 ms.
4. `vTaskDelay(1)` yields briefly.

## Timing and Latency

Approximate one-hop relay overhead is below 2 ms:

- Scheduler granularity: about 1 ms.
- UART TX for 26 bytes at 400 kbaud: about 650 microseconds.

This keeps control response effectively transparent for piloting.
