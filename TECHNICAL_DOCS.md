# ESP32-CRSF-Relay — Technical Documentation

> **Version:** 1.1  
> **Last Updated:** 2025-12-22  
> **Platform:** ESP32 (WROOM-32) · Arduino Framework  
> **Language:** C++ (`.ino`)  
> **License:** MIT  

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Repository Structure](#2-repository-structure)
3. [System Architecture](#3-system-architecture)
4. [Hardware Interface](#4-hardware-interface)
5. [CRSF Protocol Implementation](#5-crsf-protocol-implementation)
6. [Firmware Module Reference](#6-firmware-module-reference)
7. [Web Dashboard & HTTP API](#7-web-dashboard--http-api)
8. [Half-Duplex PCB Design](#8-half-duplex-pcb-design)
9. [Build & Flash Instructions](#9-build--flash-instructions)
10. [Configuration Reference](#10-configuration-reference)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. Project Overview

The **ESP32-CRSF-Relay** is a CRSF relay node that extends the communication range of an ExpressLRS (ELRS) radio link. It acts as a transparent bridge: an ELRS receiver decodes incoming CRSF stick-channel frames from a remote transmitter and forwards them over UART to an ESP32. The ESP32 then re-packs and re-transmits those frames to a high-power ELRS TX module, which relays the signal onward to a drone or other vehicle.

In real-world testing, the relay node approximately **doubles the effective communication range** compared to a direct single-link setup. No noticeable latency is introduced by the relay — the forwarding loop runs at sub-millisecond intervals on a dedicated core.

### Core Design Goals

| Goal | Implementation |
| :--- | :--- |
| Zero packet loss | Radio task pinned to **Core 1** at FreeRTOS priority 5 |
| Real-time monitoring | WiFi dashboard on **Core 0** with 100 ms polling |
| Safety for training | Adjustable throttle limiter (Channel 3 scaling, 0–100 %) |
| No external app required | Self-hosted WiFi AP + embedded HTML/JS dashboard |
| Minimal hardware | Single ESP32 + custom half-duplex PCB + ELRS modules |

---

## 2. Repository Structure

```
ESP32-CRSF-Relay/
├── .gitignore
├── LICENSE                              # MIT License
├── README.md                            # User-facing documentation
├── TECHNICAL_DOCS.md                    # ← This file
├── code/
│   └── esp32code.ino                    # Main firmware (single-file, 426 lines)
├── PCB/
│   ├── halfduplextranslator.kicad_pcb   # KiCad PCB layout
│   ├── halfduplextranslator.kicad_pro   # KiCad project file
│   └── halfduplextranslator.kicad_sch   # KiCad schematic
├── halfduplextranslator.kicad_pcb       # (root-level copies of PCB files)
├── halfduplextranslator.kicad_pro
└── halfduplextranslator.kicad_sch
```

### Key Files

| File | Purpose |
| :--- | :--- |
| `code/esp32code.ino` | Complete firmware — UART drivers, CRSF parsing, channel packing, half-duplex switching, WiFi AP, web server, and embedded HTML dashboard |
| `PCB/halfduplextranslator.kicad_sch` | Schematic for the full-duplex-to-half-duplex translator board |
| `PCB/halfduplextranslator.kicad_pcb` | PCB layout (KiCad 7+) |

---

## 3. System Architecture

### 3.1 High-Level Data Flow

```
┌──────────────┐    CRSF/UART     ┌─────────────┐    Half-Duplex    ┌──────────────┐
│  ELRS         │  ─────────────►  │   ESP32       │  ──────────────►  │  ELRS TX      │
│  Receiver     │    420 kbaud     │   Relay Node  │    400 kbaud     │  Module       │
│  (Input)      │  ◄─────────────  │               │  ◄──────────────  │  (Output)     │
└──────────────┘    (RX pad)       └─────────────┘    via PCB         └──────────────┘
                                         │
                                         │ WiFi AP (192.168.4.1)
                                         ▼
                                   ┌─────────────┐
                                   │  Web Browser  │
                                   │  (Dashboard)  │
                                   └─────────────┘
```

### 3.2 Dual-Core Task Distribution

| Core | Task | Priority | Responsibility |
| :--- | :--- | :--- | :--- |
| **Core 1** | `RadioTask` | 5 (High) | Read receiver UART, read module UART, send stick packets, half-duplex switching |
| **Core 0** | `loop()` (default) | 1 (Normal) | WiFi AP management, HTTP server (`WebServer.handleClient()`) |

**Why this matters:** WiFi stack operations (TCP handshake, HTTP responses) can introduce jitter of 10–50 ms. By isolating radio processing on its own core, the relay guarantees continuous CRSF frame forwarding regardless of web traffic.

### 3.3 Radio Task Loop (`RadioTask`)

```
RadioTask (Core 1, infinite loop):
│
├── processReceiverInput()    // Read ELRS RX UART, decode stick channels
├── processJrModule()         // Read TX module UART, handle heartbeats & telemetry
├── if (>20 ms since last TX) → sendStickPacket()   // Keepalive
└── vTaskDelay(1 ms)          // Yield briefly
```

### 3.4 Latency

The relay introduces **no noticeable latency**. The `RadioTask` loop executes every ~1 ms (the `vTaskDelay` yield), and frame forwarding is triggered immediately upon receiving a heartbeat from the TX module. The theoretical added latency is bounded by the loop period (~1 ms) plus the UART transmission time (~650 µs for 26 bytes at 400 kbaud), totaling under 2 ms per hop.

---

## 4. Hardware Interface

### 4.1 GPIO Pin Assignments

| Macro | GPIO | Direction | UART | Function |
| :--- | :--- | :--- | :--- | :--- |
| `RE_TX_PIN` | 32 | Output → Receiver RX pad | `UART_NUM_2` | Send data to ELRS receiver |
| `RE_RX_PIN` | 33 | Input ← Receiver TX pad | `UART_NUM_2` | Receive CRSF frames from ELRS receiver |
| `JR_TX_PIN` | 17 | Output → PCB Data Line | `UART_NUM_1` | Transmit stick packets to TX module |
| `JR_RX_PIN` | 16 | Input ← PCB Data Line | `UART_NUM_1` | Receive telemetry/heartbeats from TX module |
| `BUFFER_EN_PIN` | 25 | Output (Digital) | — | Controls half-duplex direction on the SN74HC125DR buffer |

### 4.2 UART Configuration

| Parameter | Receiver UART (`RE_UART`) | Module UART (`JR_UART`) |
| :--- | :--- | :--- |
| Baud Rate | **420,000** | **400,000** |
| Data Bits | 8 | 8 |
| Parity | None | None |
| Stop Bits | 1 | 1 |
| Flow Control | None | None |
| Clock Source | `UART_SCLK_APB` | `UART_SCLK_APB` |
| Signal Inversion | None | **RXD + TXD inverted** |
| RX Buffer Size | 2048 bytes (`BUF_SIZE * 2`) | 2048 bytes (`BUF_SIZE * 2`) |
| TX Buffer Size | 0 (blocking write) | 0 (blocking write) |

> **Note:** The JR module UART uses inverted signals (`uart_set_line_inverse`) because the CRSF protocol on JR-bay modules uses inverted UART signaling.

### 4.3 Tested Hardware

#### Input Receivers (ELRS RX)

| Receiver | Frequency | Notes |
| :--- | :--- | :--- |
| **HGLRC ELRS 2.4G** | 2.4 GHz | Tested and confirmed working |
| **Radiomaster XR1 Nano ELRS** | 868 MHz | Dual-band capable (868 MHz / 2.4 GHz), tested on 868 MHz |

#### Output TX Module

| Module | Notes |
| :--- | :--- |
| **Emax Aeris Link Nano** | Only tested TX module; requires external power at high output levels |

### 4.4 Power Supply

The entire relay node is powered from a **2S LiPo battery** (7.4V nominal, 5–8.4V range):

```
2S LiPo Battery (7.4V)
├──► Step-Down Regulator ──► ESP32 (5V / 3.3V via onboard regulator)
└──► XT30 Port ──────────► ELRS TX Module (direct battery voltage)
```

- The **ESP32** is powered through a step-down (buck) module that converts the 2S battery voltage to a level suitable for the ESP32's onboard voltage regulator.
- The **TX module** (e.g., Emax Aeris Link Nano) is powered directly from the battery via its XT30 connector, which supports the full 2S voltage range.
- The **ELRS receiver** draws power from the ESP32's 3.3V or 5V output pin (minimal current draw).

> ⚠️ **Do not power high-output TX modules (>250 mW) from USB or the ESP32's voltage pins.** Modules like the Emax Aeris can draw up to 2A at 1W+, which will cause brownouts, resets, and failsafes.

### 4.5 Half-Duplex Switching Protocol

The TX module communicates over a single data line (half-duplex). The ESP32 uses `BUFFER_EN_PIN` (GPIO 25) to control the direction of the **SN74HC125DR** quad tri-state buffer on the custom PCB:

```
sendStickPacket():
  1. BUFFER_EN_PIN → LOW          // Enable buffer output (TX mode)
  2. delayMicroseconds(2)         // Allow settling time
  3. uart_write_bytes(packet, 26) // Transmit 26-byte CRSF frame
  4. delayMicroseconds(680)       // Wait for transmission to complete
  5. BUFFER_EN_PIN → HIGH         // Disable buffer output (RX mode / high-impedance)
```

**Timing breakdown:**
- At 400 kbaud, one byte ≈ 25 µs → 26 bytes ≈ 650 µs
- The 680 µs delay accounts for the full frame transmission + margin
- 2 µs pre-delay ensures the buffer chip has switched before data begins

---

## 5. CRSF Protocol Implementation

### 5.1 CRSF Sync Bytes

| Macro | Value | Origin | Description |
| :--- | :--- | :--- | :--- |
| `CRSF_SYNC_STICK` | `0xEE` | Module/Handset | Sync byte for stick/channel frames sent to a module |
| `CRSF_SYNC_TELEM` | `0xC8` | Flight Controller | Sync byte for telemetry frames |
| `CRSF_SYNC_RADIO` | `0xEA` | Radio/Module | Sync byte for radio-originated frames (heartbeat, config) |

### 5.2 CRSF Frame Types

| Macro | Value | Description |
| :--- | :--- | :--- |
| `CRSF_TYPE_STICKS` | `0x16` | RC Channels Packed — 16 channels × 11 bits |
| `CRSF_TYPE_LINKSTAT` | `0x14` | Link Statistics — RSSI, LQ, SNR |
| `CRSF_TYPE_HEARTBEAT` | `0x3A` | Module heartbeat — triggers stick packet transmission |

### 5.3 Stick Channel Frame Structure (26 bytes)

```
Byte:  [0]    [1]    [2]     [3..24]           [25]
       Sync   Len    Type    Payload (22 B)    CRC8
       0xEE   24     0x16    packed channels   checksum
```

- **Payload:** 16 channels × 11 bits = 176 bits = 22 bytes, packed LSB-first
- **Channel range:** 0–2047 (11-bit), center at 992
- **CRC8 polynomial:** `0xD5` (computed over bytes `[2..24]`, i.e., type + payload)

### 5.4 Channel Packing Algorithm (`packCrsfChannels`)

Each of the 16 channels is an 11-bit unsigned integer. They are packed sequentially into 22 bytes in LSB-first bit order:

```
Channel 0: bits [0..10]   → payload bits 0–10
Channel 1: bits [11..21]  → payload bits 11–21
Channel 2: bits [22..32]  → payload bits 22–32  (throttle limiting applied here)
...
Channel 15: bits [165..175]
```

**Throttle Limiting (Channel 3 / index 2):**

When `throttleScale < 100`, Channel 3 is scaled:

```
zero_point = 172          // CRSF value representing 0% throttle
range = channel_value - zero_point
scaled_range = (range × throttleScale) / 100
output = zero_point + scaled_range
```

This preserves the zero-throttle position while reducing the maximum proportionally.

### 5.5 CRC-8 Calculation

```cpp
uint8_t calc_crc8(const uint8_t* payload, int length) {
    uint8_t crc = 0;
    for (int i = 0; i < length; i++) {
        crc ^= payload[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc = crc << 1;
        }
    }
    return crc & 0xFF;
}
```

- **Polynomial:** `0xD5` (standard CRSF CRC-8)
- **Init:** `0x00`
- **Input:** Frame bytes from type field through end of payload (bytes `[2..24]`)

### 5.6 Link Statistics Parsing

When a `CRSF_TYPE_LINKSTAT` (`0x14`) frame is received from the TX module:

```
Payload byte offsets (from byte [i+3]):
  p[0] = Uplink RSSI (Ant. 1)    → cast to int8_t → disp_rssi
  p[2] = Link Quality (0–100%)   → disp_lq
  p[3] = SNR                     → cast to int8_t → disp_snr
```

---

## 6. Firmware Module Reference

### 6.1 Functions

| Function | Description | Runs On |
| :--- | :--- | :--- |
| `setup()` | Initializes GPIOs, both UARTs, channels to center (992), starts `RadioTask` on Core 1, starts WiFi AP and HTTP server | Core 0 |
| `loop()` | Calls `server.handleClient()` every 2 ms | Core 0 |
| `RadioTask(void*)` | Infinite loop: process receiver, process module, keepalive TX | Core 1 |
| `processReceiverInput()` | Reads `RE_UART`, scans for CRSF stick frames, unpacks 16 channels | Core 1 |
| `processJrModule()` | Reads `JR_UART`, handles heartbeats (triggers TX) and link stats | Core 1 |
| `sendStickPacket()` | Builds 26-byte CRSF frame, performs half-duplex TX via SN74HC125DR | Core 1 |
| `packCrsfChannels(payload, channels)` | Bit-packs 16 × 11-bit channels into 22 bytes, applies throttle limit | Core 1 |
| `calc_crc8(payload, length)` | Computes CRC-8 with polynomial `0xD5` | Core 1 |
| `handleRoot()` | Serves the embedded HTML dashboard | Core 0 |
| `handleData()` | Returns JSON telemetry (`/data` endpoint) | Core 0 |
| `handleLimit()` | Accepts throttle scale parameter (`/set_limit?val=N`) | Core 0 |

### 6.2 Global Variables

| Variable | Type | Volatile | Description |
| :--- | :--- | :--- | :--- |
| `currentChannels[16]` | `uint16_t[]` | ✅ | Latest decoded RC channel values (0–2047) |
| `disp_rssi` | `int` | ✅ | Current RSSI in dBm (default: -130) |
| `disp_lq` | `int` | ✅ | Current Link Quality percentage (0–100) |
| `disp_snr` | `int` | ✅ | Current Signal-to-Noise Ratio |
| `throttleScale` | `int` | ✅ | Throttle limit percentage (0–100, default: 100) |
| `lastPacketTime` | `unsigned long` | — | Timestamp of last sent stick packet |
| `lastRcInputTime` | `unsigned long` | — | Timestamp of last received RC input |
| `lastTelemTime` | `unsigned long` | — | Timestamp of last received telemetry |
| `RadioTaskHandle` | `TaskHandle_t` | — | FreeRTOS task handle for the radio task |

### 6.3 Timing Constants

| Event | Threshold | Purpose |
| :--- | :--- | :--- |
| Keepalive TX | 20 ms | If no heartbeat triggers TX within 20 ms, send a stick packet anyway |
| RC Input timeout | 100 ms | If no stick frame received in 100 ms → status: `RC LOST` |
| Module packet timeout | 100 ms | If no packet sent to module in 100 ms → status: `MODULE OFF` |
| Telemetry timeout | 1000 ms | If no link stats received in 1 s → status: `DRONE LOST` |
| Dashboard poll interval | 100 ms | Web frontend fetches `/data` every 100 ms |
| Radio task yield | 1 ms | `vTaskDelay(1 / portTICK_PERIOD_MS)` — minimal cooperative yield |

---

## 7. Web Dashboard & HTTP API

### 7.1 WiFi Access Point

| Parameter | Value |
| :--- | :--- |
| SSID | `Comm_Relay_Manager` |
| Password | *(none — open network)* |
| IP Address | `192.168.4.1` |
| Mode | SoftAP |

### 7.2 HTTP Endpoints

| Method | Path | Handler | Description |
| :--- | :--- | :--- | :--- |
| GET | `/` | `handleRoot()` | Serves the full HTML/CSS/JS dashboard (stored in `PROGMEM`) |
| GET | `/data` | `handleData()` | Returns live telemetry as JSON |
| GET | `/set_limit?val=N` | `handleLimit()` | Sets throttle scale to `N` (0–100) |

### 7.3 `/data` JSON Response Schema

```json
{
  "rssi": -65,
  "lq": 98,
  "status": 3,
  "channels": [992, 992, 172, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992]
}
```

| Field | Type | Range | Description |
| :--- | :--- | :--- | :--- |
| `rssi` | int | -130 to 0 | Uplink RSSI in dBm |
| `lq` | int | 0–100 | Link quality percentage |
| `status` | int | 0–3 | System status code (see below) |
| `channels` | int[] | 0–2047 each | 16 RC channel values |

### 7.4 Status Codes

| Code | Label | Color | Condition |
| :--- | :--- | :--- | :--- |
| 0 | `RC LOST` | 🔴 Red | No RC input from receiver for > 100 ms |
| 1 | `MODULE OFF` | 🟡 Yellow | RC input OK, but no packet sent to module for > 100 ms |
| 2 | `DRONE LOST` | 🔵 Blue | RC + module OK, but no telemetry for > 1000 ms |
| 3 | `ONLINE` | 🟢 Green | All systems nominal |

**Status determination logic (evaluated in order):**
```
if (now - lastRcInputTime >= 100 ms)    → status = 0 (RC LOST)
else if (now - lastPacketTime >= 100 ms) → status = 1 (MODULE OFF)
else if (now - lastTelemTime >= 1000 ms) → status = 2 (DRONE LOST)
else                                     → status = 3 (ONLINE)
```

### 7.5 Dashboard Features

The web interface is a single-page application embedded in `PROGMEM` (~3.5 KB) with two tabs:

**Dashboard Tab:**
- **System Status Badge** — color-coded live status indicator
- **Signal History Graph** — scrolling RSSI line chart (100 data points, canvas-based)
- **Stats Cards** — Link Quality (%) and RSSI (dBm)
- **Channel Monitor** — 16 horizontal bar indicators showing real-time channel positions

**Limits Tab:**
- **Throttle Output Limiter** — range slider (0–100%) that scales Channel 3 output
- Updates are sent immediately via `fetch('/set_limit?val=N')`

---

## 8. Half-Duplex PCB Design

### 8.1 Purpose

The ESP32 uses full-duplex UARTs (separate TX and RX lines), while ELRS TX modules typically use a single half-duplex data line for both sending and receiving. The custom PCB bridges this gap using an **SN74HC125DR** quad tri-state buffer.

### 8.2 Buffer IC: SN74HC125DR

| Parameter | Value |
| :--- | :--- |
| Manufacturer | Texas Instruments |
| Package | SOIC-14 |
| Type | Quad Bus Buffer Gate with 3-State Outputs |
| Supply Voltage | 2V – 6V |
| Propagation Delay | ~7 ns (typ. at 4.5V) |
| Output Enable | Active LOW (directly driven by `BUFFER_EN_PIN` / GPIO 25) |
| Gates Used | **1 of 4** (single gate for TX direction control) |

The SN74HC125DR contains four independent tri-state buffer gates. Only **one gate** is used in this design — it controls the TX direction from the ESP32 to the module data line. The remaining three gates are unused. Each gate has an output enable pin (`OE`, active low). When `OE` is LOW, the buffer actively drives its output; when `OE` is HIGH, the output enters a high-impedance state, effectively disconnecting it from the data line.

### 8.3 KiCad Project Files

| File | Description |
| :--- | :--- |
| `PCB/halfduplextranslator.kicad_sch` | Circuit schematic |
| `PCB/halfduplextranslator.kicad_pcb` | PCB layout |
| `PCB/halfduplextranslator.kicad_pro` | KiCad project metadata |

### 8.4 Operating Principle

```
ESP32 GPIO 25 (BUFFER_EN):
  LOW  → Buffer output enabled (TX mode: ESP32 → Module)
  HIGH → Buffer output high-impedance (RX mode: Module → ESP32)

                          ┌────────────────────┐
ESP32 GPIO 17 (JR_TX) ──►│ SN74HC125DR Gate 1  │──► Single Data Line ──► TX Module
                          │ OE = GPIO 25        │
                          └────────────────────┘
ESP32 GPIO 16 (JR_RX) ◄───────────────────────────── Single Data Line ◄── TX Module
```

When transmitting:
1. `BUFFER_EN` goes LOW → SN74HC125DR gate 1 enables, passes ESP32 TX to the data line
2. Data is clocked out at 400 kbaud (inverted)
3. After transmission completes (~680 µs for 26 bytes), `BUFFER_EN` goes HIGH
4. The buffer output enters high-impedance, allowing telemetry from the module to reach ESP32 RX directly

### 8.5 When the PCB Is Not Needed

If using an ELRS **receiver flashed as a TX module** (instead of a dedicated TX module), the PCB is unnecessary because receivers have separate TX and RX UART pads — full-duplex communication is native.

---

## 9. Build & Flash Instructions

### 9.1 Prerequisites

- **Arduino IDE** (1.8+ or 2.x) with **ESP32 board package** installed
- Board selection: `ESP32 Dev Module` (or `DOIT ESP32 DEVKIT V1`)
- **No external libraries required** — uses only built-in ESP32 Arduino libraries:
  - `driver/uart.h` (ESP-IDF UART driver)
  - `WiFi.h` (ESP32 WiFi)
  - `WebServer.h` (ESP32 HTTP server)

### 9.2 ELRS Receiver Configuration

Both the input ELRS receiver and the drone-side ELRS receiver must be configured with matching **bind phrases** in the ELRS Configurator. This project uses bind phrases (not traditional binding buttons) to pair receivers with their respective transmitters.

> **Tip:** Make sure the bind phrase on the relay's input receiver matches your handheld transmitter, and the bind phrase on the drone's receiver matches the relay's TX module output.

### 9.3 Upload Settings

| Setting | Value |
| :--- | :--- |
| Board | ESP32 Dev Module |
| Upload Speed | 921600 |
| CPU Frequency | 240 MHz (default) |
| Flash Frequency | 80 MHz |
| Flash Size | 4 MB (default) |
| Partition Scheme | Default 4MB with spiffs |

### 9.4 Steps

1. Open `code/esp32code.ino` in Arduino IDE
2. Select the correct board and COM port
3. Click **Upload**
4. After upload, the ESP32 will automatically:
   - Initialize both UARTs
   - Start the radio task on Core 1
   - Broadcast the `Comm_Relay_Manager` WiFi network
5. Connect to the WiFi and navigate to `http://192.168.4.1`

---

## 10. Configuration Reference

All configuration is done via `#define` macros and constants at the top of `esp32code.ino`:

### 10.1 WiFi

```cpp
const char* ssid = "Comm_Relay_Manager";  // Change to customize network name
const char* password = "";                 // Set a password for secured access
```

### 10.2 Pin Mapping

```cpp
#define BUFFER_EN_PIN 25    // Half-duplex direction control (SN74HC125DR OE, Gate 1)
#define JR_TX_PIN     17    // TX to module (via PCB)
#define JR_RX_PIN     16    // RX from module (via PCB)
#define RE_TX_PIN     32    // TX to receiver
#define RE_RX_PIN     33    // RX from receiver
```

### 10.3 Protocol Tuning

```cpp
#define BUF_SIZE  1024      // UART buffer allocation (actual RX buffer = BUF_SIZE * 2)
#define CRC_POLY  0xD5      // CRC-8 polynomial (CRSF standard — do not change)
```

### 10.4 Baud Rates

| UART | Baud Rate | Notes |
| :--- | :--- | :--- |
| Receiver (`RE_UART`) | 420,000 | Standard ELRS CRSF baud rate |
| Module (`JR_UART`) | 400,000 | Standard CRSF JR-bay baud rate (inverted) |
| Debug Serial | 115,200 | USB serial (currently unused for output) |

---

## 11. Troubleshooting

### Status: RC LOST

| Possible Cause | Solution |
| :--- | :--- |
| Receiver not powered | Check 3.3V/5V supply to the ELRS receiver |
| Wrong wiring | Verify GPIO 33 ← Receiver TX, GPIO 32 → Receiver RX |
| Receiver not bound | Verify the bind phrase matches between receiver and transmitter |
| Baud rate mismatch | Ensure the receiver is configured for CRSF output at 420 kbaud |

### Status: MODULE OFF

| Possible Cause | Solution |
| :--- | :--- |
| TX module not powered | Provide adequate external power via 2S LiPo battery (see Section 4.4) |
| PCB not connected | Check PCB wiring to GPIO 16, 17, and 25 |
| SN74HC125DR failure | Verify the buffer IC solder joints and supply voltage |
| Step-down regulator issue | Confirm the buck module is outputting correct voltage to the ESP32 |

### Status: DRONE LOST

| Possible Cause | Solution |
| :--- | :--- |
| Drone out of range | Move the drone closer or increase TX power |
| Drone receiver not bound | Verify bind phrase matches between drone receiver and relay TX module |
| Telemetry disabled | Enable telemetry in ELRS Lua script or configurator |

### Brownouts / Resets

| Possible Cause | Solution |
| :--- | :--- |
| High-power module on USB | **Never** power modules >250 mW from USB. Use a 2S battery (5–21V) via XT30 |
| Insufficient current | Ensure your power source can deliver ≥2A for 1W+ modules |
| Step-down overloaded | Make sure the buck converter is rated for the combined ESP32 + peripherals draw |

### Dashboard Not Loading

| Possible Cause | Solution |
| :--- | :--- |
| Not connected to WiFi | Join `Comm_Relay_Manager` (open, no password) |
| Wrong IP | Navigate to `http://192.168.4.1` |
| Browser cache | Hard refresh (`Ctrl+Shift+R`) or try incognito mode |

---

*This document was generated from the source code at commit `ac6ba01` of the `main` branch.*