# ESP32-CRSF-Relay
![Platform](https://img.shields.io/badge/platform-ESP32-green)
![Language](https://img.shields.io/badge/language-C%2B%2B-orange)
![License](https://img.shields.io/badge/license-MIT-blue.svg)

A high-performance **CRSF Relay Node** that extends the range of your radio link by forwarding CRSF packets from an ELRS Receiver to a high-power TX Module.

## 🚀 Key Capabilities

* **CRSF frames forwarding** Forwards frames from receiver to transmitter to further range of radio link
* **Dual-Core Architecture:** Radio logic runs exclusively on **Core 1** (High Priority) to ensure 0% packet loss, while WiFi runs on **Core 0**.
* **WiFi Manager:** Integrated web dashboard at `192.168.4.1` (No app required).
* **Live Telemetry:** Real-time scrolling graph of RSSI and Link Quality to visualize signal health and live channel values streaming.
* **Safety Limiter:** Adjustable throttle scaling (0-100%) via the web interface—perfect for training beginners without reprogramming the radio.
* **Smart Status:** Instantly diagnoses connection issues (e.g., "RC LOST", "MODULE OFF", "DRONE LOST").

## 🛠️ Hardware Required

* **Microcontroller:** ESP32 DevKit V1 (or compatible WROOM-32 board).
* **Input Receiver:** Any ELRS Receiver (EP1, EP2, RP1) configured to output CRSF.
* **Output Module:** A ELRS-compatible TX Module (I used **Emax Aeris Link Nano**), or an ELRS receiver flashed as TX.
* **Custom PCB:** Required to convert the ESP32's Full-Duplex UART to the Module's Half-Duplex signal. (Not needed if you are using RX as TX, since it has separate UART RX and TX pins)

# Wiring
I am using an ELRS receiver connected to a UART of ESP32, another ESPs UART is connected to a custom PCB, and from PCB to transmitter one half-duplex line, as seen on this diagram:

<img width="682" height="112" alt="Untitled Diagram drawio" src="https://github.com/user-attachments/assets/7a7d4cde-fe29-4748-855f-7ae103f23819" />

| Component | ESP32 Pin | Function | Notes |
| :--- | :--- | :--- | :--- |
| **Receiver TX** | GPIO 33 | Input | Connect to RX pad of ELRS Receiver |
| **Receiver RX** | GPIO 32 | Output | Connect to TX pad of ELRS Receiver |
| **Module RX** | GPIO 17 | TX Data | Connects to PCB Data Line |
| **Module TX** | GPIO 16 | RX Data | Connects to PCB Data Line |
| **Buffer EN** | GPIO 25 | Direction | Controls Half-Duplex switching |

## ⚡ Important Power Warning

**⚠️ DO NOT power high-output modules directly from the ESP32.**

If you are using high-power modules like the **Emax Aeris**, they can draw significant current (up to 2A at 1W+).
* **Low Power (10mW - 250mW):** May work on USB power, but not recommended.
* **High Power (500mW - 1000mW+):** You **MUST** use an external 2S Battery (5-21V) connected to the module's XT30 port[cite: 47, 94]. Failure to do so will cause brownouts and failsafes.

## 📱 Usage (Web Dashboard)

Once powered on, the ESP32 will broadcast a WiFi network.

1.  **Connect:** Join the WiFi network **`Comm_Relay_Manager`** (No password).
2.  **Dashboard:** Open your browser and navigate to **`http://192.168.4.1`**.
3.  **Controls:**
    * **Dashboard Tab:** View live stats, signal graphs, and channel movements.
    * **Limits Tab:** Adjust the "Max Throttle" slider to limit the output of Channel 3 for training purposes.

## 📂 PCB Design
The `PCB` folder contains KiCad project files for the **Half-Duplex Translator**.
* **Why is this needed?** ESP32 UARTs use two separate wires (TX and RX). Most RC Modules use a single shared wire for both sending commands and receiving telemetry. This PCB uses a buffer chip to merge these signals safely.

## 📚 Technical Docs (MkDocs)

A fully structured documentation site is now available in the `docs` folder and configured by `mkdocs.yml`.

### Run locally

```powershell
pip install -r requirements-docs.txt
mkdocs serve
```

Then open: `http://127.0.0.1:8000`

### Build static site

```powershell
mkdocs build
```
