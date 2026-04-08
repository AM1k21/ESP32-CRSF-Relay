# PCB Design

## Purpose

ESP32 UART is full-duplex (separate TX and RX), while many TX modules use a single shared half-duplex line. The custom board bridges this mismatch.

## Buffer Device

| Parameter | Value |
| --- | --- |
| Part | SN74HC125DR |
| Type | Quad tri-state buffer |
| Package | SOIC-14 |
| VCC range | 2 V to 6 V |
| OE behavior | Active low |

Only one gate is required in this design. Remaining gates are unused.

## Direction Logic

- `BUFFER_EN_PIN` low: gate enabled, ESP32 drives data line.
- `BUFFER_EN_PIN` high: gate disabled, line returns to receive mode.

## Design Files

- `PCB/halfduplextranslator.kicad_sch`
- `PCB/halfduplextranslator.kicad_pcb`
- `PCB/halfduplextranslator.kicad_pro`

## When This PCB Is Not Required

If you use an ELRS receiver flashed as TX and it exposes separate RX/TX pads for full-duplex wiring, you can bypass this translator board.
