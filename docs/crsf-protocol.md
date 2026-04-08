# CRSF Protocol

## Sync Bytes

| Macro | Value | Source |
| --- | --- | --- |
| `CRSF_SYNC_STICK` | `0xEE` | Stick/channel frames to module |
| `CRSF_SYNC_TELEM` | `0xC8` | Telemetry frames |
| `CRSF_SYNC_RADIO` | `0xEA` | Radio/module-originated control frames |

## Frame Types Used

| Macro | Value | Meaning |
| --- | --- | --- |
| `CRSF_TYPE_STICKS` | `0x16` | Packed 16-channel RC data |
| `CRSF_TYPE_LINKSTAT` | `0x14` | Link statistics payload |
| `CRSF_TYPE_HEARTBEAT` | `0x3A` | Module heartbeat trigger |

## Stick Frame Structure

A forwarded stick frame has 26 bytes total:

- Byte 0: sync (`0xEE`)
- Byte 1: frame length (`24`)
- Byte 2: frame type (`0x16`)
- Bytes 3-24: 22-byte packed channel payload
- Byte 25: CRC-8

## Channel Packing

- 16 channels, each 11-bit unsigned.
- Total bits: 16 x 11 = 176 bits = 22 bytes.
- Packing order: LSB-first bitstream.

### Throttle Limiter

If `throttleScale < 100`, channel 3 (index 2) is scaled around CRSF zero-throttle reference (`172`) before packing.

## CRC-8 Parameters

- Polynomial: `0xD5`
- Init: `0x00`
- Input range: type + payload (frame bytes 2 through 24)

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

## Telemetry Parsing

For `CRSF_TYPE_LINKSTAT` frames, payload offsets map to display values:

- `p[0]`: uplink RSSI (signed)
- `p[2]`: link quality (0 to 100)
- `p[3]`: SNR (signed)
