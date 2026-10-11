# BLE protocol of the TPMS sensors ("BLE TPMS family")

Reconstructed by **static analysis** of the Android app *"BLE TPMS"*
(`com.po.tyrecheck` v2.28.10) for **interoperability** with owned hardware
(a 4-sensor external BLE TPMS kit).

- The formulas below reproduce **exactly** the app's code (verified by a
  native test: `firmware/test/tpms_parse_test.cpp`).
- The byte offsets still need **confirmation against real sensors**: every
  sketch embeds a `[SNIFF]` mode (serial monitor) and all offsets are
  tunable constants at the top of each sketch.

## 1. Direct mode — advertisements (no connection)

The sensors periodically broadcast BLE advertisement packets (they wake up
on pressure change, then about once per minute; response within ~5 s after
applying pressure). The reader listens **passively**.

Positions are **raw bytes** inside the advertisement payload; in parentheses,
the equivalent positions in the hexadecimal string that the Android app
parses (byte × 2):

| Field | Bytes | Hex (app) | Format |
|---|---|---|---|
| Family marker | 9–10 | 18–22 | `00 01`, fixed |
| Position | 11 | 22–24 | `0x80` FL, `0x81` FR, `0x82` RL, `0x83` RR |
| Sensor ID | 14–16 | 28–34 | 3 bytes — the 6-character code engraved on the sensor / QR card |
| Pressure | 17–20 | 34–42 | 32-bit **little-endian** int ÷ 100,000 → kPa |
| Temperature | 21–24 | 42–50 | 32-bit LE int ÷ 100 → °C |
| Battery | 25 | 50–52 | one byte ÷ 10 → V |

Minimum useful length: 26 bytes (packets may be longer — extra manufacturer
data is ignored).

Display conversions: `psi = kPa / 6.894757` — `bar = kPa / 100`.

Suggested plausibility filters: 0–1500 kPa, −60…150 °C, 0–5 V.
(The app does not use any CRC; frame validity is ensured by the sensor-ID
match and value plausibility.)

## 2. Pairing

- **QR / "One-Key Binding" card**: contains the sensor code (6 hex
  characters, 0-9 a-f). The QR of an optional *relay* box contains
  `TPMSR_XXXXXX` (12 characters).
- **Manual entry** of the 6-character code (app UI).
- **Automatic search**: while a pressure change is happening
  (inflating/deflating), the app binds the first unknown sensor heard.

**Sensor identity.** Every sensor carries a unique 3-byte ID — the 6
hex-character code engraved on it / QR card — present in every
advertisement. A fresh sensor's *position* byte may just be a default value
(sensors not yet bound to positions can all report the same byte): the
reference firmware detects duplicates and falls back to ordering by
ascending sensor ID (deterministic layout), with an optional pinned
ID→position table in the sketch for guaranteed corners.

## 3. Relay mode (optional — statically reconstructed, untested)

Based on code observations; only relevant if a relay box is used.

- Relay BLE name prefix: `TPMSR_` (scan filter).
- GATT service `0000fff0-0000-1000-8000-00805f9b34fb`:
  - `fff1`..`fff4`: write — per-tire command:
    `[sensor ID 3 bytes, reversed order] + CA EA 80|81|82|83`;
  - `fff5`: write — `01` (to confirm the binding);
  - `fff6`: notifications (enable via the CCCD descriptor `0x2902`).
- Measurements keep arriving as advertisements (section 1).

## 4. Original app defaults (reference)

- Thresholds: low pressure 2.0 bar / high 3.0 bar; max temperature 65 °C;
  min battery 2.0 V; units kPa / bar / psi, °C / °F.
- Signal loss: 10 minutes without data → position shown as "no data".
- Alarms: high/low pressure, fast and slow leaks, high temperature,
  no signal, low battery (voice + vibration in the app).

## 5. Validation against real sensors

1. Flash a sketch from this repository, open the serial monitor (115200 baud).
2. Find each sensor's `[SNIFF]` line (the 6-character code is engraved on
   the sensor): it contains the complete raw frame.
3. Check that the `00 01` marker sits at bytes 9–10 and that `[TPMS]` lines
   appear with plausible values.
4. On mismatch: adjust the `OFF_*` constants at the top of the sketch
   (each field offset is independent).
