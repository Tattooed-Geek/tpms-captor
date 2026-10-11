# TPMS Captor

A standalone ESP32 reader for **generic Bluetooth TPMS sensors**, developed
and tested with a 4-sensor external BLE TPMS kit (marketed as *"aqxreight
Tire Pressure Monitor Bluetooth 5.0"*, companion app *"BLE TPMS"*).

The ESP32 **passively listens** to the sensors' BLE advertisements — no
pairing, no phone, no cloud — and continuously displays **pressure /
temperature / battery** on a **LCD1602**, with audible (buzzer) and visual
alarms.

> The BLE protocol was reconstructed by static analysis of the companion app
> for **interoperability** with owned hardware. Details and formulas:
> [`docs/PROTOCOL.md`](docs/PROTOCOL.md). Passive listening does not wear the
> sensor batteries (rated 3–5 years).

## Features

- All 4 tires at a glance: front-1 / rear-1 on the left, front-2 / rear-2 on the right
- Center-top: highest temperature — center-bottom: lowest battery voltage
- Blinking **`ALERT!`** + buzzer when any alarm threshold is crossed
- Pressure unit selectable: **psi** (default), kPa or bar via `PRES_UNIT`
- Thresholds default to the original app values (200–300 kPa, 65 °C, 2.0 V)
- `!` before a value = threshold crossed — `--` = sensor silent for > 10 min
- `[SNIFF]` serial mode logs every unknown BLE device for protocol validation
- Fully offline: no Wi-Fi, no internet, no phone required

## Screen layout (LCD1602, 16x2)

```
 +----------------+
 | 32.0     32°C  | 32.1    AV1 | max temp | AV2
 | 33.1     2.9V  | 32.0    AR1 | battery  | AR2
 +----------------+
```

## Hardware

| Part | Notes |
|---|---|
| ESP32 DevKit (WROOM-32) | any ESP32 with BLE works |
| LCD1602 | parallel (16-pin) **or** I2C backpack (PCF8574) |
| 10k potentiometer | contrast (parallel wiring only) |
| Active buzzer | optional — set `BUZZER_PIN -1` to disable |
| Second ESP32 | for the **sensor simulator** (optional but recommended) |

Alternative display: `firmware/tpms_display` drives a MAX7219 LED matrix
(2 blocks = 16x8, scrolling text) instead of the LCD1602.

## Wiring

### LCD1602 — parallel (default)

| LCD1602 pin | ESP32 |
|---|---|
| VSS (1) | GND |
| VDD (2) | 5V (VIN) |
| V0 (3) | 10k potentiometer: 5V / **wiper → V0** / GND |
| RS (4) | GPIO22 |
| RW (5) | GND |
| E (6) | GPIO21 |
| D0-D3 (7-10) | not connected |
| D4 (11) | GPIO19 |
| D5 (12) | GPIO18 |
| D6 (13) | GPIO17 |
| D7 (14) | GPIO16 |
| A (15) | 5V (through 220 Ω if the module has no built-in resistor) |
| K (16) | GND |

### LCD1602 — I2C backpack (PCF8574)

SDA → GPIO21, SCL → GPIO22, VCC → 5V, GND → GND.
Build with `-DLCD_I2C=1`. Default address `0x27`; if the screen stays dark,
try `0x3F` (`LCD_I2C_ADDR`).

### Buzzer / LED matrix

Buzzer: `+` → GPIO25, `−` → GND.
Matrix variant: VCC → 5V, GND → GND, DIN → GPIO23, CS → GPIO5, CLK → GPIO18.

## Quick start

```bash
git clone https://github.com/Tattooed-Geek/tpms-captor.git
cd tpms-captor
```

### Option A — prebuilt binaries (no toolchain needed)

Grab a `.bin` from the **Releases** page and flash it with esptool:

```bash
pip install esptool

# LCD1602 with parallel wiring (default):
esptool.py --chip esp32 write_flash 0x0 tpms_lcd_flash0x0.bin
# LCD1602 with I2C backpack:
esptool.py --chip esp32 write_flash 0x0 tpms_lcd_i2c_flash0x0.bin
# MAX7219 matrix variant:
esptool.py --chip esp32 write_flash 0x0 tpms_display_flash0x0.bin
# Sensor simulator (second ESP32):
esptool.py --chip esp32 write_flash 0x0 tpms_emitter_flash0x0.bin
```

### Option B — Arduino IDE

1. Boards Manager → add
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   and install the **esp32** core.
2. Library Manager → install `LiquidCrystal` and `LiquidCrystal I2C`
   (plus `MD_MAX72xx` and `MD_Parola` for the matrix variant).
3. Open the sketch (e.g. `firmware/tpms_lcd/tpms_lcd.ino`), board
   *ESP32 Dev Module*, Upload.

### Option C — arduino-cli

```bash
arduino-cli core install esp32:esp32
arduino-cli lib install LiquidCrystal "LiquidCrystal I2C" MD_MAX72xx MD_Parola
cd firmware/tpms_lcd
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/ttyUSB0 .
```

## Test without sensors (2 ESP32)

1. Flash `tpms_lcd*` (or `tpms_display`) on the ESP32 wired to the screen.
2. Flash `tpms_emitter` on a second ESP32 (USB power is enough).
3. Within a minute the screen shows 4 simulated tires (≈ 32.0 / 31.6 / 32.5
   psi) and tire 4 (180 kPa ≈ 26.1 psi) triggers `ALERT!` + beeps.

## Validation with the real sensors

Open the serial monitor at **115200 baud**:

- `[SNIFF] <mac> name=... rssi=... hex=...` — every unknown device, once per address.
- `[TPMS] id=... slot=0x8x 220.4kPa 21.3C 2.9V` — a decoded sensor frame.
- No `[TPMS]` line? Copy the sensors' `[SNIFF]` lines into an issue — every
  field offset is a tunable constant at the top of the sketch (`OFF_MARKER`,
  `OFF_SLOT`, `OFF_ID`, `OFF_PRESS`, `OFF_TEMP`, `OFF_BATT`).
- Compare the decoded 6-character `id` with the code engraved on each sensor.

## Unit tests (native — no hardware needed)

```bash
g++ -O0 -o t firmware/test/tpms_parse_test.cpp && ./t   # decoding == original app formulas
g++ -O0 -o t firmware/test/lcd_render_test.cpp && ./t   # screen composition (psi, alarms, stale, degree glyph)
```

## Repository structure

```
firmware/
  tpms_lcd/       ESP32 + LCD1602 (main display)
  tpms_display/   ESP32 + MAX7219 LED matrix (variant)
  tpms_emitter/   sensor simulator (second ESP32)
  test/           native unit tests
docs/
  PROTOCOL.md     reconstructed BLE protocol (byte offsets, formulas, pairing)
```

## License

MIT — see [LICENSE](LICENSE).

*Interoperability research performed on a public binary and owned hardware,
without circumventing any protection. All trademarks belong to their
respective owners.*
