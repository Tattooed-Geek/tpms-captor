/*
 * ============================================================================
 *  TPMS LCD Display — generic BLE TPMS sensors ("BLE TPMS" family)
 * ============================================================================
 *  Passively listens to the TPMS sensors' BLE advertisements (no connection,
 *  no pairing) and shows everything on a LCD1602 (16x2):
 *
 *      +----------------+
 *      | 32.0     32°C  | 32.1       row 1 : FL  |  max temp  |  FR
 *      | 33.1     2.9V  | 32.0       row 2 : RL  |  battery   |  RR
 *      +----------------+
 *
 *  Pressures shown in psi (PRES_UNIT : 0=kPa, 1=bar, 2=psi).
 *
 *  - Left (top/bottom)  : FRONT-1 (front left) / REAR-1 (rear left)
 *  - Right (top/bottom) : FRONT-2 / REAR-2
 *  - Center top         : highest temperature of the 4 tires
 *  - Center bottom      : lowest battery voltage — or blinking "ALERT!"
 *    when a threshold is crossed (alarm + buzzer).
 *  - "!" before a pressure = threshold crossed ; "--" = sensor silent
 *    for > 10 min.
 *
 *  Protocol decoded by RE of the official companion app (see
 *  docs/PROTOCOL.md). Offsets in RAW BYTES (the Android app reads a hex
 *  string, so its positions are twice these) :
 *    [9..10]  00 01  family marker   [11] slot 0x80..0x83   [14..16] ID
 *    [17..20] pressure LE32 /100000 -> kPa
 *    [21..24] temperature LE32 /100 -> C   [25] battery /10 -> V
 *
 *  Target : ESP32 DevKit (WROOM-32). Two possible wirings :
 *    - parallel LCD1602 (16 pins, stock part) : default mode ;
 *    - LCD1602 with I2C backpack (PCF8574) : build with -DLCD_I2C=1
 *      (needs the "LiquidCrystal I2C" library).
 * ============================================================================
 */

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

/* ------------------------------------------------------------- LCD : 2 modes */
//#define LCD_I2C 1              // uncomment (or -DLCD_I2C=1) for I2C backpack

#ifdef LCD_I2C
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
  #define LCD_I2C_ADDR 0x27      // 0x27 or 0x3F depending on the backpack
  LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2);
  #define LCD_INIT()    do { lcd.init(); lcd.backlight(); } while (0)
#else
  #include <LiquidCrystal.h>
  // parallel wiring (LCD pins 4,6,11,12,13,14) — non-strapping GPIOs
  #define LCD_RS 22
  #define LCD_EN 21
  #define LCD_D4 19
  #define LCD_D5 18
  #define LCD_D6 17
  #define LCD_D7 16
  LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
  #define LCD_INIT()    do { lcd.begin(16, 2); } while (0)
#endif

/* ---------------------------------------------------------------- HW config */
#define BUZZER_PIN  25           // active buzzer (-1 to disable)

/* -------------------------------------------------- TPMS protocol config */
#define OFF_MARKER  9            /* RAW BYTE positions (see header) */
#define OFF_SLOT    11
#define OFF_ID      14
#define OFF_PRESS   17
#define OFF_TEMP    21
#define OFF_BATT    25
#define MIN_LEN     26

/* alarm thresholds — same default values as the official app */
#define TH_LOW_KPA   200.0f
#define TH_HIGH_KPA  300.0f
#define TH_TEMP_C     65.0f
#define TH_BATT_V      2.0f
#define STALE_MS    600000UL     // 10 min -> "--"

/* pressure display unit : 0=kPa (integer)  1=bar (2 dec)  2=psi (1 dec) */
#define PRES_UNIT 2

#define REFRESH_MS    500UL      // LCD refresh + blink period

/* ------------------------------------------------------------- local state */
#define MAX_TIRES 4

struct Tire {
  bool     used     = false;
  String   id       = "";
  uint8_t  slot     = 0xFF;
  float    kPa      = 0.0f;
  float    degC     = 0.0f;
  float    volts    = 0.0f;
  uint32_t lastSeen = 0;
  int8_t   rssi     = 0;
};

static Tire     tires[MAX_TIRES];
static uint32_t lastScanRestart = 0;
static uint32_t lastRefresh = 0;
static uint32_t lastBeep = 0;
static uint32_t beepOffAt = 0;
static bool     alarmActive = false;
static char     line1[17]     = "";
static char     line2[17]     = "";
static char     lastLine1[17] = "\xff";
static char     lastLine2[17] = "\xff";
static BLEScan* scan = nullptr;

/* Custom CGRAM character 0 : degree symbol */
static uint8_t degGlyph[8] = { B00110, B01001, B00110, B00000, B00000, B00000, B00000, B00000 };

#define DEG_CH ((char)0)

/* -------------------------------------------------------------- utilities */
static uint8_t getByte(const uint8_t* p, size_t n, int pos) {
  return ((size_t)pos < n) ? p[pos] : 0;
}
static uint32_t u32le(const uint8_t* p, size_t n, int pos) {
  if ((size_t)pos + 3 >= n) return 0;
  return (uint32_t)p[pos] | ((uint32_t)p[pos + 1] << 8)
       | ((uint32_t)p[pos + 2] << 16) | ((uint32_t)p[pos + 3] << 24);
}

static bool tireFresh(const Tire& t) {
  return t.used && (millis() - t.lastSeen) <= STALE_MS;
}
static bool tireAlarm(const Tire& t) {
  return tireFresh(t) && (t.kPa < TH_LOW_KPA || t.kPa > TH_HIGH_KPA ||
                          t.degC > TH_TEMP_C || t.volts < TH_BATT_V);
}

/* copy `src` (explicit length — may contain a NUL byte, the degree glyph)
 * into dst, right-aligned over `width` columns */
static void padFieldN(char* dst, const char* src, int srcLen, int width) {
  if (srcLen > width) srcLen = width;
  int pad = width - srcLen;
  for (int i = 0; i < pad; i++) *dst++ = ' ';
  memcpy(dst, src, srcLen);
  dst[srcLen] = 0;
}
static void padField(char* dst, const char* src, int width) {
  padFieldN(dst, src, strlen(src), width);
}

/* pressure field : "220" / "!180" / "--" / "----" */
static void fmtPressField(char* out, size_t outsz, const Tire& t) {
  if (!t.used)      { snprintf(out, outsz, "----"); return; }
  if (!tireFresh(t)){ snprintf(out, outsz, "--");   return; }
#if PRES_UNIT == 0
  int v = (int)lroundf(t.kPa);
  snprintf(out, outsz, tireAlarm(t) ? "!%d" : "%d", v);
#elif PRES_UNIT == 1
  float v = t.kPa / 100.0f;
  snprintf(out, outsz, tireAlarm(t) ? "!%.2f" : "%.2f", v);
#else
  float v = t.kPa / 6.894757f;   /* psi */
  if (v >= 100.0f) snprintf(out, outsz, tireAlarm(t) ? "!%d" : "%d", (int)lroundf(v));
  else             snprintf(out, outsz, tireAlarm(t) ? "!%.1f" : "%.1f", v);
#endif
}

/* temperature field "32°C" (degree glyph included) — returns length (max 4) */
static int fmtTempField(char* out, size_t outsz, float degC) {
  int t = (int)lroundf(degC);
  int L = snprintf(out, outsz < 6 ? outsz : 6, "%d", t);   // 4 digits max ("-60")
  if ((size_t)L + 3 <= outsz) {
    out[L]     = DEG_CH;   // degree glyph (CGRAM 0)
    out[L + 1] = 'C';
    out[L + 2] = 0;
    return L + 2;
  }
  return L;
}

/* -------------------------------------------------------------- rendering */
static void lcdWriteLine(uint8_t row, const char* s) {
  lcd.setCursor(0, row);
  for (int i = 0; i < 16; i++) lcd.write((uint8_t)s[i]);
}

static void renderLCD() {
  bool anyTire = false;
  for (int i = 0; i < MAX_TIRES; i++) if (tires[i].used) anyTire = true;

  if (!anyTire) {
    strcpy(line1, "TPMS waiting for");
    strcpy(line2, "sensors...      ");
  } else {
    float maxTemp = -999.0f, minVolt = 999.0f;
    for (int i = 0; i < MAX_TIRES; i++) {
      if (!tireFresh(tires[i])) continue;
      if (tires[i].degC  > maxTemp) maxTemp = tires[i].degC;
      if (tires[i].volts < minVolt) minVolt = tires[i].volts;
    }

    char f[10];

    /* ---- row 1 : FL | max temp | FR ---- */
    fmtPressField(f, sizeof(f), tires[0]); padField(line1, f, 5);
    if (maxTemp > -900.0f) {
      int L2 = fmtTempField(f, sizeof(f), maxTemp);
      padFieldN(line1 + 5, f, L2, 6);
    } else {
      padField(line1 + 5, "--", 6);
    }
    fmtPressField(f, sizeof(f), tires[1]); padField(line1 + 11, f, 5);

    /* ---- row 2 : RL | battery (or blinking ALERT!) | RR ---- */
    fmtPressField(f, sizeof(f), tires[2]); padField(line2, f, 5);
    if (alarmActive) {
      bool on = ((millis() / 500) % 2) == 0;
      padField(line2 + 5, on ? "ALERT!" : " ", 6);
    } else {
      if (minVolt < 900.0f) snprintf(f, sizeof(f), "%.1fV", minVolt);
      else                  snprintf(f, sizeof(f), "--");
      padField(line2 + 5, f, 6);
    }
    fmtPressField(f, sizeof(f), tires[3]); padField(line2 + 11, f, 5);
  }

  /* redraw only on change (degree glyphs are NUL bytes, so compare with
   * memcmp over 16 bytes, not strcmp) */
  if (memcmp(line1, lastLine1, 16) != 0) {
    memcpy(lastLine1, line1, 17);
    lcdWriteLine(0, line1);
  }
  if (memcmp(line2, lastLine2, 16) != 0) {
    memcpy(lastLine2, line2, 17);
    lcdWriteLine(1, line2);
  }
}

/* ----------------------------------------------------------- BLE / scan */
static void upsertTire(uint8_t slot, const String& id, float kPa, float degC, float volts, int8_t rssi) {
  int idx = -1;
  if (slot < MAX_TIRES) idx = slot;
  if (idx >= 0 && tires[idx].used && tires[idx].id.length() && tires[idx].id != id) idx = -1;
  if (idx < 0 || !tires[idx].used) {
    for (int i = 0; i < MAX_TIRES; i++)
      if (tires[i].used && tires[i].id == id) { idx = i; break; }
  }
  if (idx < 0 || (!tires[idx].used && slot >= MAX_TIRES)) {
    for (int i = 0; i < MAX_TIRES; i++)
      if (!tires[i].used) { idx = i; break; }
  }
  if (idx < 0 || idx >= MAX_TIRES) return;

  Tire& t = tires[idx];
  t.used = true; t.id = id; t.slot = slot;
  t.kPa = kPa; t.degC = degC; t.volts = volts;
  t.rssi = rssi; t.lastSeen = millis();
}

class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    const uint8_t* p = dev.getPayload();
    size_t n = dev.getPayloadLength();
    if (!p || n < (size_t)MIN_LEN) return;

    uint8_t m0 = getByte(p, n, OFF_MARKER);
    uint8_t m1 = getByte(p, n, OFF_MARKER + 1);
    uint8_t slotByte = getByte(p, n, OFF_SLOT);

    if (m0 == 0x00 && m1 == 0x01) {
      char idbuf[8];
      snprintf(idbuf, sizeof(idbuf), "%02x%02x%02x",
               getByte(p, n, OFF_ID), getByte(p, n, OFF_ID + 1), getByte(p, n, OFF_ID + 2));
      uint32_t pRaw = u32le(p, n, OFF_PRESS);
      uint32_t tRaw = u32le(p, n, OFF_TEMP);
      uint8_t  b    = getByte(p, n, OFF_BATT);

      float kPa = pRaw / 100000.0f, degC = tRaw / 100.0f, v = b / 10.0f;
      if (kPa >= 0.0f && kPa <= 1500.0f && degC >= -60.0f && degC <= 150.0f && v >= 0.0f && v <= 5.0f) {
        uint8_t slot = (slotByte >= 0x80 && slotByte <= 0x83) ? (slotByte - 0x80) : 0xFF;
        upsertTire(slot, String(idbuf), kPa, degC, v, dev.getRSSI());
        Serial.printf("[TPMS] id=%s slot=0x%02x %.1fkPa %.1fC %.1fV rssi=%d\n",
                      idbuf, slotByte, kPa, degC, v, dev.getRSSI());
      }
    } else {
      static char seen[24][18];
      static int  seenN = 0;
      char mac[18];
      strncpy(mac, dev.getAddress().toString().c_str(), sizeof(mac));
      mac[17] = 0;
      bool known = false;
      for (int i = 0; i < seenN; i++) if (!strcmp(seen[i], mac)) { known = true; break; }
      if (!known) {
        if (seenN < 24) strncpy(seen[seenN++], mac, sizeof(seen[0]));
        char hex[100];
        size_t j = 0;
        for (size_t i = 0; i < n && j + 3 < sizeof(hex); i++) { sprintf(hex + j, "%02x", p[i]); j += 2; }
        hex[j] = 0;
        Serial.printf("[SNIFF] %s name=%s rssi=%d len=%u hex=%s\n",
                      mac, dev.haveName() ? dev.getName().c_str() : "-", dev.getRSSI(), (unsigned)n, hex);
      }
    }
  }
};

static ScanCB scanCB;

static void startScan() {
  scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&scanCB, true);
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(90);
  scan->start(0, nullptr, false);
  lastScanRestart = millis();
  Serial.println("[BLE] passive scan started");
}

/* ------------------------------------------------------------- setup/loop */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== TPMS LCD Display (BLE TPMS family) ===");

  if (BUZZER_PIN >= 0) { pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW); }

  LCD_INIT();
  lcd.createChar(0, degGlyph);   // degree glyph -> code 0
  lcd.clear();

  BLEDevice::init("TPMS-LCD");
  startScan();
  lastRefresh = millis();
}

void loop() {
  uint32_t now = millis();

  if (now - lastRefresh >= REFRESH_MS) {
    lastRefresh = now;
    alarmActive = false;
    for (int i = 0; i < MAX_TIRES; i++) if (tireAlarm(tires[i])) alarmActive = true;
    renderLCD();
  }

  /* buzzer : short beep every 4 s while an alarm is active */
  if (BUZZER_PIN >= 0) {
    if (alarmActive && beepOffAt == 0 && now - lastBeep > 4000) {
      lastBeep = now; beepOffAt = now + 250;
      digitalWrite(BUZZER_PIN, HIGH);
    }
    if (beepOffAt && now >= beepOffAt) { digitalWrite(BUZZER_PIN, LOW); beepOffAt = 0; }
  }

  /* preventive scan restart every 15 min */
  if (now - lastScanRestart > 900000UL) {
    scan->stop();
    scan->clearResults();
    startScan();
  }
}
