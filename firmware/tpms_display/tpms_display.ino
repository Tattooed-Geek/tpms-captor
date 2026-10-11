/*
 * ============================================================================
 *  TPMS BLE Display — for generic BLE TPMS sensors ("BLE TPMS" family)
 * ============================================================================
 *  Passively listens to the TPMS sensors' BLE advertisements (no connection,
 *  no pairing) and shows the values on a MAX7219 LED matrix (2 blocks =
 *  16x8). Built-in alarm buzzer.
 *
 *  Protocol decoded by reverse engineering of the official companion app
 *  (see docs/PROTOCOL.md in this repository).
 *
 *  Frame (positions in RAW BYTES ; in parentheses the equivalent positions
 *  in the hex string the Android app parses — the byte offsets are half
 *  of them, e.g. byte 9 = characters 18) :
 *    [9..10]  00 01             family marker              (hex 18..22)
 *    [11]     slot              0x80=FL, 0x81=FR, 0x82=RL, 0x83=RR
 *    [14..16] sensor ID         3 bytes (engraved code / QR)
 *    [17..20] pressure          LE32 / 100000 -> kPa       (hex 34..42)
 *    [21..24] temperature       LE32 / 100    -> degC      (hex 42..50)
 *    [25]     battery           / 10 -> volts              (hex 50..52)
 *
 *  Target : ESP32 DevKit (WROOM-32)
 *  Libs   : MD_MAX72xx + MD_Parola (majicdesigns) via the library manager
 *
 *  SNIFFER MODE : on first boot (or while no sensor has been recognized yet)
 *  every unknown BLE device is logged over serial with its complete frame —
 *  this is the validation tool for the day the sensors arrive.
 * ============================================================================
 */

#include <SPI.h>
#include <MD_MAX72xx.h>
#include <MD_Parola.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

/* ---------------------------------------------------------------- HW config */
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW   // generic modules (most common)
#define MAX_DEVICES   2                     // number of 8x8 blocks daisy-chained (2 = 16x8)
#define CS_PIN        5                     // matrix CS (DIN=23, CLK=18 : hardware SPI)
#define BUZZER_PIN    25                    // active buzzer (optional, -1 to disable)
#define DISPLAY_INTENSITY 3                 // 0..15 (3 = dim for night driving)

/* -------------------------------------------------- TPMS protocol config */
/* IMPORTANT : positions are BYTES in the raw frame.
 * The Android app reads a hex string ; its positions (18, 22, 28, 34, 42,
 * 50) must therefore be divided by 2 here : 9, 11, 14, 17, 21, 25. */
#define OFF_MARKER  9     /* bytes 9-10 : 00 01 (family marker) */
#define OFF_SLOT    11
#define OFF_ID      14    /* 3 bytes : 14, 15, 16 */
#define OFF_PRESS   17    /* LE32 : 17..20, /100000 -> kPa */
#define OFF_TEMP    21    /* LE32 : 21..24, /100   -> degC */
#define OFF_BATT    25    /* 1 byte, /10 -> V */
#define MIN_LEN     26

/* alarm thresholds — same default values as the official app */
#define TH_LOW_KPA   200.0f   // 2.0 bar
#define TH_HIGH_KPA  300.0f   // 3.0 bar
#define TH_TEMP_C     65.0f
#define TH_BATT_V      2.0f
#define STALE_MS    600000UL  // 10 min without data -> "no data"

/* display refresh */
#define MSG_REFRESH_MS  8000UL   // message rebuild period
#define SCROLL_SPEED    45       // Parola scroll speed
#define SCROLL_PAUSE    1000     // pause before looping (ms)

/* ------------------------------------------------------------- local state */
#define MAX_TIRES 4

struct Tire {
  bool     used     = false;
  String   id       = "";
  uint8_t  slot     = 0xFF;    // 0..3 (0xFF = unknown)
  float    kPa      = 0.0f;
  float    degC     = 0.0f;
  float    volts    = 0.0f;
  uint32_t lastSeen = 0;
  int8_t   rssi     = 0;
};

static Tire     tires[MAX_TIRES];
static uint32_t lastMsgBuild = 0;
static uint32_t lastBeep     = 0;
static uint32_t beepOffAt    = 0;
static bool     alarmActive  = false;
static String   curMsg       = "";
static BLEScan* scan         = nullptr;
static uint32_t lastScanRestart = 0;

MD_Parola display = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

/* -------------------------------------------------------------- utilities */
// raw byte at position `pos`
static uint8_t getByte(const uint8_t* p, size_t n, int pos) {
  return ((size_t)pos < n) ? p[pos] : 0;
}

// 32-bit little-endian integer from bytes pos..pos+3
static uint32_t u32le(const uint8_t* p, size_t n, int pos) {
  if ((size_t)pos + 3 >= n) return 0;
  return (uint32_t)p[pos]
       | ((uint32_t)p[pos + 1] << 8)
       | ((uint32_t)p[pos + 2] << 16)
       | ((uint32_t)p[pos + 3] << 24);
}

static void payloadToHex(const uint8_t* p, size_t n, char* out, size_t outMax) {
  size_t j = 0;
  for (size_t i = 0; i < n && j + 3 < outMax; i++) {
    sprintf(out + j, "%02x", p[i]);
    j += 2;
  }
  out[j] = 0;
}

/* -------------------------------------------------------------- rendering */
static String fmtTire(int i) {
  const Tire& t = tires[i];
  if (!t.used) return String(i + 1) + ":--";
  bool stale = (millis() - t.lastSeen) > STALE_MS;
  if (stale) return String(i + 1) + ":" + t.id + "?";
  String s = String(i + 1) + ":" + String(t.kPa / 6.894757f, 1) + "psi " + String(t.degC, 0) + "C";
  if (t.kPa < TH_LOW_KPA || t.kPa > TH_HIGH_KPA || t.degC > TH_TEMP_C || t.volts < TH_BATT_V) s += "!";
  return s;
}

static String buildMsg() {
  bool any = false;
  String m = "";
  for (int i = 0; i < MAX_TIRES; i++) {
    if (tires[i].used) any = true;
    m += fmtTire(i);
    if (i < MAX_TIRES - 1) m += "  ";
  }
  if (!any) return String("TPMS waiting for sensors  ");
  return m + "  ";
}

static void updateAlarm() {
  alarmActive = false;
  for (int i = 0; i < MAX_TIRES; i++) {
    const Tire& t = tires[i];
    if (!t.used) continue;
    bool stale = (millis() - t.lastSeen) > STALE_MS;
    if (stale) continue;
    if (t.kPa < TH_LOW_KPA || t.kPa > TH_HIGH_KPA || t.degC > TH_TEMP_C || t.volts < TH_BATT_V)
      alarmActive = true;
  }
}

/* ----------------------------------------------------------- BLE / scan */
static void upsertTire(uint8_t slot, const String& id, float kPa, float degC, float volts, int8_t rssi) {
  int idx = -1;
  // 1) find by slot (priority), 2) by id, 3) first free
  if (slot < MAX_TIRES) idx = slot;
  if (idx >= 0 && tires[idx].used && tires[idx].id.length() && tires[idx].id != id) {
    // the slot is taken by another sensor : look for the id elsewhere
    idx = -1;
  }
  if (idx < 0 || !tires[idx].used) {
    for (int i = 0; i < MAX_TIRES; i++)
      if (tires[i].used && tires[i].id == id) { idx = i; break; }
  }
  if (idx < 0 || (!tires[idx].used && slot >= MAX_TIRES)) {
    for (int i = 0; i < MAX_TIRES; i++)
      if (!tires[i].used) { idx = i; break; }
  }
  if (idx < 0 || idx >= MAX_TIRES) return; // full : ignore

  Tire& t = tires[idx];
  t.used     = true;
  t.id       = id;
  t.slot     = slot;
  t.kPa      = kPa;
  t.degC     = degC;
  t.volts    = volts;
  t.rssi     = rssi;
  t.lastSeen = millis();
}

class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    const uint8_t* p = dev.getPayload();
    size_t n = dev.getPayloadLength();
    if (!p || n == 0) return;

    uint8_t m0 = getByte(p, n, OFF_MARKER);
    uint8_t m1 = getByte(p, n, OFF_MARKER + 1);
    uint8_t slotByte = getByte(p, n, OFF_SLOT);

    /* decode only full-length frames carrying the family marker; anything
     * else (including short frames) is logged by the sniffer below */
    if (n >= (size_t)MIN_LEN && m0 == 0x00 && m1 == 0x01) {
      /* recognized TPMS frame */
      char idbuf[8];
      snprintf(idbuf, sizeof(idbuf), "%02x%02x%02x",
               getByte(p, n, OFF_ID), getByte(p, n, OFF_ID + 1), getByte(p, n, OFF_ID + 2));

      uint32_t pRaw = u32le(p, n, OFF_PRESS);
      uint32_t tRaw = u32le(p, n, OFF_TEMP);
      uint8_t  b    = getByte(p, n, OFF_BATT);

      float kPa  = pRaw / 100000.0f;
      float degC = tRaw / 100.0f;
      float v    = b / 10.0f;

      /* plausibility filter (protects against false positives) */
      if (kPa >= 0.0f && kPa <= 1500.0f && degC >= -60.0f && degC <= 150.0f && v >= 0.0f && v <= 5.0f) {
        uint8_t slot = (slotByte >= 0x80 && slotByte <= 0x83) ? (slotByte - 0x80) : 0xFF;
        upsertTire(slot, String(idbuf), kPa, degC, v, dev.getRSSI());
        Serial.printf("[TPMS] id=%s slot=0x%02x %.1fkPa %.1fC %.1fV rssi=%d\n",
                      idbuf, slotByte, kPa, degC, v, dev.getRSSI());
      }
    } else {
      /* unknown : single log per address, useful on validation day */
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
        payloadToHex(p, n, hex, sizeof(hex));
        Serial.printf("[SNIFF] %s name=%s rssi=%d len=%u hex=%s\n",
                      mac, dev.haveName() ? dev.getName().c_str() : "-", dev.getRSSI(), (unsigned)n, hex);
      }
    }
  }
};

static ScanCB scanCB;

static void startScan() {
  scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&scanCB, true); // true = duplicates (fresh values)
  scan->setActiveScan(false);                        // PASSIVE : we don't wake the sensors
  scan->setInterval(100);
  scan->setWindow(90);
  scan->start(0, nullptr, false);                    // 0 = no time limit
  lastScanRestart = millis();
  Serial.println("[BLE] passive scan started");
}

/* ------------------------------------------------------------- setup/loop */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== TPMS BLE Display (BLE TPMS family) ===");

  if (BUZZER_PIN >= 0) {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
  }

  display.begin();
  display.setIntensity(DISPLAY_INTENSITY);
  display.displayClear();
  display.displayZoneText(0, (char*)"TPMS", PA_CENTER, 0, 800, PA_PRINT, PA_NO_EFFECT);

  BLEDevice::init("TPMS-Display");
  startScan();

  lastMsgBuild = millis();
}

void loop() {
  /* matrix animation */
  if (display.displayAnimate()) {
    display.displayReset();   // loop the current scroll
  }

  /* periodic rebuild of the displayed message */
  uint32_t now = millis();
  if (now - lastMsgBuild >= MSG_REFRESH_MS) {
    lastMsgBuild = now;
    updateAlarm();
    String m = buildMsg();
    if (alarmActive) m = "ALARM!  " + m;
    if (m != curMsg) {
      curMsg = m;
      display.displayZoneText(0, (char*)curMsg.c_str(), PA_LEFT,
                              SCROLL_SPEED, SCROLL_PAUSE, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
    }
  }

  /* buzzer : short beep every 4 s while an alarm is active */
  if (BUZZER_PIN >= 0) {
    if (alarmActive && beepOffAt == 0 && now - lastBeep > 4000) {
      lastBeep  = now;
      beepOffAt = now + 250;
      digitalWrite(BUZZER_PIN, HIGH);
    }
    if (beepOffAt && now >= beepOffAt) {
      digitalWrite(BUZZER_PIN, LOW);
      beepOffAt = 0;
    }
  }

  /* preventive scan restart every 15 min (BLE stack hygiene) */
  if (now - lastScanRestart > 900000UL) {
    scan->stop();
    scan->clearResults();
    startScan();
  }
}
