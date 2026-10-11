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
 *  Positioning : sensors are identified by their unique 6-hex ID. Display
 *  position priority : PINNED[] table (optional) > ascending sensor ID
 *  (default mode : smallest ID -> FL, then RL, FR, RR — deterministic
 *  across boots) > reported position byte (opt-in) — see arrange().
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
  uint32_t idNum    = 0;       // numeric 3-byte sensor ID (for ordering)
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

/* ------------------------------------------- sensor identity & positioning */
/* Optional pinning : sensor ID -> display position (0=FL, 1=FR, 2=RL, 3=RR).
 * Fill in the four IDs once identified (serial [TPMS] lines + inflating one
 * tire at a time) : pinned sensors always show in their corner, from the
 * very first frame, every boot. */
struct PinEntry { uint32_t id; uint8_t slot; };
/* Add one row per sensor once its ID is known, e.g. :
 *   { 0xaabbc0, 0 },   // AABB C0 -> front-left
 * The row below is a placeholder (real IDs are 3 bytes — it never matches). */
static const PinEntry PINNED[] = {
  { 0xFFFFFFFFu, 0xFF },
};
#define PINNED_N ((int)(sizeof(PINNED) / sizeof(PINNED[0])))
static int pinSlotFor(uint32_t idNum) {
  for (int i = 0; i < PINNED_N; i++)
    if (PINNED[i].id == idNum && PINNED[i].slot < MAX_TIRES) return PINNED[i].slot;
  return -1;
}

/* Display order mode. FORCE_ID_ORDER 1 (DEFAULT) : sensors are placed by
 * ascending sensor ID — convention : smallest ID -> FL, then RL, FR, RR
 * (left column top->bottom, then right column). Deterministic from the
 * moment all four have been heard, identical at every boot, immune to
 * packet arrival order. FORCE_ID_ORDER 0 : prefer the position byte each
 * sensor reports (0x80..0x83) while it looks trustworthy; switches to
 * ascending-ID order as soon as two sensors report the SAME position. */
#define FORCE_ID_ORDER 1
static bool idMode = FORCE_ID_ORDER;

/* Fill order for ascending sensor IDs : FL (top-left), RL (bottom-left),
 * FR (top-right), RR (bottom-right). */
static const uint8_t ID_FILL[MAX_TIRES] = { 0, 2, 1, 3 };

static int idxByIdNum(uint32_t idNum) {
  for (int i = 0; i < MAX_TIRES; i++)
    if (tires[i].used && tires[i].idNum == idNum) return i;
  return -1;
}

/* Rebuilt after every frame : place the pool sensors into display
 * positions. Priority : pinned > reported position (unless idMode) >
 * ascending sensor ID. */
static void arrange() {
  if (!idMode) {
    for (int i = 0; i < MAX_TIRES && !idMode; i++) {
      if (!tires[i].used || tires[i].slot > 3) continue;
      if (pinSlotFor(tires[i].idNum) >= 0) continue;
      for (int j = i + 1; j < MAX_TIRES; j++) {
        if (!tires[j].used || tires[j].slot > 3) continue;
        if (pinSlotFor(tires[j].idNum) >= 0) continue;
        if (tires[j].slot == tires[i].slot) { idMode = true; break; }
      }
    }
  }

  int  order[MAX_TIRES]    = { -1, -1, -1, -1 };  // display pos -> pool idx
  bool posTaken[MAX_TIRES] = { false, false, false, false };
  bool placed[MAX_TIRES]   = { false, false, false, false };

  /* 1) pinned sensors first */
  for (int i = 0; i < MAX_TIRES; i++) {
    if (!tires[i].used) continue;
    int ps = pinSlotFor(tires[i].idNum);
    if (ps >= 0 && !posTaken[ps]) {
      order[ps] = i; posTaken[ps] = true; placed[i] = true;
    }
  }
  /* 2) reported positions, when trustworthy */
  if (!idMode) {
    for (int i = 0; i < MAX_TIRES; i++) {
      if (!tires[i].used || placed[i] || tires[i].slot > 3) continue;
      int ps = tires[i].slot;
      if (!posTaken[ps]) { order[ps] = i; posTaken[ps] = true; placed[i] = true; }
    }
  }
  /* 3) the rest : ascending sensor ID into the positions left (fill order :
   * left column top->bottom, then right column — see ID_FILL) */
  for (;;) {
    int best = -1;
    for (int i = 0; i < MAX_TIRES; i++) {
      if (!tires[i].used || placed[i]) continue;
      if (best < 0 || tires[i].idNum < tires[best].idNum) best = i;
    }
    if (best < 0) break;
    int ps = -1;
    for (int k = 0; k < MAX_TIRES; k++)
      if (!posTaken[ID_FILL[k]]) { ps = ID_FILL[k]; break; }
    if (ps < 0) break;
    order[ps] = best; posTaken[ps] = true; placed[best] = true;
  }

  /* apply the new arrangement */
  Tire old[MAX_TIRES];
  for (int i = 0; i < MAX_TIRES; i++) old[i] = tires[i];
  for (int pos = 0; pos < MAX_TIRES; pos++) {
    if (order[pos] >= 0) tires[pos] = old[order[pos]];
    else { tires[pos].used = false; tires[pos].id = ""; tires[pos].idNum = 0; tires[pos].slot = 0xFF; }
  }
}

/* update the matching sensor entry (or create one), then re-layout */
static void placeFrame(uint32_t idNum, const String& id, uint8_t slotByte,
                       float kPa, float degC, float volts, int8_t rssi) {
  int idx = idxByIdNum(idNum);
  if (idx < 0) {
    for (int i = 0; i < MAX_TIRES; i++) if (!tires[i].used) { idx = i; break; }
    if (idx < 0) return;                       // pool full : 4 distinct sensors max
    tires[idx].used  = true;
    tires[idx].idNum = idNum;
    tires[idx].id    = id;
  }
  Tire& t = tires[idx];
  t.slot     = (slotByte >= 0x80 && slotByte <= 0x83) ? (uint8_t)(slotByte - 0x80) : 0xFF;
  t.kPa      = kPa;
  t.degC     = degC;
  t.volts    = volts;
  t.rssi     = rssi;
  t.lastSeen = millis();
  arrange();
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
        uint32_t idNum = ((uint32_t)getByte(p, n, OFF_ID) << 16)
                       | ((uint32_t)getByte(p, n, OFF_ID + 1) << 8)
                       |  (uint32_t)getByte(p, n, OFF_ID + 2);
        placeFrame(idNum, String(idbuf), slotByte, kPa, degC, v, dev.getRSSI());
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
