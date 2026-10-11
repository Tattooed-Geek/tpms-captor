/*
 * ============================================================================
 *  TPMS BLE Emitter — SENSOR SIMULATOR (for testing the display)
 * ============================================================================
 *  Broadcasts Bluetooth advertisements with EXACTLY the same frame as the
 *  generic BLE TPMS sensors ("BLE TPMS" family — see docs/PROTOCOL.md).
 *  Lets you validate the whole chain (reception + decoding + display +
 *  alarm) BEFORE the real sensors arrive.
 *
 *  Usage : flash this sketch on a second ESP32. It cycles through the
 *  4 simulated tires (a different tire every 5 seconds, because a classic
 *  ESP32 can only advertise one frame at a time).
 *    Tire 1 : 220.5 kPa  21 C  2.9 V
 *    Tire 2 : 218.0 kPa  22 C  2.9 V
 *    Tire 3 : 224.3 kPa  23 C  3.0 V
 *    Tire 4 : 180.0 kPa  24 C  2.8 V   <- below threshold : triggers the alarm
 *  (Set ALARM_TEST to 0 for a normal tire 4 at 222.1 kPa.)
 *
 *  Emitted frame (26 bytes, aligned with the decoder offsets) :
 *    [0..2]   02 01 06            flags
 *    [3..6]   03 03 a5 27         dummy service UUID (filler)
 *    [7]      12                  manufacturer data length
 *    [8]      ff                  manufacturer data type
 *    [9..10]  00 01               TPMS family marker
 *    [11]     slot                0x80..0x83
 *    [12..13] 00 00               reserved
 *    [14..16] sensor ID           3 bytes
 *    [17..20] pressure            LE32 (/100000 -> kPa)
 *    [21..24] temperature         LE32 (/100 -> degC)
 *    [25]     battery             (/10 -> V)
 * ============================================================================
 */

#include <BLEDevice.h>
#include <BLEAdvertising.h>

#define ALARM_TEST 1   // 1 = tire 4 below threshold (alarm test)

static BLEAdvertising* adv = nullptr;
static int  slotIdx = 0;
static uint32_t lastRotate = 0;
static bool advReady = false;   // false until the BLE controller has started

/* simulated values per tire (index = slot 0..3) */
static const float MOCK_KPA[4]  = {220.5f, 218.0f, 224.3f, 222.1f};
static const float MOCK_DEGC[4] = {21.0f,  22.0f,  23.0f,  24.0f};
static const float MOCK_VOLT[4] = {2.9f,   2.9f,   3.0f,   2.8f};

static void putU32le(uint8_t* p, uint32_t v) {
  p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

static void startSlot(int i) {
  uint8_t pdu[26];
  memset(pdu, 0, sizeof(pdu));
  pdu[0] = 0x02; pdu[1] = 0x01; pdu[2] = 0x06;              // flags
  pdu[3] = 0x03; pdu[4] = 0x03; pdu[5] = 0xA5; pdu[6] = 0x27; // dummy service uuid
  pdu[7] = 0x12; pdu[8] = 0xFF;                               // manufacturer data (len=18)
  pdu[9] = 0x00; pdu[10] = 0x01;                              // family marker
  pdu[11] = (uint8_t)(0x80 + i);                              // slot
  pdu[12] = 0x00; pdu[13] = 0x00;
  pdu[14] = 0xAA; pdu[15] = 0xBB; pdu[16] = (uint8_t)(0xC0 + i); // simulated sensor ID

  float kPa  = MOCK_KPA[i];
  float degC = MOCK_DEGC[i];
  float volt = MOCK_VOLT[i];
  if (ALARM_TEST && i == 3) kPa = 180.0f;

  putU32le(&pdu[17], (uint32_t)lroundf(kPa * 100000.0f));
  putU32le(&pdu[21], (uint32_t)lroundf(degC * 100.0f));
  pdu[25] = (uint8_t)lroundf(volt * 10.0f);

  BLEAdvertisementData data;
  data.addData((char*)pdu, sizeof(pdu));   // exact raw frame (core 3.x)

  adv->stop();
  delay(50);
  adv->setAdvertisementData(data);
  adv->setScanResponseData(data);
  advReady = false;
  if (adv->start()) advReady = true;

  Serial.printf("[EMIT] tire %d : slot=0x%02x id=aabb%02x %.1fkPa %.1fC %.1fV (advReady=%d)\n",
                i + 1, 0x80 + i, 0xC0 + i, kPa, degC, volt, (int)advReady);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("=== TPMS BLE Emitter (simulator) ===");

  BLEDevice::init("TPMS_EMU");
  adv = BLEDevice::getAdvertising();
  adv->setMinInterval(320);   // ~200 ms (0.625 ms units)
  adv->setMaxInterval(480);

  startSlot(0);
  lastRotate = millis();
}

void loop() {
  /* wait until the BLE controller is ready (host sync) */
  if (!advReady) {
    if (adv->start()) {
      advReady = true;
      lastRotate = millis();
      Serial.println("[EMIT] advertising started");
    } else {
      Serial.println("[EMIT] host not synced, retrying in 1 s...");
      delay(1000);
      return;
    }
  }

  if (millis() - lastRotate >= 5000) {
    lastRotate = millis();
    slotIdx = (slotIdx + 1) % 4;
    startSlot(slotIdx);
  }
}
