// Native test of the TPMS decoding : reproduces the Android app formula
// (hex string) and checks that the ESP32 byte decoding gives the same result.
// Build : g++ -O0 -o /tmp/tpms_test tpms_parse_test.cpp && /tmp/tpms_test
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <string>

// ---- ESP32 helpers (copies of tpms_display.ino) ----
static uint8_t getByte(const uint8_t* p, size_t n, int pos) {
  return ((size_t)pos < n) ? p[pos] : 0;
}
static uint32_t u32le(const uint8_t* p, size_t n, int pos) {
  if ((size_t)pos + 3 >= n) return 0;
  return (uint32_t)p[pos]
       | ((uint32_t)p[pos + 1] << 8)
       | ((uint32_t)p[pos + 2] << 16)
       | ((uint32_t)p[pos + 3] << 24);
}

// ---- Android app equivalence : hex string + parseInt(s4+s3+s2+s1) ----
static std::string toHex(const uint8_t* p, size_t n) {
  std::string s;
  char b[3];
  for (size_t i = 0; i < n; i++) { sprintf(b, "%02x", p[i]); s += b; }
  return s;
}
static uint32_t javaU32(const std::string& hex, int charPos) {
  std::string v = hex.substr(charPos + 6, 2) + hex.substr(charPos + 4, 2)
                + hex.substr(charPos + 2, 2) + hex.substr(charPos + 0, 2);
  return (uint32_t)strtoul(v.c_str(), nullptr, 16);
}

static int fails = 0;
static void check(const char* what, bool ok, double got, double want) {
  printf("%-28s %s  got=%.4f want=%.4f\n", what, ok ? "OK" : "FAIL", got, want);
  if (!ok) fails++;
}

int main() {
  // test frame : tire 1, id AABB C0, 220.5 kPa, 21.0 C, 2.9 V
  uint8_t pdu[26];
  memset(pdu, 0, sizeof(pdu));
  pdu[0]=0x02; pdu[1]=0x01; pdu[2]=0x06;
  pdu[3]=0x03; pdu[4]=0x03; pdu[5]=0xA5; pdu[6]=0x27;
  pdu[7]=0x12; pdu[8]=0xFF;
  pdu[9]=0x00; pdu[10]=0x01;
  pdu[11]=0x80;
  pdu[14]=0xAA; pdu[15]=0xBB; pdu[16]=0xC0;

  uint32_t pRaw = (uint32_t)(220.5f * 100000.0f + 0.5f);
  uint32_t tRaw = (uint32_t)(21.0f * 100.0f + 0.5f);
  pdu[17]=pRaw & 0xFF; pdu[18]=(pRaw>>8)&0xFF; pdu[19]=(pRaw>>16)&0xFF; pdu[20]=(pRaw>>24)&0xFF;
  pdu[21]=tRaw & 0xFF; pdu[22]=(tRaw>>8)&0xFF; pdu[23]=(tRaw>>16)&0xFF; pdu[24]=(tRaw>>24)&0xFF;
  pdu[25]=29;

  // --- ESP32 decoding (bytes) ---
  bool markerOk = (getByte(pdu, 26, 9) == 0x00) && (getByte(pdu, 26, 10) == 0x01);
  check("marker bytes 9-10", markerOk, getByte(pdu,26,9)*256+getByte(pdu,26,10), 1);
  check("slot byte 11", getByte(pdu,26,11) == 0x80, getByte(pdu,26,11), 0x80);

  char id[8];
  snprintf(id, sizeof(id), "%02x%02x%02x", getByte(pdu,26,14), getByte(pdu,26,15), getByte(pdu,26,16));
  printf("%-28s %s  got=%s want=aabbc0\n", "id bytes 14-16", strcmp(id,"aabbc0")==0?"OK":"FAIL", id);
  if (strcmp(id,"aabbc0")!=0) fails++;

  float kPa_esp  = u32le(pdu, 26, 17) / 100000.0f;
  float degC_esp = u32le(pdu, 26, 21) / 100.0f;
  float v_esp    = getByte(pdu, 26, 25) / 10.0f;
  check("pressure ESP32", kPa_esp == 220.5f, kPa_esp, 220.5);
  check("temperature ESP32", degC_esp == 21.0f, degC_esp, 21.0);
  check("battery ESP32", v_esp == 2.9f, v_esp, 2.9);

  // --- Android formula equivalence (hex string) ---
  std::string hex = toHex(pdu, sizeof(pdu));
  float kPa_java  = javaU32(hex, 34) / 100000.0f;   // substring(34..42)
  float degC_java = javaU32(hex, 42) / 100.0f;      // substring(42..50)
  float v_java    = (float)strtoul(hex.substr(50, 2).c_str(), nullptr, 16) / 10.0f;
  check("pressure == app formula", kPa_java == kPa_esp, kPa_java, kPa_esp);
  check("temperature == app formula", degC_java == degC_esp, degC_java, degC_esp);
  check("battery == app formula", v_java == v_esp, v_java, v_esp);

  printf("\n%s (%d failure(s))\n", fails ? "FAILED" : "ALL TESTS PASSED", fails);
  return fails ? 1 : 0;
}
