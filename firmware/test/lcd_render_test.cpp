// Native test of the LCD screen composition (buffers, degree glyph, alignment)
// and of the positioning algorithm (pinned > reported position > ascending ID).
// Build : g++ -O0 -o /tmp/lcd_test lcd_render_test.cpp && /tmp/lcd_test
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdlib>

static int fails = 0;
static void checkBytes(const char* what, const char* got, const char* want, int n) {
  if (memcmp(got, want, n) == 0) { printf("%-30s OK\n", what); return; }
  printf("%-30s FAIL\n  got : ", what);
  for (int i = 0; i < n; i++) printf("%02x ", (unsigned char)got[i]);
  printf("\n  want: ");
  for (int i = 0; i < n; i++) printf("%02x ", (unsigned char)want[i]);
  printf("\n");
  fails++;
}

/* ---- copies of tpms_lcd.ino ---- */
#define DEG_CH ((char)0)
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
static int fmtTempField(char* out, size_t outsz, float degC) {
  int t = (int)lroundf(degC);
  int L = snprintf(out, outsz < 6 ? outsz : 6, "%d", t);
  if ((size_t)L + 3 <= outsz) {
    out[L] = DEG_CH; out[L + 1] = 'C'; out[L + 2] = 0;
    return L + 2;
  }
  return L;
}
static void fmtPressField(char* out, size_t outsz, bool used, bool stale, bool alarm, int kPa) {
  if (!used)  { snprintf(out, outsz, "----"); return; }
  if (stale)  { snprintf(out, outsz, "--"); return; }
  snprintf(out, outsz, alarm ? "!%d" : "%d", kPa);
}

/* psi variant (copy of tpms_lcd.ino, PRES_UNIT 2) */
static void fmtPressPsi(char* out, size_t outsz, bool alarm, float psi) {
  if (psi >= 100.0f) snprintf(out, outsz, alarm ? "!%d" : "%d", (int)lroundf(psi));
  else               snprintf(out, outsz, alarm ? "!%.1f" : "%.1f", psi);
}
static void checkStr(const char* what, const char* got, const char* want) {
  if (!strcmp(got, want)) { printf("%-30s OK\n", what); return; }
  printf("%-30s FAIL  got='%s' want='%s'\n", what, got, want);
  fails++;
}
static void checkInt(const char* what, int got, int want) {
  printf("%-30s %s  got=%d want=%d\n", what, got == want ? "OK" : "FAIL", got, want);
  if (got != want) fails++;
}

/* ---- arrangement algorithm (copy of tpms_lcd.ino layout logic) ---- */
struct TEntry { int id; int slot; };            // slot : -1 none, 0..3 reported
static bool gIdMode;
static int  gPinIds[8], gPinSlots[8], gPinN;
static int pinFor(int id) {
  for (int i = 0; i < gPinN; i++) if (gPinIds[i] == id && gPinSlots[i] < 4) return gPinSlots[i];
  return -1;
}
static void arrangeT(const TEntry in[4], TEntry out[4]) {
  if (!gIdMode) {
    for (int i = 0; i < 4 && !gIdMode; i++) {
      if (in[i].id < 0 || in[i].slot < 0 || in[i].slot > 3) continue;
      if (pinFor(in[i].id) >= 0) continue;
      for (int j = i + 1; j < 4; j++) {
        if (in[j].id < 0 || in[j].slot < 0 || in[j].slot > 3) continue;
        if (pinFor(in[j].id) >= 0) continue;
        if (in[j].slot == in[i].slot) { gIdMode = true; break; }
      }
    }
  }
  int order[4] = { -1, -1, -1, -1 }; bool posT[4] = { false, false, false, false };
  bool done[4] = { false, false, false, false };
  for (int i = 0; i < 4; i++) {
    if (in[i].id < 0) continue;
    int ps = pinFor(in[i].id);
    if (ps >= 0 && !posT[ps]) { order[ps] = i; posT[ps] = true; done[i] = true; }
  }
  if (!gIdMode) for (int i = 0; i < 4; i++) {
    if (in[i].id < 0 || done[i] || in[i].slot < 0 || in[i].slot > 3) continue;
    int ps = in[i].slot;
    if (!posT[ps]) { order[ps] = i; posT[ps] = true; done[i] = true; }
  }
  for (;;) {
    int best = -1;
    for (int i = 0; i < 4; i++) {
      if (in[i].id < 0 || done[i]) continue;
      if (best < 0 || in[i].id < in[best].id) best = i;
    }
    if (best < 0) break;
    int ps = 0; while (ps < 4 && posT[ps]) ps++;
    if (ps >= 4) break;
    order[ps] = best; posT[ps] = true; done[best] = true;
  }
  for (int p = 0; p < 4; p++) out[p] = (order[p] >= 0) ? in[order[p]] : TEntry{-1, -1};
}

int main() {
  char f[10];
  char line1[17], line2[17];

  /* case 1 : everything normal */
  fmtPressField(f, sizeof(f), true, false, false, 220); padField(line1, f, 5);
  int L2 = fmtTempField(f, sizeof(f), 32.0f);            padFieldN(line1 + 5, f, L2, 6);
  fmtPressField(f, sizeof(f), true, false, false, 218); padField(line1 + 11, f, 5);
  fmtPressField(f, sizeof(f), true, false, false, 221); padField(line2, f, 5);
  padField(line2 + 5, "2.9V", 6);
  fmtPressField(f, sizeof(f), true, false, false, 219); padField(line2 + 11, f, 5);

  const char w1[17] = {' ',' ','2','2','0',' ',' ','3','2', 0 ,'C',' ',' ','2','1','8', 0};
  const char w2[17] = {' ',' ','2','2','1',' ',' ','2','.','9','V',' ',' ','2','1','9', 0};
  checkBytes("normal case row 1", line1, w1, 16);
  checkBytes("normal case row 2", line2, w2, 16);

  /* case 2 : tire alarm + stale + negative temp */
  fmtPressField(f, sizeof(f), true, false, true, 180);  padField(line1, f, 5);
  L2 = fmtTempField(f, sizeof(f), -12.0f);              padFieldN(line1 + 5, f, L2, 6);
  fmtPressField(f, sizeof(f), true, true, false, 0);    padField(line1 + 11, f, 5);
  const char w3[17] = {' ','!','1','8','0',' ','-','1','2', 0 ,'C',' ',' ',' ','-','-', 0};
  checkBytes("alarm + stale + -12C", line1, w3, 16);

  /* case 3 : unused tires (all three fields written, like renderLCD) */
  fmtPressField(f, sizeof(f), false, false, false, 0); padField(line1, f, 5);
  padField(line1 + 5, "--", 6);
  fmtPressField(f, sizeof(f), false, false, false, 0); padField(line1 + 11, f, 5);
  const char w4[17] = {' ','-','-','-','-',' ',' ',' ',' ','-','-',' ','-','-','-','-'};
  checkBytes("unused tires (full row)", line1, w4, 16);

  /* case 4 : psi formats (220.5 kPa ~ 32.0 psi ; >=100 psi -> integer) */
  char pb[10];
  fmtPressPsi(pb, sizeof(pb), false, 220.5f / 6.894757f);  checkStr("psi normal", pb, "32.0");
  fmtPressPsi(pb, sizeof(pb), true,  105.0f);              checkStr("psi >=100 + alarm", pb, "!105");
  fmtPressPsi(pb, sizeof(pb), false, 105.0f);              checkStr("psi >=100 normal", pb, "105");
  fmtPressPsi(pb, sizeof(pb), true,  33.1f);               checkStr("psi normal alarm", pb, "!33.1");
  padField(line1, pb, 5);
  checkStr("psi 5-column field bound", line1, "!33.1");

  /* case 5 : arrangement — distinct reported positions -> slot order */
  gIdMode = false; gPinN = 0;
  const TEntry in5[4] = {{50,0},{40,1},{60,2},{30,3}};
  TEntry out5[4];
  arrangeT(in5, out5);
  checkInt("layout: slot FL", out5[0].id, 50);
  checkInt("layout: slot FR", out5[1].id, 40);
  checkInt("layout: slot RL", out5[2].id, 60);
  checkInt("layout: slot RR", out5[3].id, 30);
  checkInt("layout: slot mode kept", gIdMode ? 1 : 0, 0);

  /* case 6 : all sensors report the same position -> ascending-ID order */
  gIdMode = false;
  const TEntry in6[4] = {{50,0},{20,0},{35,0},{10,0}};
  TEntry out6[4];
  arrangeT(in6, out6);
  checkInt("layout: id-mode triggered", gIdMode ? 1 : 0, 1);
  checkInt("layout: ID order 1", out6[0].id, 10);
  checkInt("layout: ID order 2", out6[1].id, 20);
  checkInt("layout: ID order 3", out6[2].id, 35);
  checkInt("layout: ID order 4", out6[3].id, 50);

  /* case 7 : no position reported -> ascending-ID order */
  gIdMode = false;
  const TEntry in7[4] = {{50,-1},{20,-1},{-1,-1},{-1,-1}};
  TEntry out7[4];
  arrangeT(in7, out7);
  checkInt("layout: no-report order 1", out7[0].id, 20);
  checkInt("layout: no-report order 2", out7[1].id, 50);
  checkInt("layout: no-report slot free", out7[2].id, -1);

  /* case 8 : pinned sensor keeps its corner */
  gIdMode = false; gPinIds[0] = 50; gPinSlots[0] = 3; gPinN = 1;
  const TEntry in8[4] = {{50,0},{20,-1},{-1,-1},{-1,-1}};
  TEntry out8[4];
  arrangeT(in8, out8);
  checkInt("layout: pinned at RR", out8[3].id, 50);
  checkInt("layout: other at FL", out8[0].id, 20);
  gPinN = 0;

  printf("\n%s (%d failure(s))\n", fails ? "FAILED" : "ALL TESTS PASSED", fails);
  return fails ? 1 : 0;
}
