// Native test of the LCD screen composition (buffers, degree glyph, alignment).
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

  printf("\n%s (%d failure(s))\n", fails ? "FAILED" : "ALL TESTS PASSED", fails);
  return fails ? 1 : 0;
}
