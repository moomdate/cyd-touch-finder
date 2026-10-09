// CYD Touch Finder: probe -> wait for a touch -> calibrate -> test. Hold BOOT while powering on to forget the
// saved result and probe again; press BOOT any time to toggle panel colour inversion.
#include <Arduino.h>
#include <Preferences.h>
#include <TFT_eSPI.h>

#include "logic.h"
#include "touch.h"

namespace {

constexpr int W = 320, H = 240;
constexpr int PIN_BOOT = 0;
constexpr int PIN_BL_ALT = 27;            // backlight on the capacitive "C" boards (21 on the "R" boards)
constexpr uint32_t BOOT_GUARD_MS = 1500;  // resistive panels report ghost presses at power-up
constexpr uint32_t POLL_MS = 15;
constexpr int PRESS_STREAK = 4;           // consecutive "down" polls before a wiring counts as the touch
constexpr float CAL_MAX_ERR_PX = 20;
constexpr const char* NVS_NS = "tfind";

constexpr uint16_t C_BG = TFT_BLACK, C_FG = TFT_WHITE, C_DIM = 0x8410, C_OK = TFT_GREEN, C_WARN = TFT_ORANGE,
                   C_BAD = TFT_RED, C_ACC = TFT_CYAN, C_BAR = 0x18E3;

TFT_eSPI tft;
Preferences prefs;

enum class Screen : uint8_t { Probe, Wait, Calibrate, Test };
Screen screen = Screen::Probe;

tf::TouchConfig cands[tf::MAX_FOUND];
int nCands = 0;
bool candsDetected = false;  // false: nothing answered the probe, so cands = every known wiring
tf::TouchConfig active;
logic::Affine cal;
bool invert = true;

// ---------------------------------------------------------------------------------------------------------------
// Drawing helpers
void text(const char* s, int x, int y, int font, uint16_t fg, uint8_t datum = TL_DATUM, uint16_t bg = C_BG) {
  tft.setTextDatum(datum);
  tft.setTextColor(fg, bg);
  tft.drawString(s, x, y, font);
}

void header(const char* title, uint16_t color) {
  tft.fillRect(0, 0, W, 22, C_BAR);
  text(title, 6, 11, 2, color, ML_DATUM, C_BAR);
}

void shortName(const tf::TouchConfig& c, char* buf, size_t n) {
  if (c.chip == tf::Chip::Xpt2046)
    snprintf(buf, n, "XPT2046 %s", c.bus == tf::Bus::SharedSpi ? "TFT bus" : "own pins");
  else
    snprintf(buf, n, "%s @%02X", tf::chipName(c.chip), c.addr);
}

// ---------------------------------------------------------------------------------------------------------------
// Persistence
bool loadResult() {
  if (!prefs.begin(NVS_NS, true)) return false;
  bool ok = prefs.getBytesLength("cfg") == sizeof active && prefs.getBytesLength("cal") == sizeof cal;
  if (ok) {
    prefs.getBytes("cfg", &active, sizeof active);
    prefs.getBytes("cal", &cal, sizeof cal);
  }
  prefs.end();
  return ok && active.version == tf::TouchConfig().version && active.chip != tf::Chip::None;
}

void saveResult() {
  prefs.begin(NVS_NS, false);
  prefs.putBytes("cfg", &active, sizeof active);
  prefs.putBytes("cal", &cal, sizeof cal);
  prefs.end();
}

void forgetResult() {
  prefs.begin(NVS_NS, false);
  prefs.remove("cfg");
  prefs.remove("cal");
  prefs.end();
}

void loadInvert() {
  if (prefs.begin(NVS_NS, true)) {
    invert = prefs.getBool("inv", true);
    prefs.end();
  }
}

void saveInvert() {
  prefs.begin(NVS_NS, false);
  prefs.putBool("inv", invert);
  prefs.end();
}

// ---------------------------------------------------------------------------------------------------------------
// Serial report: everything needed to make touch work in another project.
void printReport() {
  char chip[48], pins[64];
  tf::describeChip(active, chip, sizeof chip);
  tf::describePins(active, pins, sizeof pins);
  logic::SimpleCal s;
  bool simple = logic::toSimple(cal, W, H, s);
  Serial.println();
  Serial.println("==================== CYD Touch Finder result ====================");
  Serial.printf("chip : %s (%s)\n", chip, tf::chipKind(active.chip));
  Serial.printf("pins : %s\n", pins);
  if (active.chip == tf::Chip::Xpt2046)
    Serial.println("read : rx = cmd 0x91, ry = cmd 0xD1, pressed when Z1 (cmd 0xB1) > 150, ~1 MHz clock");
  Serial.printf("panel: rotation 1 (%dx%d landscape), inversion %s\n", W, H, invert ? "ON" : "OFF");
  Serial.println("calibration (raw -> screen):");
  Serial.printf("  sx = %.6f*rx + %.6f*ry + %.2f\n", cal.a, cal.b, cal.c);
  Serial.printf("  sy = %.6f*rx + %.6f*ry + %.2f\n", cal.d, cal.e, cal.f);
  if (simple) {
    Serial.println("  map() form:");
    Serial.printf("    #define TOUCH_SWAP_XY %d\n", s.swapXY);
    Serial.printf("    #define TOUCH_X_MIN %d   // raw at screen x = 0\n", s.xMin);
    Serial.printf("    #define TOUCH_X_MAX %d   // raw at screen x = %d\n", s.xMax, W - 1);
    Serial.printf("    #define TOUCH_Y_MIN %d   // raw at screen y = 0\n", s.yMin);
    Serial.printf("    #define TOUCH_Y_MAX %d   // raw at screen y = %d\n", s.yMax, H - 1);
    Serial.printf("    x = map(TOUCH_SWAP_XY ? ry : rx, TOUCH_X_MIN, TOUCH_X_MAX, 0, %d);\n", W - 1);
    Serial.printf("    y = map(TOUCH_SWAP_XY ? rx : ry, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, %d);\n", H - 1);
  }
  Serial.println("=================================================================");
}

// ---------------------------------------------------------------------------------------------------------------
// Probe
int logY = 0;
void probeLog(const char* line) {
  if (logY > H - 16) return;
  text(line, 6, logY, 2, C_FG);
  logY += 17;
}

void enterWait();

void runProbe() {
  screen = Screen::Probe;
  tft.fillScreen(C_BG);
  header("CYD Touch Finder - probing...", C_ACC);
  logY = 28;
  nCands = tf::probeAll(cands, tf::MAX_FOUND, probeLog);
  candsDetected = nCands > 0;
  if (!candsDetected) nCands = tf::candidates(cands, tf::MAX_FOUND);
  delay(candsDetected ? 1200 : 2000);  // let people read the log
  enterWait();
}

// ---------------------------------------------------------------------------------------------------------------
// Wait: poll every candidate until one reports a held press. Shows live raw values, so a board where nothing
// answers still tells you which wiring reacts to your finger.
int streak[tf::MAX_FOUND];
tf::RawTouch lastRaw[tf::MAX_FOUND];
bool lastOk[tf::MAX_FOUND];
int rowsY = 0;
uint32_t lastRows = 0, lastReprobe = 0;

void enterCalibrate();

void enterWait() {
  screen = Screen::Wait;
  memset(streak, 0, sizeof streak);
  tft.fillScreen(C_BG);
  int y = 28;
  if (candsDetected) {
    header(nCands > 1 ? "Touch chips found" : "Touch chip found", C_OK);
    for (int i = 0; i < nCands && i < 3; i++) {
      char chip[48], pins[64];
      tf::describeChip(cands[i], chip, sizeof chip);
      tf::describePins(cands[i], pins, sizeof pins);
      text(chip, 6, y, 2, C_OK);
      text(pins, 6, y + 16, 2, C_DIM);
      y += 36;
    }
  } else {
    header("No touch chip answered the probe", C_WARN);
    text("Trying every known wiring live.", 6, y, 2, C_FG);
    text("The row that turns green is your touch.", 6, y + 17, 2, C_FG);
    y += 38;
  }
  text("Press & hold anywhere", W / 2, y + 6, 4, C_ACC, TC_DATUM);
  rowsY = y + 40;
  lastRows = 0;
  lastReprobe = millis();
}

void drawRows() {
  for (int i = 0; i < nCands; i++) {
    int y = rowsY + i * 17;
    if (y > H - 16) break;
    char name[32], line[80];
    shortName(cands[i], name, sizeof name);
    const tf::RawTouch& t = lastRaw[i];
    if (!lastOk[i])
      snprintf(line, sizeof line, "%-16s no reply", name);
    else if (t.down)
      snprintf(line, sizeof line, "%-16s DOWN x=%4d y=%4d z=%4d", name, t.x, t.y, t.z);
    else
      snprintf(line, sizeof line, "%-16s up   z=%4d", name, t.z);
    tft.fillRect(0, y, W, 16, C_BG);
    text(line, 6, y, 2, !lastOk[i] ? C_DIM : t.down ? C_OK : C_FG);
  }
}

void waitTick() {
  static bool guardOver = false;
  if (!guardOver && millis() >= BOOT_GUARD_MS) guardOver = true;

  for (int i = 0; i < nCands; i++) {
    tf::RawTouch t;
    lastOk[i] = tf::read(cands[i], t);
    lastRaw[i] = t;
    streak[i] = (lastOk[i] && t.down && guardOver) ? streak[i] + 1 : 0;
    if (streak[i] < PRESS_STREAK) continue;

    active = cands[i];
    if (active.bus == tf::Bus::I2c && !candsDetected) {
      // Found by touch alone (e.g. a CST816 that slept through the probe): re-probe to read its chip id.
      tf::TouchConfig fresh[tf::MAX_FOUND];
      int n = tf::probeAll(fresh, tf::MAX_FOUND, nullptr);
      for (int k = 0; k < n; k++)
        if (fresh[k].bus == tf::Bus::I2c && fresh[k].addr == active.addr) active = fresh[k];
    }
    char chip[48], pins[64];
    tf::describeChip(active, chip, sizeof chip);
    tf::describePins(active, pins, sizeof pins);
    Serial.printf("[touch] press seen on %s, %s\n", chip, pins);
    enterCalibrate();
    return;
  }

  uint32_t now = millis();
  if (logic::elapsedMs(now, lastRows) >= 120) {
    lastRows = now;
    drawRows();
  }
  // Nothing answered: keep re-probing, a capacitive chip may wake up (CST816 sleeps until touched or reset).
  if (!candsDetected && logic::elapsedMs(now, lastReprobe) >= 5000) {
    lastReprobe = now;
    tf::TouchConfig fresh[tf::MAX_FOUND];
    int n = tf::probeAll(fresh, tf::MAX_FOUND, nullptr);
    if (n > 0) {
      memcpy(cands, fresh, sizeof fresh);
      nCands = n;
      candsDetected = true;
      enterWait();
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Calibrate: 4 corner targets fit the model, the 5th (centre) checks it.
constexpr int N_CAL = 4, N_TARGETS = 5;
constexpr int TX[N_TARGETS] = {30, W - 30, W - 30, 30, W / 2};
constexpr int TY[N_TARGETS] = {30, 30, H - 30, H - 30, H / 2};

enum class CalPhase : uint8_t { WaitRelease, WaitPress, Sampling, Message };
CalPhase calPhase;
int calStep = 0, nSamp = 0;
int sampX[32], sampY[32];
float rawX[N_TARGETS], rawY[N_TARGETS];
uint32_t phaseSince = 0;

void enterTest();

void drawTarget(int i, uint16_t col) {
  tft.drawFastHLine(TX[i] - 12, TY[i], 25, col);
  tft.drawFastVLine(TX[i], TY[i] - 12, 25, col);
  tft.drawCircle(TX[i], TY[i], 6, col);
}

void drawCalStep(const char* hint = nullptr) {
  tft.fillScreen(C_BG);
  text(hint ? hint : "Touch the + and hold until it turns green", W / 2, 62, 2, hint ? C_WARN : C_FG, TC_DATUM);
  char step[24];
  snprintf(step, sizeof step, "Point %d / %d", calStep + 1, N_TARGETS);
  text(step, W / 2, 168, 2, C_DIM, TC_DATUM);
  for (int i = 0; i < calStep; i++) drawTarget(i, C_OK);
  drawTarget(calStep, C_ACC);
}

void enterCalibrate() {
  screen = Screen::Calibrate;
  calStep = 0;
  calPhase = CalPhase::WaitRelease;  // the finger that picked the wiring is probably still down
  phaseSince = millis();
  drawCalStep();
}

void calFailed(const char* why) {
  Serial.printf("[cal] %s\n", why);
  tft.fillScreen(C_BG);
  text(why, W / 2, H / 2 - 20, 2, C_BAD, TC_DATUM);
  text("Starting over...", W / 2, H / 2 + 4, 2, C_FG, TC_DATUM);
  calPhase = CalPhase::Message;
  phaseSince = millis();
}

void targetDone() {
  int skip = nSamp / 4;  // the first readings come while the finger is still settling
  rawX[calStep] = logic::median(sampX + skip, nSamp - skip);
  rawY[calStep] = logic::median(sampY + skip, nSamp - skip);
  Serial.printf("[cal] target %d (%d,%d) raw %.0f %.0f\n", calStep + 1, TX[calStep], TY[calStep], rawX[calStep],
                rawY[calStep]);
  drawTarget(calStep, C_OK);
  calStep++;

  if (calStep == N_CAL) {
    float sx[N_CAL], sy[N_CAL];
    for (int i = 0; i < N_CAL; i++) { sx[i] = TX[i]; sy[i] = TY[i]; }
    if (!logic::fitAffine(rawX, rawY, sx, sy, N_CAL, cal)) return calFailed("Raw values don't move with the finger");
    float err = logic::maxError(cal, rawX, rawY, sx, sy, N_CAL);
    if (err > CAL_MAX_ERR_PX) {
      char why[64];
      snprintf(why, sizeof why, "Corners don't line up (%.0f px off)", err);
      return calFailed(why);
    }
  } else if (calStep == N_TARGETS) {
    float x, y;
    logic::apply(cal, rawX[N_CAL], rawY[N_CAL], x, y);
    float err = sqrtf((x - TX[N_CAL]) * (x - TX[N_CAL]) + (y - TY[N_CAL]) * (y - TY[N_CAL]));
    Serial.printf("[cal] centre check: %.1f px off\n", err);
    if (err > CAL_MAX_ERR_PX) {
      char why[64];
      snprintf(why, sizeof why, "Centre check %.0f px off", err);
      return calFailed(why);
    }
    saveResult();
    printReport();
    enterTest();
    return;
  }
  drawTarget(calStep, C_ACC);
  char step[24];
  snprintf(step, sizeof step, "Point %d / %d", calStep + 1, N_TARGETS);
  text(step, W / 2, 168, 2, C_DIM, TC_DATUM);
}

void calTick() {
  tf::RawTouch t;
  bool down = tf::read(active, t) && t.down;
  uint32_t now = millis();
  switch (calPhase) {
    case CalPhase::WaitRelease:
      if (down) phaseSince = now;
      else if (logic::elapsedMs(now, phaseSince) >= 250) calPhase = CalPhase::WaitPress;
      break;
    case CalPhase::WaitPress:
      if (down) { calPhase = CalPhase::Sampling; nSamp = 0; phaseSince = now; }
      break;
    case CalPhase::Sampling:
      if (!down) {  // lifted too early
        calPhase = CalPhase::WaitRelease;
        phaseSince = now;
        drawCalStep("Hold a little longer");
        break;
      }
      if (nSamp < 32) { sampX[nSamp] = t.x; sampY[nSamp] = t.y; nSamp++; }
      if (logic::elapsedMs(now, phaseSince) >= 400 && nSamp >= 8) {
        calPhase = CalPhase::WaitRelease;
        phaseSince = now;
        targetDone();
      }
      break;
    case CalPhase::Message:
      if (logic::elapsedMs(now, phaseSince) >= 2000) enterCalibrate();
      break;
  }
}

// ---------------------------------------------------------------------------------------------------------------
// Test: paint with your finger, plus Recalibrate / Rescan / Clear buttons.
constexpr int PAINT_TOP = 64, PAINT_BOTTOM = 196, BTN_Y = 202, BTN_H = 34;
struct Button { int x, w; const char* label; };
constexpr Button BTN_RECAL = {4, 102, "Recalibrate"}, BTN_RESCAN = {109, 102, "Rescan"},
                 BTN_CLEAR = {214, 102, "Clear"};

logic::TapFilter tapFilter;
bool wasDown = false;
int prevX = 0, prevY = 0;
uint32_t lastCoords = 0;

void drawButton(const Button& b) {
  tft.fillRoundRect(b.x, BTN_Y, b.w, BTN_H, 6, C_BAR);
  tft.drawRoundRect(b.x, BTN_Y, b.w, BTN_H, 6, C_DIM);
  text(b.label, b.x + b.w / 2, BTN_Y + BTN_H / 2, 2, C_FG, MC_DATUM, C_BAR);
}

bool hit(const Button& b, int x, int y) { return x >= b.x && x < b.x + b.w && y >= BTN_Y && y < BTN_Y + BTN_H; }

void clearPaint() {
  tft.fillRect(0, PAINT_TOP, W, PAINT_BOTTOM - PAINT_TOP, C_BG);
  tft.drawRect(0, PAINT_TOP, W, PAINT_BOTTOM - PAINT_TOP, C_BAR);
  text("draw here", W / 2, (PAINT_TOP + PAINT_BOTTOM) / 2, 2, C_BAR, MC_DATUM);
}

void enterTest() {
  screen = Screen::Test;
  tft.fillScreen(C_BG);
  char chip[48], line[80];
  tf::describeChip(active, chip, sizeof chip);
  snprintf(line, sizeof line, "%s  touch OK", chip);
  header(line, C_OK);
  tf::describePins(active, line, sizeof line);
  text(line, 6, 25, 2, C_FG);
  logic::SimpleCal s;
  if (logic::toSimple(cal, W, H, s))
    snprintf(line, sizeof line, "swapXY=%d  X %d..%d  Y %d..%d", s.swapXY, s.xMin, s.xMax, s.yMin, s.yMax);
  else
    snprintf(line, sizeof line, "calibration: affine only");
  text(line, 6, 43, 2, C_DIM);
  clearPaint();
  drawButton(BTN_RECAL);
  drawButton(BTN_RESCAN);
  drawButton(BTN_CLEAR);
  wasDown = false;
  tapFilter = logic::TapFilter();
}

void testTick() {
  tf::RawTouch t;
  bool down = tf::read(active, t) && t.down;
  int x = -1, y = -1;
  if (down) {
    float fx, fy;
    logic::apply(cal, t.x, t.y, fx, fy);
    x = (int)lroundf(fx);
    y = (int)lroundf(fy);
  }

  uint32_t now = millis();
  if (down && logic::elapsedMs(now, lastCoords) >= 100) {
    lastCoords = now;
    char c[40];
    snprintf(c, sizeof c, "x=%3d y=%3d  raw %4d %4d", x, y, t.x, t.y);
    tft.fillRect(W - 170, PAINT_TOP + 2, 168, 16, C_BG);
    text(c, W - 4, PAINT_TOP + 2, 2, C_ACC, TR_DATUM);
  }

  bool inPaint = down && x >= 0 && x < W && y > PAINT_TOP && y < PAINT_BOTTOM - 2;
  if (inPaint) {
    if (wasDown) tft.drawLine(prevX, prevY, x, y, C_ACC);
    tft.fillCircle(x, y, 2, C_ACC);
    prevX = x;
    prevY = y;
  }
  wasDown = inPaint;

  if (tapFilter.update(down, now, x, y, W, H)) {
    if (hit(BTN_CLEAR, x, y)) clearPaint();
    else if (hit(BTN_RECAL, x, y)) enterCalibrate();
    else if (hit(BTN_RESCAN, x, y)) { forgetResult(); runProbe(); }
  }
}

// ---------------------------------------------------------------------------------------------------------------
void bootButtonTick() {
  static bool armed = false, last = true;
  static uint32_t changedAt = 0;
  bool up = digitalRead(PIN_BOOT) == HIGH;
  uint32_t now = millis();
  if (up != last) { last = up; changedAt = now; return; }
  if (logic::elapsedMs(now, changedAt) < 50) return;
  if (up) { armed = true; return; }
  if (!armed) return;  // still held from power-on
  armed = false;
  invert = !invert;
  tft.invertDisplay(invert);
  saveInvert();
  Serial.printf("[panel] inversion %s\n", invert ? "ON" : "OFF");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(PIN_BL_ALT, OUTPUT);
  digitalWrite(PIN_BL_ALT, HIGH);

  tft.init();
  tft.setRotation(1);
  loadInvert();
  tft.invertDisplay(invert);
  tft.fillScreen(C_BG);

  bool forceProbe = digitalRead(PIN_BOOT) == LOW;
  if (forceProbe) {
    forgetResult();
    Serial.println("[boot] BOOT held: saved result cleared");
  } else if (loadResult() && tf::present(active)) {
    Serial.println("[boot] using saved result (hold BOOT at power-on to probe again)");
    printReport();
    enterTest();
    return;
  }
  runProbe();
}

void loop() {
  static uint32_t lastPoll = 0;
  bootButtonTick();
  uint32_t now = millis();
  if (logic::elapsedMs(now, lastPoll) < POLL_MS) return;
  lastPoll = now;
  switch (screen) {
    case Screen::Probe: break;
    case Screen::Wait: waitTick(); break;
    case Screen::Calibrate: calTick(); break;
    case Screen::Test: testTick(); break;
  }
}
