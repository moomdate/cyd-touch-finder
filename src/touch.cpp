#include "touch.h"

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>

#include "logic.h"

namespace tf {
namespace {

// ---------------------------------------------------------------------------------------------------------------
// Known wirings.
//  A  ESP32-2432S028R / CYD2USB / most 2.8" "R" boards: XPT2046 on its own pins, bit-banged.
//  B  ESP32-2432S024R / 2432S032R / 3248S035R: XPT2046 sharing the panel's SPI bus (SCK14 MOSI13 MISO12), CS 33.
//  C  "C" boards (2432S024C, 2432S028C, JC2432W328C, 3248S035C): capacitive chip on I2C SDA33 SCL32 RST25 INT21.
constexpr int I2C_SDA = 33, I2C_SCL = 32, I2C_RST = 25, I2C_INT = 21;
constexpr uint8_t ADDR_CST = 0x15, ADDR_FT = 0x38, ADDR_GT_A = 0x5D, ADDR_GT_B = 0x14, ADDR_NS = 0x48;

TouchConfig xptBitBang() {
  TouchConfig c;
  c.chip = Chip::Xpt2046; c.bus = Bus::BitBangSpi;
  c.cs = 33; c.clk = 25; c.mosi = 32; c.miso = 39; c.irq = 36;
  return c;
}

TouchConfig xptShared() {
  TouchConfig c;
  c.chip = Chip::Xpt2046; c.bus = Bus::SharedSpi;
  c.cs = 33; c.clk = TFT_SCLK; c.mosi = TFT_MOSI; c.miso = 12; c.irq = 36;
  return c;
}

TouchConfig i2cBase(Chip chip, uint8_t addr) {
  TouchConfig c;
  c.chip = chip; c.bus = Bus::I2c;
  c.sda = I2C_SDA; c.scl = I2C_SCL; c.rst = I2C_RST; c.intr = I2C_INT; c.addr = addr;
  return c;
}

// ---------------------------------------------------------------------------------------------------------------
// Pin ownership: only one wiring drives the shared GPIOs (33/32/25) at a time.
bool sameWiring(const TouchConfig& a, const TouchConfig& b) {
  return a.bus == b.bus && a.cs == b.cs && a.clk == b.clk && a.mosi == b.mosi && a.miso == b.miso &&
         a.sda == b.sda && a.scl == b.scl;
}

TouchConfig wired;  // what the pins are set up for now
bool anyWired = false, wireOn = false;

void releaseI2c() {
  if (wireOn) { Wire.end(); wireOn = false; }
}

SPIClass& tftSpi() { return TFT_eSPI::getSPIinstance(); }

void setupWiring(const TouchConfig& c) {
  if (anyWired && sameWiring(c, wired)) return;
  if (c.bus != Bus::I2c) releaseI2c();
  switch (c.bus) {
    case Bus::BitBangSpi:
      pinMode(c.cs, OUTPUT); digitalWrite(c.cs, HIGH);
      pinMode(c.clk, OUTPUT); digitalWrite(c.clk, LOW);
      pinMode(c.mosi, OUTPUT); digitalWrite(c.mosi, LOW);
      pinMode(c.miso, INPUT);
      break;
    case Bus::SharedSpi:
      // The panel's bus is set up without MISO (TFT_eSPI only writes); route the touch chip's DOUT in.
      pinMode(c.cs, OUTPUT); digitalWrite(c.cs, HIGH);
      spiAttachMISO(tftSpi().bus(), c.miso);
      break;
    case Bus::I2c:
      releaseI2c();
      wireOn = Wire.begin(c.sda, c.scl, 400000);
      Wire.setTimeOut(20);
      break;
  }
  if (c.irq >= 0) pinMode(c.irq, INPUT);
  wired = c;
  anyWired = true;
}

// ---------------------------------------------------------------------------------------------------------------
// XPT2046. Both readers return the same "normalised" layout: 12 data bits << 4 | trailing bits.

// Bit-banged with ~1 us clock: proven on both of our CYDs (TFT_Touch's unthrottled bit-bang fails on one).
uint16_t bbRaw16(const TouchConfig& c, uint8_t cmd) {
  digitalWrite(c.cs, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(c.mosi, (cmd >> i) & 1);
    delayMicroseconds(1); digitalWrite(c.clk, HIGH); delayMicroseconds(1); digitalWrite(c.clk, LOW);
  }
  uint16_t v = 0;
  for (int i = 0; i < 16; i++) {
    delayMicroseconds(1); digitalWrite(c.clk, HIGH); delayMicroseconds(1); digitalWrite(c.clk, LOW);
    v = (v << 1) | digitalRead(c.miso);
  }
  digitalWrite(c.cs, HIGH);
  return v;
}

// On the panel's bus, between TFT_eSPI transactions (it leaves the bus in full-duplex mode after each one).
uint16_t hwRaw16(const TouchConfig& c, uint8_t cmd) {
  SPIClass& spi = tftSpi();
  spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(c.cs, LOW);
  spi.transfer(cmd);
  uint16_t v = spi.transfer16(0);  // busy bit, 12 data bits, 3 zeros
  digitalWrite(c.cs, HIGH);
  spi.endTransaction();
  return (uint16_t)(v << 1);
}

uint16_t xptRaw16(const TouchConfig& c, uint8_t cmd) {
  return c.bus == Bus::SharedSpi ? hwRaw16(c, cmd) : bbRaw16(c, cmd);
}
uint16_t xpt12(const TouchConfig& c, uint8_t cmd) { return xptRaw16(c, cmd) >> 4; }

int xptMedian5(const TouchConfig& c, uint8_t cmd) {
  int s[5];
  for (int& v : s) v = xpt12(c, cmd);
  return logic::median(s, 5);
}

constexpr uint8_t XPT_Z1 = 0xB1, XPT_X = 0x91, XPT_Y = 0xD1, XPT_IDLE = 0xD0;  // 0xD0: power down, PENIRQ on
constexpr int XPT_PRESSED_Z1 = 150;                                           // idle reads ~0

// Temperature conversion, first against the board's VREF, then the internal 2.5 V reference (for boards that
// leave VREF floating; the internal reference may be overdriven by an external one, so this is harmless).
bool xptAlive(const TouchConfig& c, int* tempOut) {
  setupWiring(c);
  bool alive = false;
  for (uint8_t cmd : {(uint8_t)0x84, (uint8_t)0x87}) {
    uint16_t s[8];
    xptRaw16(c, cmd);  // first conversion after a reference change settles
    for (auto& v : s) v = xptRaw16(c, cmd);
    if (tempOut) *tempOut = s[0] >> 4;
    if (logic::xptLooksAlive(s, 8)) { alive = true; break; }
  }
  xptRaw16(c, XPT_IDLE);
  return alive;
}

bool xptRead(const TouchConfig& c, RawTouch& t) {
  setupWiring(c);
  t.z = xpt12(c, XPT_Z1);
  t.down = false;
  if (t.z < XPT_PRESSED_Z1) return true;
  t.x = xptMedian5(c, XPT_X);
  t.y = xptMedian5(c, XPT_Y);
  // Lifted while sampling (half-way readings), or the readings are pinned to the rails (not a real press).
  t.down = xpt12(c, XPT_Z1) >= XPT_PRESSED_Z1 && t.x > 40 && t.x < 4055 && t.y > 40 && t.y < 4055;
  return true;
}

// ---------------------------------------------------------------------------------------------------------------
// I2C helpers
bool ack(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool rd8(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)n) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool rd16(uint8_t addr, uint16_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg >> 8); Wire.write(reg & 0xFF);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)n) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool wr16(uint8_t addr, uint16_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg >> 8); Wire.write(reg & 0xFF); Wire.write(val);
  return Wire.endTransmission() == 0;
}

// NS2009 / TSC2007-style resistive controller on I2C: command byte in, 12-bit result out.
int nsRead(uint8_t addr, uint8_t cmd) {
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom(addr, (uint8_t)2) != 2) return -1;
  uint8_t hi = Wire.read(), lo = Wire.read();
  return (hi << 4) | (lo >> 4);
}

// GT911 only reports when it has news: keep the last state between reports.
struct GtState { bool down = false; int x = 0, y = 0; } gt;

bool i2cRead(const TouchConfig& c, RawTouch& t) {
  setupWiring(c);
  t.down = false;
  uint8_t d[6];
  switch (c.chip) {
    case Chip::Cst8xx: {  // 0x01 gesture, 0x02 fingers, 0x03.. XH XL YH YL
      if (!rd8(c.addr, 0x01, d, 6)) return false;
      int n = d[1] & 0x0F;
      t.z = n;
      t.down = n > 0 && n <= 2;
      t.x = ((d[2] & 0x0F) << 8) | d[3];
      t.y = ((d[4] & 0x0F) << 8) | d[5];
      return true;
    }
    case Chip::Ft6x36: {  // 0x02 fingers, 0x03.. XH(event in bits 7:6) XL YH YL
      if (!rd8(c.addr, 0x02, d, 5)) return false;
      int n = d[0] & 0x0F, event = d[1] >> 6;
      t.z = n;
      t.down = n > 0 && n <= 2 && event != 1;  // event 1 = lift-up
      t.x = ((d[1] & 0x0F) << 8) | d[2];
      t.y = ((d[3] & 0x0F) << 8) | d[4];
      return true;
    }
    case Chip::Gt911: {  // 0x814E status (bit7 = buffer ready, low nibble = points), point 1 at 0x8150
      uint8_t st;
      if (!rd16(c.addr, 0x814E, &st, 1)) return false;
      if (st & 0x80) {
        int n = st & 0x0F;
        if (n > 0 && n <= 5 && rd16(c.addr, 0x8150, d, 4)) {
          gt.down = true;
          gt.x = d[0] | (d[1] << 8);
          gt.y = d[2] | (d[3] << 8);
        } else {
          gt.down = false;
        }
        wr16(c.addr, 0x814E, 0);
      }
      t.down = gt.down; t.x = gt.x; t.y = gt.y; t.z = gt.down;
      return true;
    }
    case Chip::Ns2009: {
      int z = nsRead(c.addr, 0xE0);
      if (z < 0) return false;
      t.z = z;
      if (z < 100) return true;
      t.x = nsRead(c.addr, 0xC0);
      t.y = nsRead(c.addr, 0xD0);
      t.down = t.x > 0 && t.y > 0;
      return true;
    }
    default:
      return false;
  }
}

void logf(Log log, const char* fmt, ...) {
  char line[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof line, fmt, ap);
  va_end(ap);
  if (!log) return;  // background re-probes stay quiet
  Serial.println(line);
  log(line);
}

void resetI2cChip() {
  pinMode(I2C_RST, OUTPUT);
  digitalWrite(I2C_RST, LOW);
  delay(10);
  digitalWrite(I2C_RST, HIGH);
  delay(150);  // GT911 needs >= 50 ms, CST8xx ~100 ms before it answers
}

int probeI2c(TouchConfig* out, int max, Log log) {
  int n = 0;
  resetI2cChip();  // also wakes a CST816 that went to sleep (it then stops ACKing until touched)
  TouchConfig base = i2cBase(Chip::None, 0);
  anyWired = false;
  setupWiring(base);
  if (!wireOn) { logf(log, "I2C SDA%d/SCL%d: bus init failed", I2C_SDA, I2C_SCL); return 0; }

  char others[48] = "";
  size_t olen = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (!ack(a)) continue;
    TouchConfig c;
    if (a == ADDR_CST) {
      c = i2cBase(Chip::Cst8xx, a);
      rd8(a, 0xA7, c.id, 1);   // chip id: B4 CST816S, B5 CST816T, B6 CST816D, B7 CST820
      rd8(a, 0xA9, c.id + 1, 1);  // firmware version
    } else if (a == ADDR_FT) {
      c = i2cBase(Chip::Ft6x36, a);
      rd8(a, 0xA3, c.id, 1);   // chip id: 06 FT6206, 36 FT6236, 64 FT6336U
      rd8(a, 0xA8, c.id + 1, 1);  // vendor id (0x11 = FocalTech)
    } else if (a == ADDR_GT_A || a == ADDR_GT_B) {
      c = i2cBase(Chip::Gt911, a);
      rd16(a, 0x8140, c.id, 4);  // product id, ASCII ("911")
    } else if (a == ADDR_NS) {
      c = i2cBase(Chip::Ns2009, a);
    } else {
      if (olen < sizeof others - 6) olen += snprintf(others + olen, sizeof others - olen, " 0x%02X", a);
      continue;
    }
    if (n < max) out[n++] = c;
    char chip[40];
    describeChip(c, chip, sizeof chip);
    logf(log, "I2C: found %s", chip);
  }
  if (olen) logf(log, "I2C: other devices at%s", others);
  if (!n) logf(log, "I2C SDA%d/SCL%d: no touch chip", I2C_SDA, I2C_SCL);
  return n;
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------
const char* chipName(Chip c) {
  switch (c) {
    case Chip::Xpt2046: return "XPT2046";
    case Chip::Cst8xx: return "CST816/CST820";
    case Chip::Ft6x36: return "FT6x36";
    case Chip::Gt911: return "GT911";
    case Chip::Ns2009: return "NS2009/TSC2007";
    default: return "none";
  }
}

const char* chipKind(Chip c) {
  return (c == Chip::Xpt2046 || c == Chip::Ns2009) ? "resistive" : "capacitive";
}

void describeChip(const TouchConfig& c, char* buf, size_t n) {
  switch (c.chip) {
    case Chip::Cst8xx:
    case Chip::Ft6x36:
      snprintf(buf, n, "%s (id 0x%02X) @0x%02X", chipName(c.chip), c.id[0], c.addr);
      break;
    case Chip::Gt911: {
      char pid[5] = {0};
      for (int i = 0; i < 4; i++) pid[i] = isprint(c.id[i]) ? (char)c.id[i] : 0;
      snprintf(buf, n, "GT%s @0x%02X", pid[0] ? pid : "911", c.addr);
      break;
    }
    case Chip::Ns2009:
      snprintf(buf, n, "%s @0x%02X", chipName(c.chip), c.addr);
      break;
    default:
      snprintf(buf, n, "%s", chipName(c.chip));
  }
}

void describePins(const TouchConfig& c, char* buf, size_t n) {
  switch (c.bus) {
    case Bus::BitBangSpi:
      snprintf(buf, n, "own SPI CS=%d CLK=%d DIN=%d DOUT=%d IRQ=%d", c.cs, c.clk, c.mosi, c.miso, c.irq);
      break;
    case Bus::SharedSpi:
      snprintf(buf, n, "TFT's SPI CS=%d CLK=%d DIN=%d DOUT=%d IRQ=%d", c.cs, c.clk, c.mosi, c.miso, c.irq);
      break;
    case Bus::I2c:
      snprintf(buf, n, "I2C SDA=%d SCL=%d RST=%d INT=%d", c.sda, c.scl, c.rst, c.intr);
      break;
  }
}

int candidates(TouchConfig* out, int max) {
  const TouchConfig all[] = {xptBitBang(), xptShared(), i2cBase(Chip::Cst8xx, ADDR_CST), i2cBase(Chip::Ft6x36, ADDR_FT),
                             i2cBase(Chip::Gt911, ADDR_GT_A), i2cBase(Chip::Gt911, ADDR_GT_B)};
  int n = 0;
  for (const auto& c : all)
    if (n < max) out[n++] = c;
  return n;
}

int probeAll(TouchConfig* out, int max, Log log) {
  // I2C first: the bit-bang probe would wiggle the same pins (33/32/25) on a capacitive board.
  int n = probeI2c(out, max, log);
  releaseI2c();
  anyWired = false;

  for (const TouchConfig& c : {xptBitBang(), xptShared()}) {
    int temp = 0;
    bool alive = xptAlive(c, &temp);
    logf(log, "XPT2046 %s: %s (temp raw %d)", c.bus == Bus::SharedSpi ? "on TFT bus" : "own pins",
         alive ? "found" : "no answer", temp);
    if (alive && n < max) out[n++] = c;
  }
  return n;
}

bool read(const TouchConfig& c, RawTouch& t) {
  if (c.chip == Chip::Xpt2046) return xptRead(c, t);
  return i2cRead(c, t);
}

bool present(const TouchConfig& c) {
  if (c.chip == Chip::Xpt2046) return xptAlive(c, nullptr);
  if (c.bus != Bus::I2c) return false;
  setupWiring(c);
  if (wireOn && ack(c.addr)) return true;
  resetI2cChip();  // a sleeping CST816 only answers after a reset or a touch
  return ack(c.addr);
}

}  // namespace tf
