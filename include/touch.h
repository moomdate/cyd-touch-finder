// Touch controllers found on CYD-family boards, how to probe for them, and how to read them.
#pragma once
#include <Arduino.h>

namespace tf {

enum class Chip : uint8_t { None = 0, Xpt2046, Cst8xx, Ft6x36, Gt911, Ns2009 };
enum class Bus : uint8_t { BitBangSpi = 0, SharedSpi, I2c };

// Stored as a blob in NVS: bump `version` when the layout changes.
struct TouchConfig {
  uint8_t version = 2;
  Chip chip = Chip::None;
  Bus bus = Bus::BitBangSpi;
  int8_t cs = -1, clk = -1, mosi = -1, miso = -1, irq = -1;  // SPI (XPT2046)
  int8_t sda = -1, scl = -1, rst = -1, intr = -1;            // I2C
  uint8_t addr = 0;
  uint8_t id[4] = {0, 0, 0, 0};  // chip id bytes as read during the probe
};

struct RawTouch {
  bool down = false;
  int x = 0, y = 0, z = 0;  // raw controller units; z = pressure (XPT/NS2009) or finger count (capacitive)
};

constexpr int MAX_FOUND = 6;
using Log = void (*)(const char* line);

const char* chipName(Chip c);
const char* chipKind(Chip c);                        // "resistive" / "capacitive"
void describeChip(const TouchConfig& c, char* buf, size_t n);  // e.g. "GT911 (id 911) @0x5D"
void describePins(const TouchConfig& c, char* buf, size_t n);  // e.g. "I2C SDA=33 SCL=32 RST=25 INT=21"

// Every wiring a CYD-family board is known to use; the probes and the "touch anywhere" fallback walk this list.
int candidates(TouchConfig* out, int max);

// Passive probe (no finger needed). Fills `out` with what answered; `log` (may be null) gets one line per step.
int probeAll(TouchConfig* out, int max, Log log);

// Reads one sample from `c`, switching the pins over to it first if another config was in use.
// False on a bus error (chip asleep, wrong pins): treat as "not touched".
bool read(const TouchConfig& c, RawTouch& t);

// Quick "is it still there" check for a saved config at boot.
bool present(const TouchConfig& c);

}  // namespace tf
