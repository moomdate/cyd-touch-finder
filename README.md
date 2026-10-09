# CYD Touch Finder

Flash it onto any "Cheap Yellow Display" (ESP32-2432S028R and friends) and the screen tells you **which touch
controller the board has and on which pins**, walks you through calibration, and prints copy-paste config for
your own projects.

แฟลชลงบอร์ด CYD แล้วจอจะบอกเลยว่าใช้ชิปทัชอะไร ต่อขาไหน จากนั้นพาคาลิเบรต แล้วให้ลองวาดบนจอว่ากดได้จริง
พร้อมพิมพ์ค่าที่ต้องใช้ในโปรเจคอื่นออกทาง Serial

## Detects

| Chip | Wiring | Typical boards |
|---|---|---|
| XPT2046 (resistive) | own pins: CS 33, CLK 25, DIN 32, DOUT 39, IRQ 36 | ESP32-2432S028R, CYD2USB, most 2.8" "R" |
| XPT2046 (resistive) | shares the TFT SPI bus: CS 33, CLK 14, DIN 13, DOUT 12, IRQ 36 | 2432S024R, 2432S032R, 3248S035R |
| CST816 / CST820 (capacitive) | I2C 0x15: SDA 33, SCL 32, RST 25, INT 21 | 2432S024C, JC2432W328C |
| FT6x36 (capacitive) | I2C 0x38, same pins | |
| GT911 (capacitive) | I2C 0x5D / 0x14, same pins | 3248S035C, 2432S032C |
| NS2009 / TSC2007 (resistive) | I2C 0x48, same pins | |

If nothing answers the passive probe, it polls every known wiring live: press the screen and the row that turns
green is your touch.

## Flash

**Easiest:** grab `cyd-touch-finder-cyd.bin` from Releases (or build it) and write it at address **`0x0`**:

- Browser: https://espressif.github.io/esptool-js/ → Connect → address `0x0` → Program
- CLI: `esptool.py --chip esp32 --baud 460800 write_flash 0x0 cyd-touch-finder-cyd.bin`

**From source** (PlatformIO):

```bash
pio run -t upload        # builds .pio/build/cyd/cyd-touch-finder-cyd.bin too (merged, flash at 0x0)
pio device monitor       # 115200: full report
pio test -e native       # host tests for the calibration / detection logic
```

`env:cyd-st7789` is a fallback for boards whose ST7789 panel stays blank with the default ILI9341 driver (untested).

## Use

1. Power on: the probe log shows what answered.
2. **Press & hold** the screen: that confirms the wiring.
3. Touch the 5 `+` targets (4 corners, then the centre check).
4. Draw on the screen to test. Buttons: Recalibrate / Rescan / Clear.

- **BOOT button:** toggles colour inversion (if black looks white); remembered.
- **Hold BOOT while powering on:** forget the saved result and probe again.

## Serial report (115200)

```
chip : XPT2046 (resistive)
pins : own SPI CS=33 CLK=25 DIN=32 DOUT=39 IRQ=36
read : rx = cmd 0x91, ry = cmd 0xD1, pressed when Z1 (cmd 0xB1) > 150, ~1 MHz clock
calibration (raw -> screen):
  sx = a*rx + b*ry + c
  sy = d*rx + e*ry + f
  map() form:
    #define TOUCH_SWAP_XY 0
    #define TOUCH_X_MIN ...
    ...
```

Calibration is for `tft.setRotation(1)` (320×240 landscape).

## Notes

- The XPT2046 is bit-banged with ~1 µs clock delays. TFT_Touch's unthrottled bit-bang fails on some boards
  (presses never register), which is the usual reason "touch doesn't work on some CYDs".
- XPT2046 presence is detected without a touch by reading the chip's internal temperature channel: a real chip
  gives a steady mid-range value followed by its trailing zero bits; a floating line doesn't.
