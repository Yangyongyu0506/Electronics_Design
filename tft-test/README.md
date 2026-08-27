# tft-test

SPI color display test: drives the LQ_TFT18SPIV33 (1.8" 128x160 TFT, ST7735) and shows text with an embedded 5x7 font.

## Wiring

| Display | Board |
|---------|-------|
| D/C     | GPIO39 |
| SDI     | GPIO47 |
| SCK     | GPIO21 |
| RST     | GPIO48 |
| CS      | GND |

Wiring matches the table in the repository root `README.md` (`LQ_TFT18SPIV33彩屏`); RST is additionally required (many LQ 1.8" modules have no onboard reset pull-up — a floating RST holds the panel in reset = permanent white screen).

## Implementation

- No managed components: uses IDF `esp_lcd` panel-io (SPI, hardware SPI2 with DC handled automatically) plus a hand-written ST7735 init sequence (green-tab, RGB565, MADCTL `0xC0` = portrait).
- Full-frame RGB565 framebuffer in internal RAM (40 KB), redrawn once per second (`CASET`/`RASET`/`RAMWR`).
- Font: embedded 5x7 ASCII table (0x20-0x7E), advance 6 px.

## Tuning knobs (`main/tft-test.c`)

- `LCD_MADCTL` (0xC0): if the picture is mirrored/rotated try `0x00`, `0xC8`, `0xA0`.
- Red/blue swapped: set MADCTL bit 3 (e.g. `0xC8`) or change `LCD_RGB_ELEMENT_ORDER`-equivalent by swapping colors in the test strings.

## Build & flash

```sh
source ~/.espressif/v5.4.4/esp-idf/export.sh
idf.py build
idf.py flash monitor
```
