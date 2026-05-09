# Waveshare ESP32-S3-Touch-LCD-7 — Status

## Hardware

- **MCU**: ESP32-S3R8 (16MB flash, 8MB PSRAM octal)
- **LCD**: ST7262 RGB 800×480 16-bit, driven via `esp_lcd_new_rgb_panel()` with bounce buffers
- **Touch**: GT911 I2C (addr 0x5D), read via `lgfx::i2c::transactionWriteRead()`
- **IO Expander**: CH422G I2C (addr 0x20/0x24/0x38). Pin assignments:
  - IO1 = TP_RST, IO2 = BL (backlight), IO3 = LCD_RST, IO4 = SD_CS, IO5 = USB_SEL
- **SD Card**: SPI (MOSI=11, SCK=12, MISO=13), CS via CH422G IO4 only (no direct GPIO)
- **LoRa/GPS**: On C3 co-processor (dual-MCU). S3 pins defined but not connected. `LORA_ON_C3` flag skips local radio init.

## What works

| Feature | Status | Notes |
|---------|--------|-------|
| Display (RGB panel) | ✅ | `esp_lcd_new_rgb_panel()` with 800×10 bounce buffers |
| Touch (GT911) | ✅ | Via lgfx::i2c |
| LVGL UI | ✅ | Full-frame buffer (800×480, 750KB PSRAM), full_refresh=1 |
| SD card mount | ✅ | ESP-IDF sdspi driver, gpio_cs=-1, CH422G CS preamble |
| SD frames loading | ✅ | 20 frames loaded at boot |
| SD config/stats | ✅ | JSON read/write via VFS |
| SD logger | ✅ | GPS trace logging works |
| Raster tiles (PNG/JPG) | ✅ | Load from SD, correct colors |
| Dashboard/Messages/Settings | ✅ | Swipe Dashboard↔Settings |
| Brightness control | ⚠️ | CH422G digital only (on/off), no PWM |

## Known issues

| Issue | Symptom | Suspected cause |
|-------|---------|-----------------|
| NAV false colors | Roads/water in wrong colors | Viewport sprite setSwapBytes mismatch with LVGL canvas |
| Map flickering on zoom | Screen trembles during zoom change | Double-buffer sprite swap timing vs LVGL refresh |

## Root causes fixed

1. **DMA underrun / display corruption**: LovyanGFX Bus_RGB reads PSRAM directly → replaced with `esp_lcd_new_rgb_panel()` + bounce buffers
2. **SD card deselected after boot**: `ch422g_backlight_on()` overwrote bit4 (SD_CS) to HIGH → changed to `\|= (1<<2)` to preserve SD_CS
3. **SD card timeout/crash (0x107)**: Arduino SD lib with CS=-1 via `digitalWrite(255)` no-ops → ESP-IDF `esp_vfs_fat_sdspi_mount` with gpio_cs=-1 + manual 74-clock preamble
4. **Map at 2.8" (42% of 7")**: `MAP_VISIBLE_HEIGHT` hardcoded to 200px → 420px for Waveshare
5. **LoRa SPI conflict with SD**: `SPI.begin(RADIO_SCLK,...)` reconfigures SD bus → skip when `LORA_ON_C3`
6. **spiMutex type mismatch**: `xSemaphoreTake` on recursive mutex → `xSemaphoreTakeRecursive`
7. **Partial refresh tearing**: Half-height buffer (800×240) with `full_refresh=0` → full-frame (800×480) with `full_refresh=1`

## Key files

- `variants/waveshare_s3_touch_lcd_7/board_pinout.h` — pin definitions
- `variants/waveshare_s3_touch_lcd_7/platformio.ini` — build flags
- `include/ch422g.h` — CH422G I2C expander driver
- `include/waveshare_lcd.h` / `src/waveshare_lcd.cpp` — RGB panel + GT911 touch
- `include/ui_map_manager.h` — map dimensions per board
- `src/storage_utils.cpp` — SD init (Waveshare uses ESP-IDF sdspi path)
- `src/lvgl_ui.cpp` — LVGL init, flush callback
- `src/lora_utils.cpp` — skip LoRa/GPS init when LORA_ON_C3
- `src/map/map_engine.cpp`, `map_tiles.cpp`, `ui_map_manager.cpp` — sprites, tiles, canvas
