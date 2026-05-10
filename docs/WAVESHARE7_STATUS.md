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
| Raster tiles (PNG/JPG) | ✅ | Correct colors: LE PNG via getBuffer bypass |
| NAV tiles (vector) | ✅ | Correct colors: viewport+glyph sprites rgb565_nonswapped |
| Dashboard/Messages/Settings | ✅ | Swipe Dashboard↔Settings |
| Brightness control | ⚠️ | CH422G digital only (on/off), no PWM |
| Double-buffer RGB panel | ✅ | num_fbs=2, VSYNC callback registered |
| Map debug logging | ✅ | scrollMap, applyViewport, canvas position |

## Known issues

| Issue | Symptom | Suspected cause |
|-------|---------|-----------------|
| Lack of fluidity (UI globally and map pan/zoom) | All screens stutter, map pan ≈ 5 fps | LVGL produces only 5 frames/s during interactive pan despite `lv_timer_handler()` being called 500×/s (main loop runs at 2 ms/iter). LVGL render time itself reports 0 ms, but the gap between two flushes is ~240 ms. Cause is not in our code but in the LVGL ↔ esp_lcd_rgb pipeline — investigation ongoing on the `feature/rgb-native-refactor` branch. Latest hypothesis: `direct_mode = 1` combined with `num_fbs = 2` introduces a long inter-frame wait. |

## Refactor in progress: `feature/rgb-native-refactor` branch

Branch goal: stop layering patches on a SPI-era architecture. The previous design (sprite back/front + LVGL PSRAM buffer + flush copy + hardware double-fb) was 4 logical layers; on RGB the intermediate copies become visible artefacts. The refactor introduces a thin display HAL so the application stays platform-agnostic and only one file per target carries hardware-specific code.

**Phase 1 (done, on branch, not yet validated):**

- New `include/display_hal.h` interface (init, readTouch).
- New `src/display/display_hal_waveshare_rgb.cpp` — RGB panel init + LVGL display registered with hardware framebuffers as direct draw buffers (no separate PSRAM LVGL buffer, no flush_cb copy).
- `src/lvgl_ui.cpp` Waveshare path now calls `DisplayHAL::init()` instead of inline panel/LVGL setup. Other targets (T-Deck Plus, Crowpanel) untouched.
- `variants/waveshare_s3_touch_lcd_7/platformio.ini` excludes legacy `src/waveshare_lcd.cpp` from the build (its content migrated into the HAL file).
- Diagnostic instrumentation: `flush_cb` reports frames/s, draw time, gap; `monitor_cb` reports LVGL render time and pixel count; `lv_obj_set_pos(map_canvas)` reports drag rate; main loop reports iteration time.
- Sprite ping-pong (`swapViewportSprites`) replaces the 101 ms PSRAM `copyBackToFront` in the pan path. Confirmed effective (~500 µs) but did not fix the secousses on its own.

**Findings so far on the branch:**

- The 101 ms PSRAM copy was real but not the dominant cause.
- The LVGL flush copy was real but not the dominant cause either.
- The dominant cause is the LVGL frame cadence itself — only 5 frames/s during a pan, despite LVGL being polled at 500 Hz and reporting 0 ms render time. The wait happens between two complete frames, not inside one frame.
- Hypothesis under test: `direct_mode = 1` + `num_fbs = 2` combination. Toggle pending hardware validation.

**Files to delete after validation:**

- `src/waveshare_lcd.cpp` and `include/waveshare_lcd.h` — content fully migrated to the HAL, file already excluded from the build.

## Investigated and ruled out

| Hypothesis | Test | Result |
|-----------|------|--------|
| Clamp blocks tile shift | Raised clamp to PAN_TILE_THRESHOLD | No effect — tile shifts confirmed in logs |
| resetZoom clears offset too early | Deferred to applyRenderedViewport | No effect |
| VSYNC semaphore timeout | Removed `waveshare_wait_vsync()` | No effect |
| 1-frame offset/content mismatch | `lv_obj_set_pos` in applyRenderedViewport | No effect |
| Stale content drifts during render | Freeze canvas pos when `redraw_in_progress` | No effect |
| NAV false colors | `rgb565_nonswapped` on viewport + glyph sprites | ✅ Fixed |
| Raster false colors | Reverted tile cache sprites to default `rgb565_2Byte` (PNG writes LE directly via getBuffer, bypassing LGFX) | ✅ Fixed |
| GPS error spam | `#if !defined(WAVESHARE_S3_TOUCH_LCD_7)` guard | ✅ Fixed |

## Root causes fixed

1. **DMA underrun / display corruption**: LovyanGFX Bus_RGB reads PSRAM directly → replaced with `esp_lcd_new_rgb_panel()` + bounce buffers
2. **SD card deselected after boot**: `ch422g_backlight_on()` overwrote bit4 (SD_CS) to HIGH → changed to `\|= (1<<2)` to preserve SD_CS
3. **SD card timeout/crash (0x107)**: Arduino SD lib with CS=-1 via `digitalWrite(255)` no-ops → ESP-IDF `esp_vfs_fat_sdspi_mount` with gpio_cs=-1 + manual 74-clock preamble
4. **Map at 2.8" (42% of 7")**: `MAP_VISIBLE_HEIGHT` hardcoded to 200px → 420px for Waveshare
5. **LoRa SPI conflict with SD**: `SPI.begin(RADIO_SCLK,...)` reconfigures SD bus → skip when `LORA_ON_C3`
6. **spiMutex type mismatch**: `xSemaphoreTake` on recursive mutex → `xSemaphoreTakeRecursive`
7. **Partial refresh tearing**: Half-height buffer (800×240) with `full_refresh=0` → full-frame (800×480) with `full_refresh=1`
8. **NAV false colors**: Viewport + glyph sprites `rgb565_nonswapped` when `LV_COLOR_16_SWAP=0`. NAV colors are LE in file, must match.
9. **Raster false colors**: Tile cache sprites kept at default `rgb565_2Byte`. PNG decoder writes LE via `getBuffer()` directly — bypasses LGFX color conversion.
10. **GPS error spam on Waveshare**: C3 handles GPS, S3 sees no frames. Guarded `ESP_LOGE` with `#if !defined(WAVESHARE_S3_TOUCH_LCD_7)`.
11. **RGB panel tearing**: Added `num_fbs=2` + VSYNC event callback. Panel double-buffers at hardware level.
12. **Pan clamp prevents tile shift**: `MAP_MARGIN_X = -16` (negative!) caused clamp to use `PAN_TILE_THRESHOLD-1`, blocking tile shifts on X axis. Changed fallback to `PAN_TILE_THRESHOLD`.
13. **resetZoom offset jump**: Removed immediate offsetX/Y=0 in resetZoom(), deferred to applyRenderedViewport().
14. **VSYNC wait removed**: `waveshare_wait_vsync()` in flush callback was redundant (draw_bitmap blocks internally) and could time out at 100ms.
15. **Canvas position synced with content**: applyRenderedViewport() now calls lv_obj_set_pos immediately after offset recalculation.

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
