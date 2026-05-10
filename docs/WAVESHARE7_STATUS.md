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
| Map pan/zoom stutter | ~5 fps during drag, screen trembles | PSRAM bandwidth shared between LVGL rendering and DMA scan-out. `full_refresh=1` forces a full 800×480 re-render on every touch event (~168 ms). Investigation on `feature/rgb-native-refactor`. |

## Refactor in progress: `feature/rgb-native-refactor` branch

### Objective

Stop layering SPI-era patches onto RGB hardware. The previous design (sprite back/front + LVGL PSRAM buffer + flush copy + hardware double-fb) was 4 logical layers. The refactor introduces a display HAL so the app stays platform-agnostic.

### Commits on the branch

1. `bded011d` — Sprite ping-pong (swapViewportSprites, ~500 µs) replaces 101 ms copyBackToFront. Did not fix stutter alone.
2. `9ece8c42` — Display HAL interface + Waveshare RGB implementation. LVGL no longer allocates a separate PSRAM buffer; flush_cb copy eliminated. Other targets untouched.
3. Current working state (uncommitted) — Switched from `full_refresh=1` + `direct_mode=1` + full-frame hw draw buffers to `full_refresh=0` + partial draw buffers (800×48 ×2) + `num_fbs=2`. Strip-based rendering instead of blocking full-frame cycles.

### Configuration evolution (Waveshare)

| Attempt | num_fbs | full_refresh | direct_mode | draw bufs | Result |
|---------|---------|-------------|-------------|-----------|--------|
| Legacy (SPI-era) | 2 | 1 | 0 | 1× PSRAM 750KB, buf2=null | Copy PSRAM→fb, 101 ms copyBackToFront |
| HAL v1 | 2 | 1 | 1 | hw fb[0], fb[1] (750KB each) | No copy, but LVGL blocks 110 ms/frame waiting for fb |
| HAL v2 | 2 | 1 | 0 | hw fb[0], fb[1] | Tearing whole screen |
| HAL v3 | 3 | 1 | 1 | hw fb[0], fb[1] | `lv_timer_handler` still blocks 110 ms — extra fb not visible to LVGL |
| HAL v4 (current) | 2 | 0 | 0 | 2× 800×48 PSRAM (75KB each) | Strips at 20ms gap, 36-39 strips/s. Pending hardware test. |

### Key finding

`full_refresh=1` was the root bottleneck. LVGL re-blits the entire 768×768 map canvas into the 800×480 fb on every touch event. At ~30 MB/s effective PSRAM bandwidth (shared with 46 MB/s DMA scan-out), this takes ≈168 ms → 5 fps. The `monitor_cb` reported 0 ms because it measures LVGL's internal dispatch time, not the accumulated wait on the shared PSRAM bus.

With `full_refresh=0` + partial buffers (48-line strips), LVGL only touches the dirty area. 39 strips/s at 6ms per strip = the same total PSRAM work, but broken into non-blocking chunks. Combined with `num_fbs=2` for atomic VSYNC swap → no tearing during strip accumulation.

### Files to delete after validation

- `src/waveshare_lcd.cpp` and `include/waveshare_lcd.h` — content migrated to HAL, excluded from build.

## Investigated and ruled out

| Hypothesis | Test | Result |
|-----------|------|--------|
| PSRAM copy back→front (101 ms) | Ping-pong swap (~500 µs) | ✅ Fixed, not dominant |
| LVGL→fb flush copy | HAL direct mode (hw fb as draw buf) | ✅ Fixed, not dominant |
| Main loop too slow | Measured: 500 iters/s, avg 2 ms | ✅ Ruled out |
| `lv_timer_handler` blocked by hw | Per-call timing: 110-170 ms spikes | Confirmed — internal LVGL/esp_lcd wait |
| `num_fbs=3` would fix hw wait | Tested: same 110 ms block, 3rd fb invisible to LVGL | ❌ Ruled out |
| `direct_mode=0` + `num_fbs=2` | Tested: whole-screen tearing | ❌ Worse |
| `num_fbs=1` + partial buffers | Tested: 39 strips/s but tearing (no atomic swap) | ❌ Tearing |
| `full_refresh=1` bottleneck | Confirmed: 168 ms/frame = shared PSRAM bus saturation | ✅ Root cause found |
| Clamp blocks tile shift | Raised clamp to PAN_TILE_THRESHOLD | No effect |
| resetZoom clears offset too early | Deferred to applyRenderedViewport | No effect |
| VSYNC semaphore timeout | Removed `waveshare_wait_vsync()` | No effect |
| 1-frame offset/content mismatch | `lv_obj_set_pos` in applyRenderedViewport | No effect |
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
