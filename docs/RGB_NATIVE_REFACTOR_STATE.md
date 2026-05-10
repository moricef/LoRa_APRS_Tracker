# RGB Native Refactor — State at commit 51c8d1c3

## Branch: feature/rgb-native-refactor

### What this branch does

Introduces a display HAL so each target gets its own hardware init file, selected by build_src_filter in platformio.ini. Waveshare 7" RGB gets a completely rewritten init path. Other targets (T-Deck Plus SPI, Crowpanel) untouched.

### Files created

- `include/display_hal.h` — HAL interface (init, readTouch)
- `src/display/display_hal_waveshare_rgb.cpp` — Waveshare impl (panel init + LVGL reg + GT911)

### Files modified

- `src/lvgl_ui.cpp` — Waveshare path calls DisplayHAL::init() instead of inline code. flush_cb and touch wrapped #if !WAVESHARE
- `variants/waveshare_s3_touch_lcd_7/platformio.ini` — build_src_filter excludes legacy src/waveshare_lcd.cpp
- `docs/WAVESHARE7_STATUS.md` — full diagnostic history

### Files excluded from build (content migrated to HAL)

- `src/waveshare_lcd.cpp` and `include/waveshare_lcd.h` — kept on disk, excluded by build_src_filter

### Other changes on the branch

- `map_input.cpp` — drag set_pos rate counter (1s log)
- `map_render.cpp` — swapViewportSprites replaces copyBackToFront (~500 us vs 101 ms)
- `map_engine.cpp` — raster cache hit/miss/sd-load logs
- `LoRa_APRS_Tracker.cpp` — main loop iteration timing

### Current HAL config at last commit

- num_fbs=2, partial draw buffers (2x 800×48 PSRAM), full_refresh=0, direct_mode=0
- Single framebuffer alternative tested and rejected (tearing, no atomic swap)

### Root cause (confirmed by measurements)

LVGL full_refresh=1 forced re-blit of the 768×768 map canvas into 800×480 framebuffer every touch event. At ~30 MB/s effective PSRAM bandwidth (shared with 46 MB/s DMA scan-out), this takes ~168 ms → 5 fps.

With full_refresh=0 + partial buffers: 39 strips/s but still 5 full cycles/s. Same total PSRAM work, just broken into non-blocking chunks. Bandwidth is the physical ceiling.

### What was ruled out

- 101 ms PSRAM copyBackToFront → fixed with swapViewportSprites (~500 us), not dominant
- LVGL→fb flush copy → eliminated by HAL, not dominant
- Main loop too slow → measured 500 iters/s, avg 2 ms
- num_fbs=3 triple buffering → 3rd buffer invisible to LVGL, no change
- direct_mode toggle → tearing when off, blocking when on
- lv_timer_handler blocking → confirmed, but cause is PSRAM saturation not API

### What remains

The map canvas must stop going through LVGL for pan/zoom. Currently:
- canvas 768×768 → LVGL re-blits entire visible area on every lv_obj_set_pos
- Every touch event triggers this → ~168 ms

The fix: render the map area directly into the hardware framebuffer, bypass LVGL entirely for map content. LVGL handles only buttons, title bar, info bar. Sub-tile pan: memmove in the buffer. Tile boundary cross: full re-render triggered by the render task.

Pattern from IceNav-v3 (SPI 3.5") adapted to RGB 7".
