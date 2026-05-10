#ifdef WAVESHARE_S3_TOUCH_LCD_7

#include "display_hal.h"

#ifdef USE_LVGL_UI

#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <lgfx/v1/platforms/common.hpp>
#include <lvgl.h>

// =============================================================================
// Waveshare 7" S3 Touch LCD — RGB direct framebuffer mode
//
// Architecture: LVGL writes directly into one of the two hardware framebuffers
// exposed by esp_lcd_rgb_panel (num_fbs=2). The flush callback hands the
// completed framebuffer back to esp_lcd, which performs the swap on the next
// VSYNC. No intermediate PSRAM buffer, no copy in flush_cb. Eliminates the
// PSRAM DMA conflict that caused tearing/jerks during pan/zoom.
// =============================================================================

namespace {

const char* TAG = "DispHAL_WS";

constexpr int LCD_HRES = 800;
constexpr int LCD_VRES = 480;

esp_lcd_panel_handle_t panel       = nullptr;
SemaphoreHandle_t      vsync_sem   = nullptr;

lv_disp_draw_buf_t draw_buf;
lv_disp_drv_t      disp_drv;

void* fb_arr[2] = { nullptr, nullptr };

// LVGL flush — esp_lcd_panel_draw_bitmap with a framebuffer pointer just
// queues that fb as next-to-display; the actual swap happens on VSYNC.
// No memory copy.
void disp_flush_cb(lv_disp_drv_t* drv, const lv_area_t* area, lv_color_t* color_map) {
    static uint32_t flushCount = 0;
    static uint64_t lastReportUs = 0;
    static uint64_t totalDrawUs = 0;
    static uint64_t lastFlushEndUs = 0;
    static uint64_t intervalSumUs = 0;

    uint64_t t0 = esp_timer_get_time();
    if (lastFlushEndUs != 0) intervalSumUs += (t0 - lastFlushEndUs);

    esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1, color_map);

    uint64_t t1 = esp_timer_get_time();
    totalDrawUs += (t1 - t0);
    lastFlushEndUs = t1;
    flushCount++;

    if (lastReportUs == 0) lastReportUs = t1;
    if (t1 - lastReportUs >= 1000000) {
        uint32_t avgDraw = flushCount ? (uint32_t)(totalDrawUs / flushCount) : 0;
        uint32_t avgGap  = flushCount ? (uint32_t)(intervalSumUs / flushCount) : 0;
        ESP_LOGI(TAG, "flush stats (1s): %u frames, draw avg %u us, gap avg %u us",
                      flushCount, avgDraw, avgGap);
        flushCount = 0;
        totalDrawUs = 0;
        intervalSumUs = 0;
        lastReportUs = t1;
    }

    lv_disp_flush_ready(drv);
}

bool panel_init() {
    esp_lcd_rgb_panel_config_t cfg = {};
    cfg.clk_src = LCD_CLK_SRC_DEFAULT;

    cfg.timings.pclk_hz            = 16000000;
    cfg.timings.h_res              = LCD_HRES;
    cfg.timings.v_res              = LCD_VRES;
    cfg.timings.hsync_pulse_width  = 4;
    cfg.timings.hsync_back_porch   = 8;
    cfg.timings.hsync_front_porch  = 8;
    cfg.timings.vsync_pulse_width  = 4;
    cfg.timings.vsync_back_porch   = 8;
    cfg.timings.vsync_front_porch  = 8;
    cfg.timings.flags.pclk_active_neg = 1;

    cfg.data_width             = 16;
    cfg.bits_per_pixel         = 16;
    cfg.num_fbs                = 2;
    cfg.bounce_buffer_size_px  = LCD_HRES * 10;
    cfg.dma_burst_size         = 64;

    cfg.hsync_gpio_num = 46;
    cfg.vsync_gpio_num = 3;
    cfg.de_gpio_num    = 5;
    cfg.pclk_gpio_num  = 7;
    cfg.disp_gpio_num  = -1;

    cfg.data_gpio_nums[0]  = 14;
    cfg.data_gpio_nums[1]  = 38;
    cfg.data_gpio_nums[2]  = 18;
    cfg.data_gpio_nums[3]  = 17;
    cfg.data_gpio_nums[4]  = 10;
    cfg.data_gpio_nums[5]  = 39;
    cfg.data_gpio_nums[6]  = 0;
    cfg.data_gpio_nums[7]  = 45;
    cfg.data_gpio_nums[8]  = 48;
    cfg.data_gpio_nums[9]  = 47;
    cfg.data_gpio_nums[10] = 21;
    cfg.data_gpio_nums[11] = 1;
    cfg.data_gpio_nums[12] = 2;
    cfg.data_gpio_nums[13] = 42;
    cfg.data_gpio_nums[14] = 41;
    cfg.data_gpio_nums[15] = 40;

    cfg.flags.fb_in_psram = 1;

    if (esp_lcd_new_rgb_panel(&cfg, &panel) != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_rgb_panel failed");
        return false;
    }
    if (esp_lcd_panel_init(panel) != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_panel_init failed");
        return false;
    }

    vsync_sem = xSemaphoreCreateBinary();
    esp_lcd_rgb_panel_event_callbacks_t cbs = {};
    cbs.on_vsync = [](esp_lcd_panel_handle_t, const esp_lcd_rgb_panel_event_data_t*, void*) -> bool {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(vsync_sem, &woken);
        return woken == pdTRUE;
    };
    esp_lcd_rgb_panel_register_event_callbacks(panel, &cbs, nullptr);

    if (esp_lcd_rgb_panel_get_frame_buffer(panel, 2, &fb_arr[0], &fb_arr[1]) != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_rgb_panel_get_frame_buffer failed");
        return false;
    }

    ESP_LOGI(TAG, "RGB panel initialized: num_fbs=2, fb1=%p fb2=%p", fb_arr[0], fb_arr[1]);
    return true;
}

// Monitor callback: reports time + pixel count per LVGL refresh.
// Aggregates over 1s to avoid log spam. Helps diagnose where the time goes.
void disp_monitor_cb(lv_disp_drv_t* /*drv*/, uint32_t time_ms, uint32_t px) {
    static uint32_t cycles = 0;
    static uint32_t totalTimeMs = 0;
    static uint32_t totalPx = 0;
    static uint32_t maxTimeMs = 0;
    static uint64_t lastReportUs = 0;

    cycles++;
    totalTimeMs += time_ms;
    totalPx     += px;
    if (time_ms > maxTimeMs) maxTimeMs = time_ms;

    uint64_t nowUs = esp_timer_get_time();
    if (lastReportUs == 0) lastReportUs = nowUs;
    if (nowUs - lastReportUs >= 1000000) {
        uint32_t avgTime = cycles ? (totalTimeMs / cycles) : 0;
        uint32_t avgPx   = cycles ? (totalPx / cycles)     : 0;
        ESP_LOGI(TAG, "LVGL render (1s): %u cycles, avg %u ms (max %u), avg %u px/cycle",
                      cycles, avgTime, maxTimeMs, avgPx);
        cycles = 0;
        totalTimeMs = 0;
        totalPx = 0;
        maxTimeMs = 0;
        lastReportUs = nowUs;
    }
}

bool lvgl_register() {
    // Both LVGL buffers point directly at the hardware framebuffers in PSRAM.
    // LVGL alternates between them on each refresh (full_refresh=1 + double-buf).
    lv_disp_draw_buf_init(&draw_buf,
                          (lv_color_t*)fb_arr[0],
                          (lv_color_t*)fb_arr[1],
                          LCD_HRES * LCD_VRES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res        = LCD_HRES;
    disp_drv.ver_res        = LCD_VRES;
    disp_drv.flush_cb       = disp_flush_cb;
    disp_drv.monitor_cb     = disp_monitor_cb;
    disp_drv.draw_buf       = &draw_buf;
    disp_drv.full_refresh   = 1;
    // direct_mode disabled — empirically forces a long wait between frames
    // when combined with num_fbs=2. Without it, LVGL still alternates
    // between the two hardware framebuffers as draw buffers.
    disp_drv.direct_mode    = 0;
    if (lv_disp_drv_register(&disp_drv) == nullptr) {
        ESP_LOGE(TAG, "lv_disp_drv_register failed");
        return false;
    }
    ESP_LOGI(TAG, "LVGL registered: direct mode + double hw fb");
    return true;
}

// =============================================================================
// GT911 capacitive touch (I2C via lgfx::i2c, shares I2C0 with CH422G)
// =============================================================================

constexpr uint8_t  GT911_ADDR = 0x5D;
constexpr uint32_t GT911_FREQ = 400000;

bool gt911_read(uint16_t* x, uint16_t* y) {
    uint8_t reg_status[2] = { 0x81, 0x4E };
    uint8_t status = 0;

    auto res = lgfx::i2c::transactionWriteRead(
        0, GT911_ADDR, reg_status, 2, &status, 1, GT911_FREQ);
    if (!res.has_value()) return false;

    if (!(status & 0x80) || (status & 0x0F) == 0) {
        if (status & 0x80) {
            uint8_t clear[3] = { 0x81, 0x4E, 0x00 };
            lgfx::i2c::transactionWrite(0, GT911_ADDR, clear, 3, GT911_FREQ);
        }
        return false;
    }

    uint8_t reg_touch[2] = { 0x81, 0x50 };
    uint8_t td[8] = {};
    lgfx::i2c::transactionWriteRead(
        0, GT911_ADDR, reg_touch, 2, td, 8, GT911_FREQ);

    *x = td[0] | (td[1] << 8);
    *y = td[2] | (td[3] << 8);

    uint8_t clear[3] = { 0x81, 0x4E, 0x00 };
    lgfx::i2c::transactionWrite(0, GT911_ADDR, clear, 3, GT911_FREQ);

    return true;
}

} // anonymous namespace

namespace DisplayHAL {

bool init() {
    if (!panel_init())   return false;
    if (!lvgl_register()) return false;
    return true;
}

bool readTouch(uint16_t* x, uint16_t* y) {
    return gt911_read(x, y);
}

} // namespace DisplayHAL

#endif // USE_LVGL_UI
#endif // WAVESHARE_S3_TOUCH_LCD_7
