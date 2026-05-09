#ifdef WAVESHARE_S3_TOUCH_LCD_7

#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <lgfx/v1/platforms/common.hpp>
#include "waveshare_lcd.h"

static const char *TAG = "WS_LCD";

esp_lcd_panel_handle_t ws_lcd_panel = nullptr;
static SemaphoreHandle_t vsync_sem = nullptr;

void waveshare_lcd_init() {
    esp_lcd_rgb_panel_config_t panel_cfg = {};
    panel_cfg.clk_src = LCD_CLK_SRC_DEFAULT;

    panel_cfg.timings.pclk_hz = 16000000;
    panel_cfg.timings.h_res = 800;
    panel_cfg.timings.v_res = 480;
    panel_cfg.timings.hsync_pulse_width = 4;
    panel_cfg.timings.hsync_back_porch  = 8;
    panel_cfg.timings.hsync_front_porch = 8;
    panel_cfg.timings.vsync_pulse_width = 4;
    panel_cfg.timings.vsync_back_porch  = 8;
    panel_cfg.timings.vsync_front_porch = 8;
    panel_cfg.timings.flags.pclk_active_neg = 1;

    panel_cfg.data_width = 16;
    panel_cfg.bits_per_pixel = 16;
    panel_cfg.num_fbs = 2;
    panel_cfg.bounce_buffer_size_px = 800 * 10;
    panel_cfg.dma_burst_size = 64;

    panel_cfg.hsync_gpio_num = 46;
    panel_cfg.vsync_gpio_num = 3;
    panel_cfg.de_gpio_num    = 5;
    panel_cfg.pclk_gpio_num  = 7;
    panel_cfg.disp_gpio_num  = -1;

    panel_cfg.data_gpio_nums[0]  = 14;
    panel_cfg.data_gpio_nums[1]  = 38;
    panel_cfg.data_gpio_nums[2]  = 18;
    panel_cfg.data_gpio_nums[3]  = 17;
    panel_cfg.data_gpio_nums[4]  = 10;
    panel_cfg.data_gpio_nums[5]  = 39;
    panel_cfg.data_gpio_nums[6]  = 0;
    panel_cfg.data_gpio_nums[7]  = 45;
    panel_cfg.data_gpio_nums[8]  = 48;
    panel_cfg.data_gpio_nums[9]  = 47;
    panel_cfg.data_gpio_nums[10] = 21;
    panel_cfg.data_gpio_nums[11] = 1;
    panel_cfg.data_gpio_nums[12] = 2;
    panel_cfg.data_gpio_nums[13] = 42;
    panel_cfg.data_gpio_nums[14] = 41;
    panel_cfg.data_gpio_nums[15] = 40;

    panel_cfg.flags.fb_in_psram = 1;

    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_cfg, &ws_lcd_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(ws_lcd_panel));

    vsync_sem = xSemaphoreCreateBinary();

    esp_lcd_rgb_panel_event_callbacks_t cbs = {};
    cbs.on_vsync = [](esp_lcd_panel_handle_t, const esp_lcd_rgb_panel_event_data_t*, void*) -> bool {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(vsync_sem, &woken);
        return woken == pdTRUE;
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(ws_lcd_panel, &cbs, nullptr));

    ESP_LOGI(TAG, "RGB panel initialized: num_fbs=2, bounce 800x10, VSYNC sync");
}

void waveshare_wait_vsync() {
    if (vsync_sem) xSemaphoreTake(vsync_sem, pdMS_TO_TICKS(100));
}

#define GT911_ADDR  0x5D
#define GT911_FREQ  400000

bool gt911_read_touch(uint16_t *x, uint16_t *y) {
    uint8_t reg_status[2] = {0x81, 0x4E};
    uint8_t status = 0;

    auto res = lgfx::i2c::transactionWriteRead(
        0, GT911_ADDR, reg_status, 2, &status, 1, GT911_FREQ);
    if (!res.has_value()) return false;

    if (!(status & 0x80) || (status & 0x0F) == 0) {
        if (status & 0x80) {
            uint8_t clear[3] = {0x81, 0x4E, 0x00};
            lgfx::i2c::transactionWrite(0, GT911_ADDR, clear, 3, GT911_FREQ);
        }
        return false;
    }

    uint8_t reg_touch[2] = {0x81, 0x50};
    uint8_t td[8] = {};
    lgfx::i2c::transactionWriteRead(
        0, GT911_ADDR, reg_touch, 2, td, 8, GT911_FREQ);

    *x = td[0] | (td[1] << 8);
    *y = td[2] | (td[3] << 8);

    uint8_t clear[3] = {0x81, 0x4E, 0x00};
    lgfx::i2c::transactionWrite(0, GT911_ADDR, clear, 3, GT911_FREQ);

    return true;
}

#endif // WAVESHARE_S3_TOUCH_LCD_7
