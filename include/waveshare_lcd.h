#ifndef WAVESHARE_LCD_H_
#define WAVESHARE_LCD_H_

#ifdef WAVESHARE_S3_TOUCH_LCD_7

#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>

extern esp_lcd_panel_handle_t ws_lcd_panel;

void waveshare_lcd_init();
void waveshare_wait_vsync();
bool gt911_read_touch(uint16_t *x, uint16_t *y);

#endif // WAVESHARE_S3_TOUCH_LCD_7
#endif // WAVESHARE_LCD_H_
