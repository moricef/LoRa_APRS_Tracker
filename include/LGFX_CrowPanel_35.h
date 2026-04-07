/* LovyanGFX Configuration for Crowpanel Advance 3.5"
 * ESP32-S3 with ILI9488 SPI display and GT911 I2C touch
 */

#ifndef LGFX_CROWPANEL_35_H_
#define LGFX_CROWPANEL_35_H_

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX_CrowPanel_35 : public lgfx::LGFX_Device
{
    lgfx::Panel_ILI9488     _panel_instance;
    lgfx::Bus_SPI           _bus_instance;
    lgfx::Light_PWM         _light_instance;
    lgfx::Touch_GT911       _touch_instance;

public:
    LGFX_CrowPanel_35(void)
    {
        // SPI Bus Configuration
        {
            auto cfg = _bus_instance.config();
            cfg.spi_host = SPI2_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 40000000;  // 40 MHz write
            cfg.freq_read  = 16000000;  // 16 MHz read
            cfg.spi_3wire  = false;
            cfg.use_lock   = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;

            // Verified pins from async variant
            cfg.pin_sclk = 42;
            cfg.pin_mosi = 39;
            cfg.pin_miso = -1;  // No MISO for write-only
            cfg.pin_dc   = 41;

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        // Panel Configuration (ILI9488, 480x320)
        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs           = 40;
            cfg.pin_rst          = 2;
            cfg.pin_busy         = -1;
            cfg.memory_width     = 320;
            cfg.memory_height    = 480;
            cfg.panel_width      = 320;
            cfg.panel_height     = 480;
            cfg.offset_x         = 0;
            cfg.offset_y         = 0;
            cfg.offset_rotation  = 3;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits  = 1;
            cfg.readable         = false;
            cfg.invert           = true;
            cfg.rgb_order        = false;
            cfg.dlen_16bit       = false;
            cfg.bus_shared       = true;  // Shared with LoRa/SD

            _panel_instance.config(cfg);
        }

        // Backlight (PWM)
        {
            auto cfg = _light_instance.config();
            cfg.pin_bl = 38;
            cfg.invert = false;
            cfg.freq   = 44100;
            cfg.pwm_channel = 7;

            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }

        // Touch Controller (GT911, I2C)
        {
            auto cfg = _touch_instance.config();
            cfg.x_min      = 0;
            cfg.x_max      = 319;
            cfg.y_min      = 0;
            cfg.y_max      = 479;
            cfg.pin_int    = -1;
            cfg.bus_shared = false;
            cfg.offset_rotation = 0;

            // Verified I2C pins from async variant
            cfg.i2c_port = 0;
            cfg.i2c_addr = 0x5D;  // GT911 address
            cfg.pin_sda  = 15;
            cfg.pin_scl  = 16;
            cfg.freq = 400000;

            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
    }
};

#endif // LGFX_CROWPANEL_35_H_
