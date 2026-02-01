#ifndef LGFX_TDECK_H_
#define LGFX_TDECK_H_

#include <LovyanGFX.hpp>

// Custom LGFX class for LILYGO T-Deck Plus
class LGFX_TDeck : public lgfx::LGFX_Device
{
    lgfx::Panel_ST7789 _panel_instance;
    lgfx::Bus_SPI      _bus_instance;
    lgfx::Light_PWM    _light_instance;

public:
    LGFX_TDeck(void)
    {
        { // Configure bus control settings for ESP32-S3
            auto cfg = _bus_instance.config();
            cfg.spi_host = SPI2_HOST;
            cfg.spi_mode = 0;
            cfg.use_lock = false;
            cfg.freq_write = 40000000;
            cfg.freq_read = 16000000;
            cfg.spi_3wire = false;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk = 40;
            cfg.pin_mosi = 41;
            cfg.pin_miso = 38;
            cfg.pin_dc = 11;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        { // Configure display panel control settings
            auto cfg = _panel_instance.config();
            cfg.pin_cs = 12;
            cfg.pin_rst = -1;
            cfg.pin_busy = -1;
            cfg.panel_width = 320;
            cfg.panel_height = 240;
            cfg.memory_width = 320;
            cfg.memory_height = 240;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.dummy_read_pixel = 16;
            cfg.dummy_read_bits = 2;
            cfg.readable = true;
            cfg.invert = true;
            cfg.rgb_order = false; // false = RGB, true = BGR
            cfg.dlen_16bit = false;
            cfg.bus_shared = true;
            _panel_instance.config(cfg);
        }

        { // Configure backlight control settings
            auto cfg = _light_instance.config();
            cfg.pin_bl = 42;
            cfg.invert = false;
            cfg.freq = 44100;
            cfg.pwm_channel = 7;
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }
        
        setPanel(&_panel_instance);
    }
};

#endif // LGFX_TDECK_H_
