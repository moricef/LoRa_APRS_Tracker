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
            cfg.freq_write = 40000000;
            cfg.freq_read  = 40000000;
            cfg.pin_sclk = 40;
            cfg.pin_mosi = 41;
            cfg.pin_miso = 38;
            cfg.pin_dc   = 11;
            
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }
        
        { // Configure display panel control settings
            auto cfg = _panel_instance.config();
            
            cfg.pin_cs           = 12;
            cfg.pin_rst          = 10;
            cfg.pin_busy         = -1;
            
            cfg.panel_width      = 320;
            cfg.panel_height     = 240;
            cfg.memory_width     = 240;
            cfg.memory_height    = 320;
            cfg.offset_x         = 0;
            cfg.offset_y         = 0;
            cfg.rgb_order        = lgfx::rgb_order_rgb;
            cfg.invert           = true;
            
            _panel_instance.config(cfg);
        }
        
        { // Configure backlight control settings
            auto cfg = _light_instance.config();
            
            cfg.pin_bl = 42;
            cfg.invert = false;
            cfg.freq   = 44100;
            cfg.pwm_channel = 7;
            
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }
        
        setPanel(&_panel_instance);
    }
};

#endif // LGFX_TDECK_H_
