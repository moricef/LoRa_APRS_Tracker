#ifndef LGFX_WAVESHARE7_H_
#define LGFX_WAVESHARE7_H_

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>

// CH422G I2C expander (shared bus with GT911: SDA=8, SCL=9)
#define CH422G_ADDR     0x20
#define CH422G_REG_SET  0x24  // WR_SET: bit0=IO_OE, bit2=OD_EN
#define CH422G_REG_OC   0x23  // WR_OC: open-drain config
#define CH422G_REG_IO   0x38  // WR_IO: output state

// Waveshare ESP32-S3-Touch-LCD-7
// ST7262 RGB 16-bit 800x480 + GT911 I2C touch + CH422G expander
class LGFX_Waveshare7 : public lgfx::LGFX_Device
{
    lgfx::Panel_RGB       _panel_instance;
    lgfx::Bus_RGB         _bus_instance;
    lgfx::Touch_GT911     _touch_instance;

public:
    uint8_t _ch422g_io_state = 0;

    LGFX_Waveshare7(void)
    {
        // RGB bus (ST7262 800x480 16-bit)
        {
            auto cfg = _bus_instance.config();
            cfg.panel = &_panel_instance;
            cfg.freq_write = 16000000;
            cfg.pin_pclk   = 7;
            cfg.pin_vsync  = 3;
            cfg.pin_hsync  = 46;
            cfg.pin_henable = 5;

            cfg.pin_d0  = 14;  cfg.pin_d1  = 38;
            cfg.pin_d2  = 18;  cfg.pin_d3  = 17;
            cfg.pin_d4  = 10;  cfg.pin_d5  = 39;
            cfg.pin_d6  = 0;   cfg.pin_d7  = 45;
            cfg.pin_d8  = 48;  cfg.pin_d9  = 47;
            cfg.pin_d10 = 21;  cfg.pin_d11 = 1;
            cfg.pin_d12 = 2;   cfg.pin_d13 = 42;
            cfg.pin_d14 = 41;  cfg.pin_d15 = 40;

            cfg.hsync_pulse_width = 4;
            cfg.hsync_back_porch  = 8;
            cfg.hsync_front_porch = 8;
            cfg.vsync_pulse_width = 4;
            cfg.vsync_back_porch  = 16;
            cfg.vsync_front_porch = 16;
            cfg.pclk_active_neg = 1;

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        // Panel (800x480)
        {
            auto cfg = _panel_instance.config();
            cfg.memory_width  = 800;
            cfg.memory_height = 480;
            cfg.panel_width   = 800;
            cfg.panel_height  = 480;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            _panel_instance.config(cfg);
        }

        // Touch GT911 (I2C addr 0x5D, shared bus with CH422G)
        {
            auto cfg = _touch_instance.config();
            cfg.x_min = 0;
            cfg.x_max = 799;
            cfg.y_min = 0;
            cfg.y_max = 479;
            cfg.pin_int = 4;
            cfg.bus_shared = true;
            cfg.offset_rotation = 0;

            cfg.i2c_port = 0;
            cfg.i2c_addr = 0x5D;
            cfg.pin_sda = 8;
            cfg.pin_scl = 9;
            cfg.freq = 400000;

            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
    }

    // CH422G expander helpers

    void ch422g_begin() {
        Wire.beginTransmission(CH422G_ADDR);
        Wire.write(CH422G_REG_SET);
        Wire.write(0x01);  // IO_OE = 1 (all outputs)
        Wire.endTransmission();
        _ch422g_io_state = 0x00;
    }

    void ch422g_pin_write(uint8_t pin, uint8_t level) {
        if (level) _ch422g_io_state |= (1 << pin);
        else       _ch422g_io_state &= ~(1 << pin);
        Wire.beginTransmission(CH422G_ADDR);
        Wire.write(CH422G_REG_IO);
        Wire.write(_ch422g_io_state);
        Wire.endTransmission();
    }
};

#endif // LGFX_WAVESHARE7_H_
