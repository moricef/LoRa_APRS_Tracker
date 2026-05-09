#ifndef CH422G_H_
#define CH422G_H_

#include <lgfx/v1/platforms/common.hpp>
#include <driver/gpio.h>
#include <esp_rom_sys.h>

#define CH422G_I2C_PORT  0
#define CH422G_I2C_SDA   8
#define CH422G_I2C_SCL   9
#define CH422G_I2C_FREQ  400000

#define CH422G_ADDR_SET  0x24
#define CH422G_ADDR_IO   0x38

extern uint8_t _ch422g_io_state;

static inline void ch422g_write(uint8_t addr, uint8_t data) {
    lgfx::i2c::transactionWrite(CH422G_I2C_PORT, addr, &data, 1, CH422G_I2C_FREQ);
}

static inline void ch422g_init_hw() {
    lgfx::i2c::init(CH422G_I2C_PORT, CH422G_I2C_SDA, CH422G_I2C_SCL);

    // CH422G output mode
    ch422g_write(CH422G_ADDR_SET, 0x01);

    // Touch reset sequence (matches official Waveshare code)
    // IO2(BL) + IO3(LCD_RST) + IO5 — TP_RST low (reset asserted)
    gpio_set_direction((gpio_num_t)4, GPIO_MODE_OUTPUT);
    ch422g_write(CH422G_ADDR_IO, 0x2C);
    esp_rom_delay_us(100000);
    gpio_set_level((gpio_num_t)4, 0);
    esp_rom_delay_us(100000);
    // IO1(TP_RST) + IO2(BL) + IO3(LCD_RST) + IO5 — TP_RST released
    ch422g_write(CH422G_ADDR_IO, 0x2E);
    esp_rom_delay_us(200000);
    gpio_set_direction((gpio_num_t)4, GPIO_MODE_INPUT);

    _ch422g_io_state = 0x2E;
}

static inline void ch422g_backlight_on() {
    // IO2(BL) on, preserving TP_RST, LCD_RST, SD_CS, USB_SEL states
    _ch422g_io_state |= (1 << 2);
    ch422g_write(CH422G_ADDR_IO, _ch422g_io_state);
}

static inline void ch422g_backlight_off() {
    _ch422g_io_state &= ~(1 << 2);
    ch422g_write(CH422G_ADDR_IO, _ch422g_io_state);
}

static inline void ch422g_set_io(uint8_t state) {
    _ch422g_io_state = state;
    ch422g_write(CH422G_ADDR_IO, _ch422g_io_state);
}

static inline void ch422g_pin_write(uint8_t pin, uint8_t level) {
    if (level) _ch422g_io_state |= (1 << pin);
    else       _ch422g_io_state &= ~(1 << pin);
    ch422g_write(CH422G_ADDR_IO, _ch422g_io_state);
}

#endif // CH422G_H_
