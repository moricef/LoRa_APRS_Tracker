/* Waveshare ESP32-S3-Touch-LCD-7 pin definitions
 *
 * LCD (ST7262 RGB 16-bit): 0,1,2,3,5,7,10,14,17,18,21,38,39,40,41,42,45,46,47,48
 * Touch (GT911 I2C): SCL=9, SDA=8, IRQ=4, RST via CH422G pin1
 * IO Expander (CH422G I2C): SCL=9, SDA=8, addr=0x20
 *   pin1=TP_RST, pin2=DISP, pin3=LCD_RST, pin4=SD_CS
 * SD (SPI): MOSI=11, SCK=12, MISO=13, CS via CH422G pin4
 * UART0: TX=43, RX=44 (shared UART1 USB-C <-> UART2 Grove via switch)
 * USB OTG: D-=19, D+=20 (USB_SERIAL_JTAG console)
 * RS485: TXD=15, RXD=16 (unused, available)
 *
 * LoRa SX1262: not on board — connect externally or use C3 via proto UART
 */

#ifndef BOARD_PINOUT_H_
#define BOARD_PINOUT_H_

    /* UART for C3 proto or GPS (shared UART0, switch to UART2 Grove) */
    #define GPS_RX          43  // S3 TX -> C3 RX
    #define GPS_TX          44  // S3 RX <- C3 TX
    #define GPS_BAUDRATE    460800

    /* LoRa SX1262 — external module on RS485 header (GPIO15/16) + GPIO6 */
    #define HAS_SX1262
    #define RADIO_SCLK_PIN      16
    #define RADIO_MISO_PIN      15
    #define RADIO_MOSI_PIN      6
    #define RADIO_CS_PIN        33      // sensor header J3 pin 14 (GPIO33)
    #define RADIO_RST_PIN       34      // sensor header J3 pin 15 (GPIO34)
    #define RADIO_DIO1_PIN      35      // sensor header J3 pin 16 (GPIO35)
    #define RADIO_BUSY_PIN      36      // sensor header J3 pin 17 (GPIO36)

    /* Display */
    #define HAS_TFT
    #define HAS_TOUCHSCREEN

    /* SD Card */
    #define BOARD_SDCARD_CS     -1      // managed by CH422G expander pin4
    #define BOARD_SDCARD_MOSI   11
    #define BOARD_SDCARD_MISO   13
    #define BOARD_SDCARD_SCK    12

    /* I2C */
    #define BOARD_I2C_SDA       8
    #define BOARD_I2C_SCL       9

    /* Backlight via CH422G pin2 (on/off, no PWM) */
    #define BOARD_BL_PIN        -1

    /* No keyboard, no battery ADC */
    #define BATTERY_PIN         -1

#endif
