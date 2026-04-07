/* Copyright (C) 2025 Ricardo Guzman - CA2RXU
 * 
 * This file is part of LoRa APRS Tracker.
 * 
 * LoRa APRS Tracker is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * LoRa APRS Tracker is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with LoRa APRS Tracker. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef BOARD_PINOUT_H_
#define BOARD_PINOUT_H_

    //  LoRa Radio (SX1262 on external module)
    #define HAS_SX1262
    #define RADIO_SCLK_PIN      10
    #define RADIO_MISO_PIN      9
    #define RADIO_MOSI_PIN      3
    #define RADIO_CS_PIN        (uint32_t)0
    #define RADIO_RST_PIN       2   // Shared with TFT_RST
    #define RADIO_DIO1_PIN      1
    #define RADIO_BUSY_PIN      46

    //  Display (ST7789, SPI ILI9488 via LovyanGFX)
    #define HAS_TFT
    #define HAS_TOUCHSCREEN

    //  GPS (External UART module)
    #define GPS_RX              17
    #define GPS_TX              18
    #define GPS_BAUDRATE        9600

    //  Battery (no PMIC on Crowpanel)
    #define BATTERY_PIN         -1

    //  SD Card (SPI, separate from LoRa/Display bus)
    #define BOARD_SDCARD_CS     7
    #define BOARD_SDCARD_MOSI   6
    #define BOARD_SDCARD_MISO   4
    #define BOARD_SDCARD_SCK    5

    //  Buttons & Joystick
    #define BUTTON_PIN          -1  // GPIO0 used by LoRa CS, boot button disabled

    //  I2C (Touch GT911 controller)
    #define BOARD_I2C_SDA       15
    #define BOARD_I2C_SCL       16

#endif // BOARD_PINOUT_H_
