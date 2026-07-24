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

#include <Arduino.h>
#include <esp_log.h>
#include "configuration.h"
#include "battery_utils.h"
#include "board_pinout.h"
#include "power_utils.h"
#include "display.h"

#ifdef HAS_FUEL_GAUGE_I2C
    static const char *TAG = "BATT";
    #if defined(CROWPANEL_ADVANCE_35)
        // CrowPanel: GT911 touch and MAX17048 share I2C_NUM_0 via LovyanGFX bare-metal driver.
        // Arduino Wire cannot coexist with lgfx::i2c on the same peripheral, so we talk to
        // the MAX17048 directly through the lgfx I2C API (same bus as GT911).
        #include <LovyanGFX.hpp>
        static constexpr int      MAX17048_I2C_PORT = 0;
        static constexpr uint8_t  MAX17048_I2C_ADDR = 0x36;
        static constexpr uint32_t MAX17048_I2C_FREQ = 400000;
        static constexpr uint8_t  MAX17048_REG_VCELL = 0x02;
        static constexpr uint8_t  MAX17048_REG_SOC   = 0x04;
        static constexpr uint8_t  MAX17048_REG_MODE  = 0x06;
        static constexpr uint8_t  MAX17048_REG_CRATE = 0x16;
        static bool fuelGaugeReady = false;

        static bool max17048Read16(uint8_t reg, uint16_t &out) {
            uint8_t buf[2] = {0, 0};
            auto res = lgfx::v1::i2c::transactionWriteRead(
                MAX17048_I2C_PORT, MAX17048_I2C_ADDR, &reg, 1, buf, 2, MAX17048_I2C_FREQ);
            if (res.has_error()) return false;
            out = ((uint16_t)buf[0] << 8) | buf[1];
            return true;
        }
        static float max17048CellVoltage() {
            uint16_t v;
            if (!max17048Read16(MAX17048_REG_VCELL, v)) return NAN;
            return v * 78.125f / 1000000.0f;
        }
        static float max17048CellPercent() {
            uint16_t v;
            if (!max17048Read16(MAX17048_REG_SOC, v)) return NAN;
            return v / 256.0f;
        }
        static bool max17048QuickStart() {
            // MODE register: write 0x4000 to trigger ModelGauge recompute from current VCELL
            uint8_t buf[3] = { MAX17048_REG_MODE, 0x40, 0x00 };
            auto res = lgfx::v1::i2c::transactionWrite(
                MAX17048_I2C_PORT, MAX17048_I2C_ADDR, buf, 3, MAX17048_I2C_FREQ);
            return res.has_value();
        }
        static float max17048ChargeRate() {
            uint16_t v;
            if (!max17048Read16(MAX17048_REG_CRATE, v)) return NAN;
            return (int16_t)v * 0.208f;
        }
    #else
        #include <Wire.h>
        #include "Adafruit_MAX1704X.h"
        static Adafruit_MAX17048 fuelGauge;
        static bool fuelGaugeReady = false;
    #endif
#endif

#ifdef ADC_CTRL
    uint32_t    adcCtrlTime         = 0;
    uint8_t     measuringState      = 0;
#endif

#ifdef HAS_AXP192
    extern XPowersAXP192 PMU;
#endif
#ifdef HAS_AXP2101
    extern XPowersAXP2101 PMU;
#endif

extern      Configuration           Config;
uint32_t    batteryMeasurmentTime   = 0;
int         averageReadings         = 20;

String      batteryVoltage          = "";
bool        batteryConnected      = false;

extern      String                  batteryChargeDischargeCurrent;

float       lora32BatReadingCorr    = 6.5; // % of correction to higher value to reflect the real battery voltage (adjust this to your needs)


namespace BATTERY_Utils {

    // Li-Ion discharge curve lookup table (voltage -> percent)
    // Based on typical Li-Ion single cell discharge profile
    static const float battLUT[][2] = {
        {4.20, 100}, {4.15, 95}, {4.10, 90}, {4.05, 85},
        {4.00, 80},  {3.95, 75}, {3.90, 70}, {3.85, 65},
        {3.80, 60},  {3.75, 55}, {3.70, 50}, {3.65, 45},
        {3.60, 40},  {3.55, 35}, {3.50, 30}, {3.45, 25},
        {3.40, 20},  {3.35, 15}, {3.30, 10}, {3.20,  5},
        {3.00,  0}
    };
    static const int battLUTSize = sizeof(battLUT) / sizeof(battLUT[0]);

    int voltageToPercent(float voltage) {
        if (voltage >= battLUT[0][0]) return 100;
        if (voltage <= battLUT[battLUTSize - 1][0]) return 0;
        for (int i = 0; i < battLUTSize - 1; i++) {
            if (voltage >= battLUT[i + 1][0]) {
                // Linear interpolation between two points
                float v0 = battLUT[i][0],     p0 = battLUT[i][1];
                float v1 = battLUT[i + 1][0], p1 = battLUT[i + 1][1];
                return (int)(p1 + (voltage - v1) / (v0 - v1) * (p0 - p1));
            }
        }
        return 0;
    }

    String getPercentVoltageBattery(float voltage) {
        #ifdef HAS_FUEL_GAUGE_I2C
            if (fuelGaugeReady) {
                #if defined(CROWPANEL_ADVANCE_35)
                    float soc = max17048CellPercent();
                #else
                    float soc = fuelGauge.cellPercent();
                #endif
                if (!isnan(soc) && soc > 0.0f) {
                    int percent = (int)(soc + 0.5f);
                    if (percent > 100) percent = 100;
                    if (percent < 0)   percent = 0;
                    return (percent < 100) ? (((percent < 10) ? "  ": " ") + String(percent)) : "100";
                }
            }
        #endif
        int percent = voltageToPercent(voltage);
        return (percent < 100) ? (((percent < 10) ? "  ": " ") + String(percent)) : "100";
    }

    String getBatteryInfoVoltage() {
        return batteryVoltage;
    }

    void initBatteryGauge() {
        #ifdef HAS_FUEL_GAUGE_I2C
            #if defined(CROWPANEL_ADVANCE_35)
                // Bus already up via LovyanGFX (GT911). Probe MAX17048 by reading VCELL.
                delay(10);
                float v = NAN;
                for (int retry = 0; retry < 5; ++retry) {
                    v = max17048CellVoltage();
                    if (!isnan(v) && v > 0.5f) { fuelGaugeReady = true; break; }
                    delay(50);
                }
                if (fuelGaugeReady) {
                    ESP_LOGI(TAG, "MAX17048 OK (lgfx i2c) - cell=%.3fV SOC=%.1f%% (raw)",
                             v, max17048CellPercent());
                    if (max17048QuickStart()) {
                        delay(150);  // ModelGauge needs ~125ms to recompute SOC
                        ESP_LOGI(TAG, "MAX17048 QuickStart done - SOC=%.1f%%",
                                 max17048CellPercent());
                    } else {
                        ESP_LOGW(TAG, "MAX17048 QuickStart write failed");
                    }
                } else {
                    ESP_LOGE(TAG, "MAX17048 not found on lgfx i2c port %d addr 0x%02X",
                             MAX17048_I2C_PORT, MAX17048_I2C_ADDR);
                }
            #else
                delay(10);
                fuelGaugeReady = fuelGauge.begin(&Wire);
                if (fuelGaugeReady) {
                    float v = fuelGauge.cellVoltage();
                    if (v == 0.0f || isnan(v)) {
                        delay(100);
                        v = fuelGauge.cellVoltage();
                    }
                    ESP_LOGI(TAG, "MAX17048 OK - cell=%.3fV SOC=%.1f%%",
                             v, fuelGauge.cellPercent());
                } else {
                    ESP_LOGE(TAG, "MAX17048 not found, battery monitoring disabled");
                }
            #endif
        #endif
    }

    float readBatteryVoltage() {
        #ifdef HAS_FUEL_GAUGE_I2C
            if (fuelGaugeReady) {
                #if defined(CROWPANEL_ADVANCE_35)
                    float v = max17048CellVoltage();
                    return isnan(v) ? 0.0f : v;
                #else
                    return fuelGauge.cellVoltage();
                #endif
            }
            return 0.0;
        #endif
        #if defined(HAS_AXP192) || defined(HAS_AXP2101)
            return (PMU.getBattVoltage() / 1000.0);
        #else
            #ifdef BATTERY_PIN
                #if BATTERY_PIN == -1
                return 0.0;
                #endif
                int sampleSum = 0;
                analogRead(BATTERY_PIN);    // Dummy Read
                delay(1);
                for (int i = 0; i < averageReadings; i++) {
                    sampleSum += analogRead(BATTERY_PIN);
                    delay(3);
                }
                int adc_value = sampleSum/averageReadings;
                double voltage = (adc_value * 3.3 ) / 4095.0;

                #ifdef LIGHTTRACKER_PLUS_1_0
                    double inputDivider = (1.0 / (560.0 + 100.0)) * 100.0;  // The voltage divider is a 560k + 100k resistor in series, 100k on the low side.
                    return ((voltage / inputDivider) * 1.11029) + 0.14431;
                #endif
                #if defined(CROWPANEL_ADVANCE_35)
                    return (2 * (voltage + 0.1)); // Adjust this multiplier for CrowPanel if needed
                #endif
                #if defined(TTGO_T_Beam_V0_7) || defined(TTGO_T_LORA32_V2_1_GPS) || defined(TTGO_T_LORA32_V2_1_GPS_915) || defined(TTGO_T_LORA32_V2_1_TNC) || defined(TTGO_T_LORA32_V2_1_TNC_915) || defined(ESP32_DIY_LoRa_GPS) || defined(ESP32_DIY_LoRa_GPS_915) || defined(ESP32_DIY_1W_LoRa_GPS) || defined(ESP32_DIY_1W_LoRa_GPS_915) || defined(ESP32_DIY_1W_LoRa_GPS_LLCC68) || defined(OE5HWN_MeshCom) || defined(TTGO_T_DECK_GPS) || defined(TTGO_T_DECK_PLUS) || defined(ESP32S3_DIY_LoRa_GPS) || defined(ESP32S3_DIY_LoRa_GPS_915) || defined(TROY_LoRa_APRS) || defined(RPC_Electronics_1W_LoRa_GPS)
                    return (2 * (voltage + 0.1)) * (1 + (lora32BatReadingCorr/100)); // (2 x 100k voltage divider) 2 x voltage divider/+0.1 because ESP32 nonlinearity ~100mV ADC offset/extra correction
                #endif
                #if defined(HELTEC_V3_GPS) || defined(HELTEC_V3_TNC) || defined(HELTEC_V3_2_GPS) || defined(HELTEC_V3_2_TNC) || defined(HELTEC_WIRELESS_TRACKER) || defined(HELTEC_WSL_V3_GPS_DISPLAY) || defined(ESP32_C3_DIY_LoRa_GPS) || defined(ESP32_C3_DIY_LoRa_GPS_915) || defined(WEMOS_ESP32_Bat_LoRa_GPS)
                    double inputDivider = (1.0 / (390.0 + 100.0)) * 100.0;  // The voltage divider is a 390k + 100k resistor in series, 100k on the low side. 
                    return (voltage / inputDivider) + 0.285; // Yes, this offset is excessive, but the ADC on the ESP32s3 is quite inaccurate and noisy. Adjust to own measurements.
                #endif
                #if defined(HELTEC_V2_GPS) || defined(HELTEC_V2_GPS_915) || defined(HELTEC_V2_TNC) || defined(F4GOH_1W_LoRa_Tracker) || defined(F4GOH_1W_LoRa_Tracker_LLCC68)
                    double inputDivider = (1.0 / (220.0 + 100.0)) * 100.0;  // The voltage divider is a 220k + 100k resistor in series, 100k on the low side. 
                    return (voltage / inputDivider) + 0.285; // Yes, this offset is excessive, but the ADC on the ESP32 is quite inaccurate and noisy. Adjust to own measurements.
                #endif
            #else
                return 0.0;
            #endif
        #endif
    }

    void obtainBatteryInfo() {
        #ifdef HAS_FUEL_GAUGE_I2C
            if (fuelGaugeReady) {
                #if defined(CROWPANEL_ADVANCE_35)
                    float cellV = max17048CellVoltage();
                    float soc   = max17048CellPercent();
                    float rate  = max17048ChargeRate();
                #else
                    float cellV = fuelGauge.cellVoltage();
                    float soc   = fuelGauge.cellPercent();
                    float rate  = fuelGauge.chargeRate();
                #endif
                batteryVoltage                  = String(cellV, 2);
                batteryConnected                = (cellV > 1.5);
                batteryChargeDischargeCurrent   = String(rate, 0);

                static uint32_t lastBattLog    = 0;
                static float    lastVoltage    = 0.0f;
                static bool     lastConnected  = false;

                bool significantChange = (fabsf(cellV - lastVoltage) > 0.1f)
                                      || (lastConnected != batteryConnected);

                if (millis() - lastBattLog > 10000 || significantChange) {
                    const char *state = (rate > 10.0f) ? "CHARGING" :
                                        (rate < -10.0f) ? "DISCHARGING" : "IDLE";
                    ESP_LOGI(TAG, "%.3fV %.1f%% %+.0fmA/h %s",
                             cellV, soc, rate, state);
                    lastBattLog   = millis();
                    lastVoltage   = cellV;
                    lastConnected = batteryConnected;
                }
            } else {
                batteryVoltage   = "0.00";
                batteryConnected = false;
            }
            return;
        #endif
        #if defined(HAS_AXP192) || defined(HAS_AXP2101)
            batteryConnected = PMU.isBatteryConnect();
            if (batteryConnected) {
                batteryVoltage                  = String(readBatteryVoltage(), 2);
                batteryChargeDischargeCurrent   = String(POWER_Utils::getBatteryChargeDischargeCurrent(), 0);
            }
        #else
            batteryVoltage = String(readBatteryVoltage(), 2);
            if (batteryVoltage.toFloat() > 1.5) batteryConnected = true;
        #endif
    }

    void monitor() {
        #if defined(HAS_AXP192) || defined(HAS_AXP2101)
            if (batteryMeasurmentTime == 0 || (millis() - batteryMeasurmentTime) > 1 * 1000){
                obtainBatteryInfo();
                POWER_Utils::handleChargingLed();
                batteryMeasurmentTime = millis();
            }
        #elif defined(BATTERY_PIN) || defined(HAS_FUEL_GAUGE_I2C)
            if (batteryMeasurmentTime == 0 || (millis() - batteryMeasurmentTime) > 30 * 1000){ //At least 30 seconds have to pass between measurements
                #ifdef ADC_CTRL
                    switch(measuringState){
                        case 0:     // Initial Measurement
                            POWER_Utils::adc_ctrl_ON();
                            adcCtrlTime = millis();
                            delay(50);
                            obtainBatteryInfo();
                            POWER_Utils::adc_ctrl_OFF();
                            measuringState = 1;
                            break;
                        case 1:     //ADC_CTRL_ON State
                            POWER_Utils::adc_ctrl_ON();
                            adcCtrlTime = millis();
                            measuringState = 2;
                            break;
                        case 2:     // Measurement State
                            if((millis() - adcCtrlTime) > 50){ //At least 50ms have to pass after ADC_Ctrl Mosfet is turned on for voltage to stabilize
                                obtainBatteryInfo();
                                POWER_Utils::adc_ctrl_OFF();
                                measuringState = 1;
                                
                                if (batteryVoltage.toFloat() < (Config.battery.sleepVoltage - 0.1)) {
                                    displayShow("!BATTERY!", "", "LOW BATTERY VOLTAGE!",5000);
                                    POWER_Utils::shutdown();
                                }
                            }
                            break;
                    }
                #else
                    obtainBatteryInfo();
                #endif
                batteryMeasurmentTime = millis();
            }
        #endif
    }

}
