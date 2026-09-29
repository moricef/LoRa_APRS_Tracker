/* Copyright (C) 2026 Fabrice Morel - F4MLV
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

#include <esp_log.h>
#include <esp_timer.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "power_save.h"
#include "battery_utils.h"
#include "ble_utils.h"

static const char *TAG = "PowerSave";

extern bool screenDimmed;         // lvgl_ui.cpp
extern bool WiFiUserDisabled;     // wifi_utils.cpp
extern bool bluetoothActive;      // bluetooth_utils.cpp

namespace POWER_Save {

    // Duree du ralentissement. 50 ms laisse la tache idle executer plusieurs
    // WFI entre deux passages de boucle, tout en restant tres inferieur au
    // budget du buffer UART du GPS (266 ms a 38400 bauds avec 1024 octets).
    static const uint32_t THROTTLE_MS = 50;

    // Au-dela de cet ecart entre deux passages, la boucle a ete retenue
    // ailleurs (ecriture flash, rendu carte, modale) : imputer ce temps a une
    // cause serait faux, on ne le compte pas.
    static const uint64_t MAX_ATTRIBUTABLE_US = 5ULL * 1000000ULL;

    static bool     _enabled        = true;
    static bool     _throttling     = false;
    static uint64_t _throttledUs    = 0;
    static uint64_t _measuredUs     = 0;
    static uint64_t _blockedUs[BLK_COUNT] = { 0 };
    static uint64_t _lastCheckUs    = 0;

    // Rythme de boucle : c'est lui qui montre l'effet du ralentissement.
    // Sans throttle la boucle tourne a plusieurs centaines de tours par
    // seconde en attente active ; avec, elle tombe vers 1000/THROTTLE_MS.
    static uint32_t _loopCount      = 0;
    static uint32_t _loopsPerSec    = 0;
    static uint64_t _rateWindowUs   = 0;

    // Renvoie BLK_COUNT quand rien ne bloque, sinon la premiere cause trouvee.
    static Blocker currentBlocker() {
        if (!_enabled)                                   return BLK_DISABLED;
        if (!screenDimmed)                               return BLK_SCREEN_ON;
        if (BLE_Utils::getConnectedDeviceAddress().length() > 0)
                                                         return BLK_BLE_CLIENT;
        if (!WiFiUserDisabled || WiFi.status() == WL_CONNECTED)
                                                         return BLK_WIFI_ON;
        if (bluetoothActive && !BLE_Utils::isSleeping())  return BLK_BLE_ON;
        if (BATTERY_Utils::isExternallyPowered())         return BLK_EXTERNAL_POWER;
        // Pas de condition sur la radio. La reception est sur interruption
        // (DIO1 -> setFlag), la trame attend dans le FIFO du SX1262 et 50 ms de
        // retard ne la perdent pas. L'emission est bloquante dans
        // sendNewPacket(), donc la boucle n'atteint pas loopEnd() pendant un TX.
        // operationDone ne signifie pas "radio occupee" mais "quelque chose a
        // lire" : le tester ici bloquait le ralentissement des la premiere trame.
        return BLK_COUNT;
    }

    void setEnabled(bool on) {
        _enabled = on;
        if (!on) _throttling = false;
    }

    bool enabled() { return _enabled; }

    void loopEnd() {
        const uint64_t nowUs = (uint64_t)esp_timer_get_time();
        const Blocker  blk   = currentBlocker();
        const bool     idle  = (blk == BLK_COUNT);

        if (_lastCheckUs != 0) {
            const uint64_t elapsed = nowUs - _lastCheckUs;
            if (elapsed <= MAX_ATTRIBUTABLE_US) {
                _measuredUs += elapsed;
                if (idle) _throttledUs      += elapsed;
                else      _blockedUs[blk]   += elapsed;
            }
        }

        if (idle != _throttling) {
            _throttling = idle;
            if (idle) ESP_LOGD(TAG, "Loop throttled (idle)");
            else      ESP_LOGD(TAG, "Loop resumed (%s)", blockerName(blk));
        }

        // Horodater AVANT l'attente : l'intervalle mesure doit aller du debut
        // d'un passage au debut du suivant, attente comprise. En le prenant
        // apres, les 50 ms de vTaskDelay echappaient au comptage et les
        // pourcentages portaient sur une fraction du temps reel.
        _lastCheckUs = nowUs;

        if (idle) {
            vTaskDelay(pdMS_TO_TICKS(THROTTLE_MS));
        } else {
            yield();
        }

        // Rythme de boucle sur une fenetre d'une seconde.
        _loopCount++;
        if (_rateWindowUs == 0) _rateWindowUs = nowUs;
        if (nowUs - _rateWindowUs >= 1000000ULL) {
            _loopsPerSec  = _loopCount;
            _loopCount    = 0;
            _rateWindowUs = nowUs;
        }
    }

    bool isThrottling() { return _throttling; }

    uint8_t pctThrottled() {
        if (_measuredUs == 0) return 0;
        return (uint8_t)((_throttledUs * 100ULL) / _measuredUs);
    }

    uint8_t blockedPct(Blocker b) {
        if (_measuredUs == 0 || b >= BLK_COUNT) return 0;
        return (uint8_t)((_blockedUs[b] * 100ULL) / _measuredUs);
    }

    Blocker topBlocker() {
        Blocker  best    = BLK_COUNT;
        uint64_t bestUs  = 0;
        for (int i = 0; i < BLK_COUNT; ++i) {
            if (_blockedUs[i] > bestUs) {
                bestUs = _blockedUs[i];
                best   = (Blocker)i;
            }
        }
        return best;
    }

    uint32_t loopsPerSecond() { return _loopsPerSec; }

    String statusLine() {
        const Blocker top = topBlocker();
        String out = "throttled " + String(pctThrottled()) + "% ("
                   + String(_loopsPerSec) + " loop/s)";
        if (top != BLK_COUNT) {
            out += ", top blocker: " + String(blockerName(top))
                 + " " + String(blockedPct(top)) + "%";
        }
        return out;
    }

    const char* blockerName(Blocker b) {
        switch (b) {
            case BLK_DISABLED:       return "disabled";
            case BLK_SCREEN_ON:      return "screen on";
            case BLK_BLE_CLIENT:     return "BLE client";
            case BLK_WIFI_ON:        return "WiFi on";
            case BLK_BLE_ON:         return "BLE on";
            case BLK_EXTERNAL_POWER: return "external power";
            default:                 return "none";
        }
    }

}
