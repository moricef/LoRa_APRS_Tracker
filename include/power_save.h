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

#ifndef POWER_SAVE_H_
#define POWER_SAVE_H_

#include <Arduino.h>

// Economiseur de boucle principale.
//
// La boucle se termine par yield(), qui sur Arduino ESP32 est un vTaskDelay(0) :
// elle rend la main sans jamais laisser la tache idle de FreeRTOS executer un
// WFI. Le coeur tourne donc en attente active en permanence. Quand l'appareil
// est au repos (ecran eteint, WiFi et BLE coupes, sur batterie, radio au calme)
// on remplace ce yield() par un vTaskDelay de quelques dizaines de ms, ce qui
// laisse la tache idle arreter l'horloge du coeur entre les ticks.
//
// Le trace GNSS n'est pas interrompu : GPXWriter::addPoint() n'est appele qu'au
// moment d'une transmission APRS (station_utils.cpp), a la cadence du smart
// beacon, et la reception LoRa est sur interruption (DIO1) donc aucune trame
// n'est perdue. La seule contrainte est le buffer UART du GPS, dimensionne en
// consequence dans gps_utils.cpp.
namespace POWER_Save {

    // Cause qui maintient l'appareil eveille, dans l'ordre ou les conditions
    // sont testees. Sert a repondre a "l'economiseur ne fait rien" autrement
    // que par un pourcentage global : on sait laquelle bloque.
    enum Blocker : uint8_t {
        BLK_DISABLED = 0,
        BLK_SCREEN_ON,
        BLK_BLE_CLIENT,
        BLK_WIFI_ON,
        BLK_BLE_ON,
        BLK_EXTERNAL_POWER,
        BLK_COUNT
    };

    void setEnabled(bool on);
    bool enabled();

    // A appeler a la toute fin de loop(), en remplacement du yield().
    void loopEnd();

    // Instrumentation, lue par l'UI de diagnostic et le heartbeat SD.
    bool        isThrottling();
    uint8_t     pctThrottled();            // % du temps ecoule passe ralenti
    uint8_t     blockedPct(Blocker b);     // % du temps ou cette cause bloquait
    Blocker     topBlocker();              // cause dominante
    const char* blockerName(Blocker b);
    uint32_t    loopsPerSecond();          // rythme de boucle, mesure glissante

    // Une ligne de synthese pour le heartbeat SD et la console.
    // Ex: "throttled 62% (3.1 loop/s), top blocker: screen on 31%"
    String      statusLine();

}

#endif
