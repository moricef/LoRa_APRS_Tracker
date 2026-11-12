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

#ifndef WIFI_UTILS_H_
#define WIFI_UTILS_H_

#include <Arduino.h>
#include <IPAddress.h>


namespace WIFI_Utils {

    void checkWiFi();
    void startBlockingWebConfig();
    void startStationMode();
    void stop();
    void setup();
    bool needsWebConfig();
    bool isConnected();                 // station associated with an IP address
    String getLocalIP();                // station address, "0.0.0.0" when none
    int getRSSI();                      // station RSSI in dBm, 0 when not associated
    uint8_t getAPStationCount();        // clients connected to the access point
    bool resolveHost(const char* host, IPAddress& result);   // blocking DNS lookup (IPv4)
    String getStatusLine();
    bool startAPModeNonBlocking();
    String getAPName();

}

#endif
