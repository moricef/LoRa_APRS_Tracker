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

#include "kiss_utils.h"
#ifdef UNIT_TEST
#include "mock_esp_log.h"
#else
#include <esp_log.h>
#endif


namespace KISS_Utils {

    static bool validateAX25Address(const String& address, bool isRelay, const char*& reason) {
        String value = address;
        int starIndex = value.indexOf('*');
        if (starIndex != -1) {
            if (!isRelay || starIndex != value.length() - 1) {
                reason = "invalid repeated flag";
                return false;
            }
            value.remove(starIndex);
        }

        int separatorIndex = value.indexOf('-');
        int callsignLength = separatorIndex == -1 ? value.length() : separatorIndex;
        if (callsignLength < 1 || callsignLength > 6) {
            reason = "callsign must contain 1-6 characters";
            return false;
        }
        for (int i = 0; i < callsignLength; ++i) {
            char c = value.charAt(i);
            if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) {
                reason = "callsign must be uppercase alphanumeric";
                return false;
            }
        }

        if (separatorIndex != -1) {
            String suffix = value.substring(separatorIndex + 1);
            if (suffix.isEmpty() || suffix.length() > 2) {
                reason = "SSID must be numeric from 0 to 15";
                return false;
            }
            for (int i = 0; i < suffix.length(); ++i) {
                if (suffix.charAt(i) < '0' || suffix.charAt(i) > '9') {
                    reason = "SSID must be numeric from 0 to 15";
                    return false;
                }
            }
            if (suffix.toInt() > 15) {
                reason = "SSID exceeds 15";
                return false;
            }
        }
        return true;
    }

    bool validateTNC2Frame(const String& tnc2FormattedFrame) {
        int colonPos        = tnc2FormattedFrame.indexOf(':');
        int greaterThanPos  = tnc2FormattedFrame.indexOf('>');
        return (colonPos != -1) && (greaterThanPos != -1) && (colonPos > greaterThanPos);
    }

    bool validateKISSFrame(const String& kissFormattedFrame) {
        return kissFormattedFrame.charAt(0) == (char)KissChar::FEND && kissFormattedFrame.charAt(kissFormattedFrame.length() - 1) == (char)KissChar::FEND;
    }

    String decodeAddressAX25(const String& ax25Address, bool& isLastAddress, bool isRelay) {
        String address = "";
        for (int i = 0; i < 6; ++i) {
            uint8_t currentCharacter = ax25Address.charAt(i);
            currentCharacter >>= 1;
            if (currentCharacter != ' ') address += (char)currentCharacter;
        }
        auto ssidChar           = (uint8_t)ax25Address.charAt(6);
        bool hasBeenDigipited   = ssidChar & HAS_BEEN_DIGIPITED_MASK;
        isLastAddress           = ssidChar & IS_LAST_ADDRESS_POSITION_MASK;
        ssidChar >>= 1;

        int ssid = 0b1111 & ssidChar;
        if (ssid) {
            address += '-';
            address += ssid;
        }
        if (isRelay && hasBeenDigipited) address += '*';
        return address;
    }

    String decapsulateKISS(const String& frame) {
        String ax25Frame = "";
        for (int i = 2; i < frame.length() - 1; ++i) {
            char currentChar = frame.charAt(i);
            if (currentChar == (char)KissChar::FESC) {
                char nextChar = frame.charAt(i + 1);
                if (nextChar == (char)KissChar::TFEND) {
                    ax25Frame += (char)KissChar::FEND;
                } else if (nextChar == (char)KissChar::TFESC) {
                    ax25Frame += (char)KissChar::FESC;
                }
                i++;
            } else {
                ax25Frame += currentChar;
            }
        }
        return ax25Frame;
    }

    String encapsulateKISS(const String& ax25Frame, uint8_t command) {
        String kissFrame = "";
        kissFrame += (char)KissChar::FEND;
        kissFrame += (char)(0x0f & command);

        for (int i = 0; i < ax25Frame.length(); ++i) {
            char currentChar = ax25Frame.charAt(i);
            if (currentChar == (char)KissChar::FEND) {
                kissFrame += (char)KissChar::FESC;
                kissFrame += (char)KissChar::TFEND;
            } else if (currentChar == (char)KissChar::FESC) {
                kissFrame += (char)KissChar::FESC;
                kissFrame += (char)KissChar::TFESC;
            } else {
                kissFrame += currentChar;
            }
        }
        kissFrame += (char)KissChar::FEND; // end of frame
        return kissFrame;
    }

    String encodeAddressAX25(String address) {
        bool hasBeenDigipited = address.indexOf('*') != -1;
        if (address.indexOf('-') == -1) {
            if (hasBeenDigipited) address = address.substring(0, address.length() - 1);
            address += "-0";
        }

        int separatorIndex  = address.indexOf('-');
        int ssid            = address.substring(separatorIndex + 1).toInt();
        String kissAddress  = "";
        for (int i = 0; i < 6; ++i) {
            char addressChar = ' ';
            if (address.length() > i && i < separatorIndex) addressChar = address.charAt(i);
            kissAddress += (char)(addressChar << 1);
        }
        kissAddress += (char)((ssid << 1) | 0b01100000 | (hasBeenDigipited ? HAS_BEEN_DIGIPITED_MASK : 0));
        return kissAddress;
    }

    String decodeKISS(const String& inputFrame, bool& dataFrame) {
        String frame = "";
        if (KISS_Utils::validateKISSFrame(inputFrame)) {
            dataFrame = inputFrame.charAt(1) == KissCmd::Data;
            if (dataFrame) {
                String ax25Frame    = decapsulateKISS(inputFrame);
                bool isLastAddress         = false;
                String dstAddr      = decodeAddressAX25(ax25Frame.substring(0, 7), isLastAddress, false);
                String srcAddr      = decodeAddressAX25(ax25Frame.substring(7, 14), isLastAddress, false);

                frame = srcAddr + ">" + dstAddr;

                int digiInfoIndex = 14;
                while (!isLastAddress && digiInfoIndex + 7 < ax25Frame.length()) {
                    String digiAddr = decodeAddressAX25(ax25Frame.substring(digiInfoIndex, digiInfoIndex + 7), isLastAddress, true);
                    frame += ',' + digiAddr;
                    digiInfoIndex += 7;
                }
                frame += ':';
                frame += ax25Frame.substring(digiInfoIndex + 2);
            } else {
                frame += inputFrame;
            }
        }
        return frame;
    }

    String encodeKISS(const String& frame) {
        if (!KISS_Utils::validateTNC2Frame(frame)) {
            ESP_LOGW("KISS", "Cannot encode malformed TNC2 frame");
            return "";
        }

        int colonIndex = frame.indexOf(':');
        int greaterThanIndex = frame.indexOf('>');
        String source = frame.substring(0, greaterThanIndex);
        String headerRest = frame.substring(greaterThanIndex + 1, colonIndex);
        int commaIndex = headerRest.indexOf(',');
        String destination = commaIndex == -1 ? headerRest : headerRest.substring(0, commaIndex);
        const char* reason = nullptr;
        if (!validateAX25Address(source, false, reason)) {
            ESP_LOGW("KISS", "Cannot encode source '%s': %s", source.c_str(), reason);
            return "";
        }
        if (!validateAX25Address(destination, false, reason)) {
            ESP_LOGW("KISS", "Cannot encode destination '%s': %s", destination.c_str(), reason);
            return "";
        }

        String ax25Frame = encodeAddressAX25(destination) + encodeAddressAX25(source);
        while (commaIndex != -1) {
            int nextCommaIndex = headerRest.indexOf(',', commaIndex + 1);
            String relay = nextCommaIndex == -1
                ? headerRest.substring(commaIndex + 1)
                : headerRest.substring(commaIndex + 1, nextCommaIndex);
            if (!validateAX25Address(relay, true, reason)) {
                ESP_LOGW("KISS", "Cannot encode relay '%s': %s", relay.c_str(), reason);
                return "";
            }
            ax25Frame += encodeAddressAX25(relay);
            commaIndex = nextCommaIndex;
        }
        auto lastAddressChar = (uint8_t)ax25Frame.charAt(ax25Frame.length() - 1);
        ax25Frame.setCharAt(ax25Frame.length() - 1, (char)(lastAddressChar | IS_LAST_ADDRESS_POSITION_MASK));
        ax25Frame += (char)AX25Char::ControlField;
        ax25Frame += (char)AX25Char::InformationField;
        ax25Frame += frame.substring(colonIndex + 1);
        return encapsulateKISS(ax25Frame, KissCmd::Data);
    }

}
