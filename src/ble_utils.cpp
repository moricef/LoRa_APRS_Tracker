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

#ifdef UNIT_TEST
#include "mock_esp_log.h"
#else
#include <esp_log.h>
#endif
#include <NimBLEDevice.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "configuration.h"
#include "lora_utils.h"
#include "kiss_utils.h"
#include "ble_utils.h"
#include "wifi_utils.h"
#include "display.h"
#ifdef USE_LVGL_UI
#include "lvgl_ui.h"
#endif

#define BLE_CHUNK_SIZE       512
#define MAX_KISS_BUFFER      1024
#define MAX_TNC2_LINE        512
#define TNC2_QUEUE_CAPACITY  8


// APPLE - APRS.fi app
#define SERVICE_UUID_0            "00000001-ba2a-46c9-ae49-01b0961f68bb"
#define CHARACTERISTIC_UUID_TX_0  "00000003-ba2a-46c9-ae49-01b0961f68bb"
#define CHARACTERISTIC_UUID_RX_0  "00000002-ba2a-46c9-ae49-01b0961f68bb"

// ANDROID - BLE Terminal app (Serial Bluetooth Terminal from Playstore)
#define SERVICE_UUID_1            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
// Nordic UART Service (NUS), from the peripheral's point of view:
// the central writes to RX (...0002) and subscribes to TX (...0003).
#define CHARACTERISTIC_UUID_RX_1  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX_1  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer               *pServer;
BLECharacteristic       *pCharacteristicTx;
BLECharacteristic       *pCharacteristicRx;

extern Configuration    Config;
extern Beacon           *currentBeacon;
extern bool             bluetoothConnected;
extern bool             bluetoothActive;

static const char *TAG = "BLE";

bool    shouldSendBLEtoLoRa     = false;
String  BLEToLoRaPacket         = "";
String  kissSerialBuffer        = "";
String  tnc2SerialBuffer        = "";
String  bleConnectedDeviceAddr  = "";  // Connected device MAC address
String  bleConnectedDeviceName  = "";  // Connected device name (from GAP)
bool    bleNeedToReadName       = false;  // Flag to read name after connection
NimBLEAddress bleConnectedPeerAddr;  // Peer address for client connection

// BLE state flags
bool        bleSleeping         = false;    // BLE is currently stopped (for WiFi coexistence)
bool        bleWakeRequested    = false;    // Deferred wake flag (set from LVGL, processed in main loop)

// BLE callbacks run in the NimBLE host task while sendToLoRa() runs in the
// Arduino loop. Keep complete TNC2 lines in a bounded queue so consecutive
// writes cannot overwrite one another.
static SemaphoreHandle_t tnc2QueueMutex = nullptr;
static String tnc2Queue[TNC2_QUEUE_CAPACITY];
static uint8_t tnc2QueueHead = 0;
static uint8_t tnc2QueueTail = 0;
static uint8_t tnc2QueueCount = 0;
static bool tnc2DiscardUntilDelimiter = false;

static void resetTnc2Input() {
    if (tnc2QueueMutex == nullptr) {
        tnc2QueueMutex = xSemaphoreCreateMutex();
    }
    if (tnc2QueueMutex == nullptr ||
        xSemaphoreTake(tnc2QueueMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }

    tnc2SerialBuffer = "";
    tnc2DiscardUntilDelimiter = false;
    for (uint8_t i = 0; i < TNC2_QUEUE_CAPACITY; ++i) tnc2Queue[i] = "";
    tnc2QueueHead = 0;
    tnc2QueueTail = 0;
    tnc2QueueCount = 0;
    xSemaphoreGive(tnc2QueueMutex);
}

static bool enqueueTnc2Line(const String& line) {
    if (tnc2QueueMutex == nullptr ||
        xSemaphoreTake(tnc2QueueMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return false;
    }

    bool queued = false;
    if (tnc2QueueCount < TNC2_QUEUE_CAPACITY) {
        tnc2Queue[tnc2QueueTail] = line;
        tnc2QueueTail = (tnc2QueueTail + 1) % TNC2_QUEUE_CAPACITY;
        ++tnc2QueueCount;
        queued = true;
    }
    xSemaphoreGive(tnc2QueueMutex);
    return queued;
}

static bool dequeueTnc2Line(String& line) {
    if (tnc2QueueMutex == nullptr ||
        xSemaphoreTake(tnc2QueueMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return false;
    }

    bool available = false;
    if (tnc2QueueCount > 0) {
        line = tnc2Queue[tnc2QueueHead];
        tnc2Queue[tnc2QueueHead] = "";
        tnc2QueueHead = (tnc2QueueHead + 1) % TNC2_QUEUE_CAPACITY;
        --tnc2QueueCount;
        available = true;
    }
    xSemaphoreGive(tnc2QueueMutex);
    return available;
}

static void finishTnc2Line() {
    if (tnc2DiscardUntilDelimiter) {
        tnc2DiscardUntilDelimiter = false;
        tnc2SerialBuffer = "";
        return;
    }
    if (tnc2SerialBuffer.isEmpty()) return;

    if (!KISS_Utils::validateTNC2Frame(tnc2SerialBuffer)) {
        ESP_LOGW(TAG, "Ignoring invalid BLE TNC2 line: %s", tnc2SerialBuffer.c_str());
    } else if (!enqueueTnc2Line(tnc2SerialBuffer)) {
        ESP_LOGW(TAG, "BLE TNC2 receive queue full, dropping line");
    }
    tnc2SerialBuffer = "";
}

static void receiveTnc2Bytes(const std::string& receivedData) {
    for (uint8_t c : receivedData) {
        if (c == '\r' || c == '\n') {
            finishTnc2Line();
            continue;
        }
        if (tnc2DiscardUntilDelimiter) continue;
        if (tnc2SerialBuffer.length() >= MAX_TNC2_LINE) {
            ESP_LOGW(TAG, "BLE TNC2 line exceeds %d bytes, discarding", MAX_TNC2_LINE);
            tnc2SerialBuffer = "";
            tnc2DiscardUntilDelimiter = true;
            continue;
        }
        tnc2SerialBuffer += (char)c;
    }
}

class MyServerCallbacks : public NimBLEServerCallbacks {
    // NimBLE 1.4 invokes both overloads. Handle the descriptor overload only
    // so connection state is not processed twice.
    void onConnect(NimBLEServer*) override {}

    void onConnect(NimBLEServer*, ble_gap_conn_desc* desc) override {
        bluetoothConnected = true;
        // Get connected device MAC address from connection descriptor
        bleConnectedPeerAddr = NimBLEAddress(desc->peer_ota_addr);
        bleConnectedDeviceAddr = bleConnectedPeerAddr.toString().c_str();
        bleConnectedDeviceName = "";  // Will be read later
        bleNeedToReadName = true;  // Signal to read name in loop

        ESP_LOGI(TAG, "BLE Client Connected: %s", bleConnectedDeviceAddr.c_str());
    }

    void onDisconnect(NimBLEServer* pServer) override {}

    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        bluetoothConnected = false;
        bleConnectedDeviceAddr = "";
        bleConnectedDeviceName = "";
        bleNeedToReadName = false;
        resetTnc2Input();
        kissSerialBuffer = "";
        ESP_LOGI(TAG, "BLE client disconnected");
        pServer->startAdvertising();
    }
};

class MyCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic) override {
        if (Config.bluetooth.useKISS) {   // KISS (AX.25)
            std::string receivedData = pCharacteristic->getValue();

            for (uint8_t c : receivedData) {                                                // save all received data from buffer
                kissSerialBuffer += (char)c;
            }
            if (kissSerialBuffer.length() > MAX_KISS_BUFFER) {                              // buffer overflow protection
                kissSerialBuffer = "";
                return;
            }

            int maxIterations = 10;                                                        // infinite loop protection
            while (maxIterations-- > 0) {

                if (kissSerialBuffer.length() == 0) break;                                  // empty buffer protection

                int fendIndex = -1;
                if (kissSerialBuffer.charAt(0) == (char)KissChar::FEND) {                   // starts with FEND???
                    for (int i = 1; i < kissSerialBuffer.length(); i++) {                   // look for next FEND
                        if (kissSerialBuffer.charAt(i) == (char)KissChar::FEND) {
                            fendIndex = i;
                            break;
                        }
                    }
                } else {
                    int firstFendIndex = kissSerialBuffer.indexOf((char)KissChar::FEND);    // find first FEND byte to discard leading corrupted bytes
                    if (firstFendIndex != -1) {
                        kissSerialBuffer.remove(0, firstFendIndex);                         // delete corrupted data before FEND 
                    } else {
                        kissSerialBuffer = "";                                              // if no FEND found, delete all
                        break;
                    }
                    continue;
                }

                if (fendIndex == -1) {                                                      // exit: no FEND byte to process the kissSerialBuffer (yet)
                    break;
                }

                String frame = kissSerialBuffer.substring(0, fendIndex + 1);                // extract full frame (With FEND at start and end)
                kissSerialBuffer.remove(0, fendIndex + 1);
                
                if (frame.length() >= 4) {                                                  // FEND | CMD | DATA | FEND
                    bool isDataFrame    = false;
                    BLEToLoRaPacket     = KISS_Utils::decodeKISS(frame, isDataFrame);
                    if (isDataFrame) shouldSendBLEtoLoRa = true;
                }
            }
        } else {                            // TNC2
            std::string receivedData = pCharacteristic->getValue();
            receiveTnc2Bytes(receivedData);
        }
    }

    // NimBLE 1.4 invokes this after the one-argument overload above.
    void onWrite(NimBLECharacteristic*, ble_gap_conn_desc*) override {}
};

static MyCallbacks rxCallbacks;

namespace BLE_Utils {

    void stop() {
        if (BLEDevice::getInitialized()) {
            // clearAll is required: otherwise setup() adds duplicate services
            // and the client can subscribe to an old characteristic while the
            // firmware notifies through the newly-created one.
            BLEDevice::deinit(true);
        }
        pServer = nullptr;
        pCharacteristicTx = nullptr;
        pCharacteristicRx = nullptr;
        bluetoothConnected = false;
        resetTnc2Input();
        kissSerialBuffer = "";
        BLEToLoRaPacket = "";
        shouldSendBLEtoLoRa = false;
    }

    void setup() {
        bleSleeping = false;
        resetTnc2Input();

        String BLEid = Config.bluetooth.deviceName;
        BLEDevice::init(BLEid.c_str());
        BLEDevice::setPower(ESP_PWR_LVL_P3);  // Moderate power for coexistence
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(new MyServerCallbacks());

        BLEService *pService = nullptr;

        //  KISS (AX.25) or TNC2
        bool useKISS = Config.bluetooth.useKISS;
        pService = pServer->createService(useKISS ? SERVICE_UUID_0 : SERVICE_UUID_1);
        pCharacteristicTx = pService->createCharacteristic(useKISS ? CHARACTERISTIC_UUID_TX_0 : CHARACTERISTIC_UUID_TX_1, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
        pCharacteristicRx = pService->createCharacteristic(useKISS ? CHARACTERISTIC_UUID_RX_0 : CHARACTERISTIC_UUID_RX_1, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);

        if (pService != nullptr) {
            pCharacteristicRx->setCallbacks(&rxCallbacks);
            pService->start();

            BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
            pAdvertising->addServiceUUID(useKISS ? SERVICE_UUID_0 : SERVICE_UUID_1);

            pServer->getAdvertising()->setScanResponse(true);
            pServer->getAdvertising()->setMinPreferred(0x06);
            pServer->getAdvertising()->setMaxPreferred(0x0C);
            pAdvertising->start();
            ESP_LOGI(TAG, "BLE %s service ready; waiting for central",
                     useKISS ? "KISS" : "TNC2/NUS");
        } else {
            ESP_LOGE(TAG, "Failed to create BLE service");
        }
    }

    void sendToLoRa() {
        if (!Config.bluetooth.useKISS) {
            String line;
            if (!dequeueTnc2Line(line)) return;

            ESP_LOGD(TAG, "TNC2 Tx %s", line.c_str());
            #ifdef USE_LVGL_UI
                LVGL_UI::showTxPacket(line.c_str());
            #else
                displayShow("BLE Tx >>", "", line, 1000);
            #endif
            LoRa_Utils::sendNewPacket(line);
            return;
        }

        if (!shouldSendBLEtoLoRa) return;

        ESP_LOGD(TAG, "Tx %s", BLEToLoRaPacket.c_str());
        #ifdef USE_LVGL_UI
            LVGL_UI::showTxPacket(BLEToLoRaPacket.c_str());
        #else
            displayShow("BLE Tx >>", "", BLEToLoRaPacket, 1000);
        #endif
        LoRa_Utils::sendNewPacket(BLEToLoRaPacket);
        BLEToLoRaPacket = "";
        shouldSendBLEtoLoRa = false;
    }

    static size_t notificationChunkSize() {
        size_t chunkSize = BLE_CHUNK_SIZE;
        if (pServer != nullptr) {
            std::vector<uint16_t> peers = pServer->getPeerDevices();
            for (uint16_t peer : peers) {
                uint16_t mtu = pServer->getPeerMTU(peer);
                size_t payloadSize = mtu > 3 ? mtu - 3 : 20;
                if (payloadSize < chunkSize) chunkSize = payloadSize;
            }
        }
        return chunkSize > 0 ? chunkSize : 20;
    }

    static void notifyBytes(const uint8_t* data, size_t length) {
        if (pCharacteristicTx == nullptr || data == nullptr || length == 0) return;

        const size_t chunkSize = notificationChunkSize();
        for (size_t offset = 0; offset < length; offset += chunkSize) {
            size_t lengthRemaining = length - offset;
            size_t currentSize = lengthRemaining < chunkSize ? lengthRemaining : chunkSize;
            pCharacteristicTx->setValue(data + offset, currentSize);
            pCharacteristicTx->notify();
            delay(3);
        }
    }

    void txToPhoneOverBLE(const String& frame) {
        if (Config.bluetooth.useKISS) {   // KISS (AX.25)
            const String kissEncodedFrame = KISS_Utils::encodeKISS(frame);

            notifyBytes(reinterpret_cast<const uint8_t*>(kissEncodedFrame.c_str()),
                        kissEncodedFrame.length());
        } else {        // TNC2
            String line = frame;
            line += '\n';
            notifyBytes(reinterpret_cast<const uint8_t*>(line.c_str()), line.length());
        }
    }

    void sendToPhone(const String& packet) {
        if (!packet.isEmpty() && bluetoothConnected) {
            ESP_LOGD(TAG, "Rx %s", packet.c_str());
            String receivedPacketString = "";
            for (int i = 0; i < packet.length(); i++) receivedPacketString += packet[i];
            txToPhoneOverBLE(receivedPacketString);
        }
    }

    String getConnectedDeviceAddress() {
        return bleConnectedDeviceAddr;
    }

    String getConnectedDeviceName() {
        return bleConnectedDeviceName;
    }

    // Try to read the device name from the connected peer
    // Note: Most smartphones don't allow reverse client connections, so this usually fails
    // Keeping the function but it's essentially a no-op for now
    void tryReadDeviceName() {
        // Disabled - smartphones typically don't expose their GAP service to peripherals
        // The MAC address will be displayed instead
        bleNeedToReadName = false;
    }

    // Process deferred BLE wake request — call from main loop
    void checkEcoMode() {
        if (!bleWakeRequested || !bleSleeping) {
            bleWakeRequested = false;
            return;
        }
        bleWakeRequested = false;

        // BT controller needs ~47KB SRAM internal to init
        size_t freeDram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (freeDram < 50000) {
            ESP_LOGW(TAG, "Not enough SRAM to restart BLE (%u bytes free, need 50000)", (unsigned)freeDram);
            return;
        }

        bleSleeping = false;
        WIFI_Utils::stop();
        setup();
        ESP_LOGI(TAG, "BLE restarted (WiFi stopped)");
    }

    // Request BLE wake from eco mode (safe to call from LVGL callbacks)
    void wake() {
        if (!bleSleeping) return;
        bleWakeRequested = true;
        ESP_LOGI(TAG, "Wake requested (deferred to main loop)");
    }

    // Check if BLE is sleeping
    bool isSleeping() {
        return bleSleeping;
    }

}
