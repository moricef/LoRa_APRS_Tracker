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

#include <esp_log.h>
#include <esp_bt.h>
#include <esp_heap_caps.h>
#include <nvs_flash.h>
// Arduino's initArduino() must retain BLE memory for this native IDF user.
// Without this startup hook it returns the controller's data/BSS to the heap.
#include <esp32-hal-alloc-ble-mem.h>
#include <atomic>
#include <cstring>
extern "C" {
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"
void ble_store_config_init(void);
}
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
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

// BLE-KISS service for AX.25 frames transported with KISS framing.
#define SERVICE_UUID_0            "00000001-ba2a-46c9-ae49-01b0961f68bb"
#define CHARACTERISTIC_UUID_TX_0  "00000003-ba2a-46c9-ae49-01b0961f68bb"
#define CHARACTERISTIC_UUID_RX_0  "00000002-ba2a-46c9-ae49-01b0961f68bb"

// ANDROID - BLE Terminal app (Serial Bluetooth Terminal from Playstore)
#define SERVICE_UUID_1            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
// Nordic UART Service (NUS), from the peripheral's point of view:
// the central writes to RX (...0002) and subscribes to TX (...0003).
#define CHARACTERISTIC_UUID_RX_1  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX_1  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

extern Configuration Config;
extern bool bluetoothConnected;
extern bool bluetoothActive;

static const char* TAG = "BLE";
static bool bleInitialized = false;  // protected by lifecycleMutex()
static bool bleHostStarted = false;
static bool bleUseKISS = false;      // immutable while the host runs
static std::atomic<bool> bleStopping{false};
static std::atomic<bool> bleSynced{false};
static std::atomic<int> bleStartResult{0};
static std::atomic<uint16_t> bleConnection{BLE_HS_CONN_HANDLE_NONE};
static std::atomic<bool> bleSubscribed{false};
static std::atomic<uint32_t> bleSession{0};
static bool bleSleeping = false;
static bool bleWakeRequested = false;
static uint8_t ownAddressType;
static uint16_t txValueHandle;
static ble_uuid_any_t serviceUuid, txUuid, rxUuid;
static ble_gatt_chr_def characteristics[3];
static ble_gatt_svc_def services[2];

// GAP callbacks and characteristic reads run in the NimBLE host task.
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
static char peerAddress[18] = {};
static uint8_t txValue[BLE_CHUNK_SIZE];
static uint16_t txValueLength = 0;
static StaticSemaphore_t hostExitedStorage, hostReadyStorage;
static SemaphoreHandle_t hostExited = nullptr;
static SemaphoreHandle_t hostReady = nullptr;

static SemaphoreHandle_t lifecycleMutex() {
    static StaticSemaphore_t storage;
    static SemaphoreHandle_t mutex = xSemaphoreCreateMutexStatic(&storage);
    return mutex;
}

// Host callbacks never take the lifecycle mutex: stop() waits for the host.
class LifecycleLock {
public:
    LifecycleLock() { xSemaphoreTake(lifecycleMutex(), portMAX_DELAY); }
    ~LifecycleLock() { xSemaphoreGive(lifecycleMutex()); }
};

// The host assembles frames; loopTask dequeues them before sending over LoRa.
static SemaphoreHandle_t tnc2QueueMutex = nullptr;
static String tnc2Queue[TNC2_QUEUE_CAPACITY];
static uint8_t tnc2QueueHead = 0;
static uint8_t tnc2QueueTail = 0;
static uint8_t tnc2QueueCount = 0;
static String kissSerialBuffer;
static String tnc2SerialBuffer;
static bool tnc2DiscardUntilDelimiter = false;

// GATT callbacks only copy bytes. Parsing and warnings (which can write to SD)
// run in loopTask, not on the 4 KB NimBLE host stack.
struct RxWrite {
    uint32_t session;
    uint32_t stream;
    uint16_t length;
    uint8_t data[BLE_CHUNK_SIZE];
};
static constexpr size_t RX_WRITE_QUEUE_SIZE = 8;
static QueueHandle_t rxWriteQueue = nullptr;
static StaticQueue_t rxWriteQueueControl;
static uint8_t* rxWriteQueueStorage = nullptr;
static std::atomic<uint32_t> rxStream{0};
static std::atomic<uint32_t> droppedWrites{0};

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

static void receiveTnc2Bytes(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        const uint8_t c = data[i];
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

static void receiveKissBytes(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        const uint8_t c = data[i];                                                // save all received data from buffer
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
            const String decoded = KISS_Utils::decodeKISS(frame, isDataFrame);
            if (isDataFrame && !enqueueTnc2Line(decoded)) {
                ESP_LOGW(TAG, "BLE KISS receive queue full, dropping frame");
            }
        }
    }
}

static void processRxWrites() {
    static uint32_t parserSession = UINT32_MAX;
    static uint32_t parserStream = UINT32_MAX;
    auto syncParser = [&]() {
        const uint32_t session = bleSession.load();
        const uint32_t stream = rxStream.load();
        if (session != parserSession || stream != parserStream) {
            const bool interrupted = session == parserSession;
            resetTnc2Input();
            kissSerialBuffer = "";
            // A dropped chunk can end inside a line; wait for its delimiter.
            tnc2DiscardUntilDelimiter = interrupted;
            parserSession = session;
            parserStream = stream;
        }
    };
    syncParser();
    if (!rxWriteQueue) return;
    RxWrite incoming;
    for (size_t count = 0; count < RX_WRITE_QUEUE_SIZE &&
         xQueueReceive(rxWriteQueue, &incoming, 0) == pdTRUE; ++count) {
        syncParser();
        if (incoming.session != parserSession || incoming.stream != parserStream) continue;
        if (bleUseKISS) receiveKissBytes(incoming.data, incoming.length);
        else receiveTnc2Bytes(incoming.data, incoming.length);
    }
    // A disconnect or overflow during parsing invalidates partial/queued frames.
    syncParser();
    const uint32_t lost = droppedWrites.exchange(0);
    if (lost) ESP_LOGW(TAG, "BLE RX overflow: %lu writes dropped; stream reset", (unsigned long)lost);
}


static void clearConnection() {
    bleSession.fetch_add(1);
    bleConnection.store(BLE_HS_CONN_HANDLE_NONE);
    bleSubscribed.store(false);
    bluetoothConnected = false;
    portENTER_CRITICAL(&stateMux);
    peerAddress[0] = '\0';
    txValueLength = 0;
    portEXIT_CRITICAL(&stateMux);
}

static int gapEvent(ble_gap_event* event, void* arg);

static int startAdvertising() {
    if (bleStopping.load() || !bleSynced.load()) return BLE_HS_EDISABLED;
    ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &serviceUuid.u128;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) return rc;

    // The 128-bit service is in the primary payload; the name is in scan response.
    const char* name = ble_svc_gap_device_name();
    const size_t nameLength = strlen(name);
    ble_hs_adv_fields scan{};
    scan.name = reinterpret_cast<const uint8_t*>(name);
    scan.name_len = nameLength > 29 ? 29 : nameLength;
    scan.name_is_complete = nameLength <= 29;
    rc = ble_gap_adv_rsp_set_fields(&scan);
    if (rc != 0) return rc;

    ble_gap_adv_params params{};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(ownAddressType, nullptr, BLE_HS_FOREVER,
                          &params, gapEvent, nullptr);
    if (rc != 0) ESP_LOGE(TAG, "Advertising failed: %d", rc);
    return rc;
}

static int gapEvent(ble_gap_event* event, void*) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT: {
            if (event->connect.status != 0) {
                startAdvertising();
                return 0;
            }
            ble_gap_conn_desc desc{};
            const int rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) return rc;
            const uint8_t* a = desc.peer_ota_addr.val;
            char address[18];
            snprintf(address, sizeof(address), "%02x:%02x:%02x:%02x:%02x:%02x",
                     a[5], a[4], a[3], a[2], a[1], a[0]);
            portENTER_CRITICAL(&stateMux);
            memcpy(peerAddress, address, sizeof(peerAddress));
            portEXIT_CRITICAL(&stateMux);
            bleConnection.store(event->connect.conn_handle);
            bleSubscribed.store(false);
            bluetoothConnected = true;
            ESP_LOGI(TAG, "BLE client connected: %s", address);
            return 0;
        }
        case BLE_GAP_EVENT_DISCONNECT:
            clearConnection();
            ESP_LOGI(TAG, "BLE client disconnected (reason %d)", event->disconnect.reason);
            if (!bleStopping.load()) startAdvertising();
            return 0;
        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.conn_handle == bleConnection.load() &&
                event->subscribe.attr_handle == txValueHandle) {
                bleSubscribed.store(event->subscribe.cur_notify);
            }
            return 0;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            if (!bleStopping.load() && bleConnection.load() == BLE_HS_CONN_HANDLE_NONE)
                startAdvertising();
            return 0;
        case BLE_GAP_EVENT_REPEAT_PAIRING:
            return BLE_GAP_REPEAT_PAIRING_IGNORE;
        default:
            return 0;
    }
}

static int accessGatt(uint16_t conn, uint16_t attr, ble_gatt_access_ctxt* ctxt, void*) {
    RxWrite incoming;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR && attr == txValueHandle) {
        uint16_t length;
        portENTER_CRITICAL(&stateMux);
        length = txValueLength;
        memcpy(incoming.data, txValue, length);
        portEXIT_CRITICAL(&stateMux);
        return os_mbuf_append(ctxt->om, incoming.data, length) == 0
                   ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR ||
        bleStopping.load() || conn != bleConnection.load()) return BLE_ATT_ERR_UNLIKELY;

    const uint16_t length = OS_MBUF_PKTLEN(ctxt->om);
    if (length > BLE_CHUNK_SIZE) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    if (!rxWriteQueue) return BLE_ATT_ERR_INSUFFICIENT_RES;
    uint16_t copied = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, incoming.data, sizeof(incoming.data), &copied) != 0)
        return BLE_ATT_ERR_UNLIKELY;
    incoming.session = bleSession.load();
    incoming.stream = rxStream.load();
    incoming.length = copied;
    if (xQueueSend(rxWriteQueue, &incoming, 0) != pdTRUE) {
        rxStream.fetch_add(1);
        droppedWrites.fetch_add(1);
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return 0;
}

static int configureGatt() {
    int rc = ble_uuid_from_str(&serviceUuid, bleUseKISS ? SERVICE_UUID_0 : SERVICE_UUID_1);
    if (rc == 0) rc = ble_uuid_from_str(&txUuid, bleUseKISS ? CHARACTERISTIC_UUID_TX_0 : CHARACTERISTIC_UUID_TX_1);
    if (rc == 0) rc = ble_uuid_from_str(&rxUuid, bleUseKISS ? CHARACTERISTIC_UUID_RX_0 : CHARACTERISTIC_UUID_RX_1);
    if (rc != 0) return rc;
    memset(characteristics, 0, sizeof(characteristics));
    memset(services, 0, sizeof(services));
    txValueHandle = 0;
    characteristics[0].uuid = &txUuid.u;
    characteristics[0].access_cb = accessGatt;
    characteristics[0].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY;
    characteristics[0].val_handle = &txValueHandle;
    characteristics[1].uuid = &rxUuid.u;
    characteristics[1].access_cb = accessGatt;
    characteristics[1].flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP;
    services[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    services[0].uuid = &serviceUuid.u;
    services[0].characteristics = characteristics;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    rc = ble_gatts_count_cfg(services);
    return rc == 0 ? ble_gatts_add_svcs(services) : rc;
}

static void hostReset(int reason) {
    bleSynced.store(false);
    clearConnection();
    ESP_LOGW(TAG, "NimBLE host reset: %d", reason);
}

static void hostSync() {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) rc = ble_hs_id_infer_auto(0, &ownAddressType);
    bleSynced.store(rc == 0);
    if (rc == 0) rc = startAdvertising();
    bleStartResult.store(rc);
    xSemaphoreGive(hostReady);
}

static void hostTask(void*) {
    nimble_port_run();  // returns only after nimble_port_stop()
    xSemaphoreGive(hostExited);
    vTaskDelete(nullptr);
}

// Called only under lifecycleMutex(); no host callback acquires that mutex.
static bool stopHost() {
    bleStopping.store(true);
    if (bleInitialized) {
        if (bleHostStarted) {
            const int rc = nimble_port_stop();
            if (rc != 0) {
                ESP_LOGE(TAG, "NimBLE stop failed: %d", rc);
                return false;  // keep state so a later stop can retry
            }
            xSemaphoreTake(hostExited, portMAX_DELAY);
            bleHostStarted = false;
        }
        const esp_err_t err = nimble_port_deinit();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NimBLE deinit failed: %s", esp_err_to_name(err));
            return false;
        }
        bleInitialized = false;
    }
    bleSynced.store(false);
    clearConnection();
    resetTnc2Input();
    kissSerialBuffer = "";
    if (rxWriteQueue) xQueueReset(rxWriteQueue);
    return true;
}

namespace BLE_Utils {

    void stop() {
        LifecycleLock lock;
        stopHost();
    }

    void setup() {
        LifecycleLock lock;
        if (bleInitialized && !stopHost()) return;
        if (!hostExited) hostExited = xSemaphoreCreateBinaryStatic(&hostExitedStorage);
        if (!hostReady) hostReady = xSemaphoreCreateBinaryStatic(&hostReadyStorage);
        xSemaphoreTake(hostExited, 0);
        xSemaphoreTake(hostReady, 0);
        resetTnc2Input();
        if (!tnc2QueueMutex) {
            ESP_LOGE(TAG, "Cannot allocate BLE receive queue mutex");
            return;
        }
        if (!rxWriteQueue) {
            rxWriteQueueStorage = static_cast<uint8_t*>(heap_caps_malloc(
                RX_WRITE_QUEUE_SIZE * sizeof(RxWrite), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (rxWriteQueueStorage)
                rxWriteQueue = xQueueCreateStatic(RX_WRITE_QUEUE_SIZE, sizeof(RxWrite),
                                                 rxWriteQueueStorage, &rxWriteQueueControl);
            if (!rxWriteQueue) {
                heap_caps_free(rxWriteQueueStorage);
                rxWriteQueueStorage = nullptr;
                ESP_LOGE(TAG, "Cannot allocate BLE RX byte queue in PSRAM");
                return;
            }
        }
        xQueueReset(rxWriteQueue);
        rxStream.store(0);
        droppedWrites.store(0);
        bleUseKISS = Config.bluetooth.useKISS;
        bleSleeping = false;
        bleStopping.store(false);
        clearConnection();
        esp_err_t err = nvs_flash_init();
        if (err == ESP_OK) err = nimble_port_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NimBLE init failed: %s", esp_err_to_name(err));
            return;
        }
        bleInitialized = true;
        ble_hs_cfg.reset_cb = hostReset;
        ble_hs_cfg.sync_cb = hostSync;
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
        // Preserve the existing no-bonding, no-passkey behavior.
        ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
        ble_hs_cfg.sm_bonding = 0;
        ble_hs_cfg.sm_mitm = 0;
        ble_hs_cfg.sm_sc = 0;
        ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;
        ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;
        int rc = configureGatt();
        if (rc == 0) rc = ble_svc_gap_device_name_set(Config.bluetooth.deviceName.c_str());
        if (rc == 0) rc = ble_att_set_preferred_mtu(BLE_CHUNK_SIZE + 3);
        if (rc != 0) {
            ESP_LOGE(TAG, "NimBLE GATT setup failed: %d", rc);
            stopHost();
            return;
        }
        ble_store_config_init();
        esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P3);
        if (xTaskCreatePinnedToCore(hostTask, "nimble_host", CONFIG_BT_NIMBLE_HOST_TASK_STACK_SIZE,
                                   nullptr, configMAX_PRIORITIES - 4, nullptr,
                                   CONFIG_BT_NIMBLE_PINNED_TO_CORE) != pdPASS) {
            ESP_LOGE(TAG, "Cannot create NimBLE host task");
            stopHost();
            return;
        }
        bleHostStarted = true;
        if (xSemaphoreTake(hostReady, pdMS_TO_TICKS(5000)) != pdTRUE || bleStartResult.load() != 0) {
            ESP_LOGE(TAG, "NimBLE synchronization/advertising failed: %d", bleStartResult.load());
            stopHost();
            return;
        }
        ESP_LOGI(TAG, "Native ESP-IDF NimBLE %s service ready", bleUseKISS ? "KISS" : "TNC2/NUS");
    }

    void sendToLoRa() {
        processRxWrites();
        String frame;
        if (!dequeueTnc2Line(frame)) return;
        ESP_LOGD(TAG, "%s Tx %s", bleUseKISS ? "KISS" : "TNC2", frame.c_str());
        #ifdef USE_LVGL_UI
            LVGL_UI::showTxPacket(frame.c_str());
        #else
            displayShow("BLE Tx >>", "", frame, 1000);
        #endif
        LoRa_Utils::sendNewPacket(frame);
    }

    static void notifyBytes(const uint8_t* data, size_t length) {
        LifecycleLock lock;
        const uint16_t conn = bleConnection.load();
        const uint32_t session = bleSession.load();
        if (!bleInitialized || bleStopping.load() || conn == BLE_HS_CONN_HANDLE_NONE ||
            !data || length == 0) return;
        const uint16_t mtu = ble_att_mtu(conn);
        size_t chunkSize = mtu > 3 ? mtu - 3 : 20;
        if (chunkSize > BLE_CHUNK_SIZE) chunkSize = BLE_CHUNK_SIZE;
        for (size_t offset = 0; offset < length; offset += chunkSize) {
            if (bleSession.load() != session || bleConnection.load() != conn || bleStopping.load()) break;
            const size_t currentSize = (length - offset) < chunkSize ? length - offset : chunkSize;
            portENTER_CRITICAL(&stateMux);
            memcpy(txValue, data + offset, currentSize);
            txValueLength = currentSize;
            portEXIT_CRITICAL(&stateMux);
            if (bleSubscribed.load()) {
                os_mbuf* payload = ble_hs_mbuf_from_flat(data + offset, currentSize);
                if (!payload) {
                    ESP_LOGW(TAG, "No NimBLE buffer for notification");
                    break;
                }
                // NimBLE consumes payload on success and on failure.
                const int rc = ble_gatts_notify_custom(conn, txValueHandle, payload);
                if (rc != 0) {
                    ESP_LOGW(TAG, "Notification failed: %d", rc);
                    break;
                }
            }
            delay(3);
        }
    }

    void txToPhoneOverBLE(const String& frame) {
        if (bleUseKISS) {   // KISS (AX.25)
            const String kissEncodedFrame = KISS_Utils::encodeKISS(frame);
            if (kissEncodedFrame.isEmpty()) return;

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
        char address[sizeof(peerAddress)];
        portENTER_CRITICAL(&stateMux);
        memcpy(address, peerAddress, sizeof(address));
        portEXIT_CRITICAL(&stateMux);
        return String(address);
    }

    String getConnectedDeviceName() {
        return "";  // smartphones do not expose their GAP name to the peripheral
    }

    void tryReadDeviceName() {
        // Preserve the existing no-op; the UI displays the peer address.
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
