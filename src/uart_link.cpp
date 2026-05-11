/* UART Link — S3 side of C3↔S3 binary protocol (Arduino / PlatformIO)
 *
 * Uses shared uart_proto.h/.c from lib/uart_proto/ (C, portable).
 * HardwareSerial(2) at 460800 baud on pins from board_pinout.h.
 */

#ifdef USE_LVGL_UI

#include "uart_link.h"

#include <Arduino.h>
#include <esp_log.h>
#include "board_pinout.h"

static const char* TAG = "UartLink";

// ---------------------------------------------------------------------------
// Static state
// ---------------------------------------------------------------------------

static HardwareSerial*     uart       = nullptr;
static proto_parser_t       parser;

static UartLink::GpsFixHandler    gpsFixHandler    = nullptr;
static UartLink::LoraRxHandler    loraRxHandler    = nullptr;
static UartLink::LoraTxAckHandler loraTxAckHandler = nullptr;
static UartLink::StatusHandler    statusHandler    = nullptr;
static UartLink::ErrorHandler     errorHandler     = nullptr;

// ---------------------------------------------------------------------------
// Dispatch — called by proto_parser_feed when a valid frame is received
// ---------------------------------------------------------------------------

static void dispatch(uint8_t type, const uint8_t* payload, uint16_t length) {
    (void)length;
    switch (type) {
    case MSG_GPS_FIX:
        if (gpsFixHandler && length >= sizeof(gps_fix_t))
            gpsFixHandler(reinterpret_cast<const gps_fix_t*>(payload));
        break;
    case MSG_LORA_RX: {
        // lora_rx_t has flexible array member data[]; validate pkt_len fits
        if (!loraRxHandler || length < 8) break;
        const lora_rx_t* rx = reinterpret_cast<const lora_rx_t*>(payload);
        if (length < 8u + rx->pkt_len) break;
        loraRxHandler(rx);
        break;
    }
    case MSG_LORA_TX_ACK:
        if (loraTxAckHandler && length >= sizeof(lora_tx_ack_t))
            loraTxAckHandler(reinterpret_cast<const lora_tx_ack_t*>(payload));
        break;
    case MSG_STATUS:
        if (statusHandler && length >= sizeof(status_t))
            statusHandler(reinterpret_cast<const status_t*>(payload));
        break;
    case MSG_ERROR:
        if (errorHandler && length >= sizeof(proto_error_t))
            errorHandler(reinterpret_cast<const proto_error_t*>(payload));
        break;
    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

namespace UartLink {

void init(int rxPin, int txPin, uint32_t baud) {
#if !defined(LORA_ON_C3)
    (void)rxPin; (void)txPin; (void)baud;
    return;
#else
    // Waveshare Grove connector uses pins 43/44. UART0 console also lives here;
    // Serial2.begin overrides the pin mapping so both can coexist.
    if (rxPin < 0) rxPin = 44;  // S3 RX ← C3 TX
    if (txPin < 0) txPin = 43;  // S3 TX → C3 RX

    uart = &Serial2;
    uart->begin(baud, SERIAL_8N1, rxPin, txPin);
    proto_parser_init(&parser);

    ESP_LOGI(TAG, "Serial2 init: baud=%u RX=%d TX=%d", baud, rxPin, txPin);
#endif
}

void loop() {
    if (!uart) return;
    while (uart->available()) {
        uint8_t b = uart->read();
        proto_parser_feed(&parser, b, dispatch);
    }
}

// --- S3 → C3 -------------------------------------------------------------

bool sendTxReq(const uint8_t* aprs_pkt, uint16_t len) {
    if (!uart || !aprs_pkt || len == 0) return false;
    // Payload = pkt_len(2B LE) + raw APRS frame
    uint8_t payload[PROTO_MAX_PAYLOAD];
    payload[0] = (uint8_t)(len);
    payload[1] = (uint8_t)(len >> 8);
    memcpy(payload + 2, aprs_pkt, len);

    uint8_t frame[PROTO_MAX_FRAME];
    size_t fLen = proto_build_frame(frame, MSG_LORA_TX_REQ, payload, 2 + len);
    size_t written = uart->write(frame, fLen);
    return written == fLen;
}

bool sendConfig(const config_t& cfg) {
    if (!uart) return false;
    uint8_t frame[PROTO_MAX_FRAME];
    size_t fLen = proto_build_frame(frame, MSG_CONFIG,
                                    reinterpret_cast<const uint8_t*>(&cfg),
                                    sizeof(config_t));
    size_t written = uart->write(frame, fLen);
    return written == fLen;
}

// --- callback setters ----------------------------------------------------

void setGpsFixHandler(GpsFixHandler h)       { gpsFixHandler    = h; }
void setLoraRxHandler(LoraRxHandler h)       { loraRxHandler    = h; }
void setLoraTxAckHandler(LoraTxAckHandler h)  { loraTxAckHandler = h; }
void setStatusHandler(StatusHandler h)        { statusHandler    = h; }
void setErrorHandler(ErrorHandler h)          { errorHandler     = h; }

// --- diagnostic ----------------------------------------------------------

uint16_t getCrcErrorCount() { return parser.crc_error_count; }
uint16_t getRxFrameCount()  { return parser.rx_frame_count; }

} // namespace UartLink

#endif // USE_LVGL_UI
