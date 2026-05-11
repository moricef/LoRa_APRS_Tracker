/* UART Link — S3 side of C3↔S3 binary protocol
 *
 * Wraps uart_proto.h parser/builder (C, shared with C3 firmware).
 * UART2, 460800 baud, pins from board_pinout.h (GPS_RX/GPS_TX).
 */

#ifndef UART_LINK_H
#define UART_LINK_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "uart_proto.h"
#ifdef __cplusplus
}
#endif

namespace UartLink {

// --- init / loop ---------------------------------------------------------

void init(int rxPin = -1, int txPin = -1, uint32_t baud = 460800);
void loop();

// --- S3 → C3 -------------------------------------------------------------

bool sendTxReq(const uint8_t* aprs_pkt, uint16_t len);
bool sendConfig(const config_t& cfg);

// --- callbacks (C3 → S3) -------------------------------------------------

typedef void (*GpsFixHandler)(const gps_fix_t* fix);
typedef void (*LoraRxHandler)(const lora_rx_t* rx);
typedef void (*LoraTxAckHandler)(const lora_tx_ack_t* ack);
typedef void (*StatusHandler)(const status_t* st);
typedef void (*ErrorHandler)(const proto_error_t* err);

void setGpsFixHandler(GpsFixHandler);
void setLoraRxHandler(LoraRxHandler);
void setLoraTxAckHandler(LoraTxAckHandler);
void setStatusHandler(StatusHandler);
void setErrorHandler(ErrorHandler);

// --- diagnostic ----------------------------------------------------------

uint16_t getCrcErrorCount();
uint16_t getRxFrameCount();

} // namespace UartLink

#endif
