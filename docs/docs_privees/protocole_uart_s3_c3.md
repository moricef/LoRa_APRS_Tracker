# Protocole UART S3 ↔ C3

## Vue d'ensemble

Protocole binaire full-duplex entre ESP32-S3 (UI) et ESP32-C3 (Radio/GNSS).
UART 460800 baud, 8N1. Pas de handshake matériel (ni RTS/CTS, ni IRQ GPIO).

Le C3 envoie quand il veut (GPS fix, paquets LoRa reçus, ACK, heartbeat).
Le S3 envoie quand il veut (demande TX, changement de config).
L'ISR UART côté S3 absorbe les rafales via ring buffer 4096 octets.

## Format de trame

```
+------+------+--------+-----------------+----------+
| SYNC | TYPE | LENGTH |     PAYLOAD     |  CRC16   |
| 1B   | 1B   | 2B LE  |   0-512 octets  |  2B LE   |
+------+------+--------+-----------------+----------+
```

| Champ | Taille | Description |
|-------|--------|-------------|
| SYNC | 1 octet | `0xAA` — marqueur de début de trame |
| TYPE | 1 octet | Type de message (voir table ci-dessous) |
| LENGTH | 2 octets | Taille du payload en octets (little-endian), 0-512 |
| PAYLOAD | 0-512 octets | Données (structure dépend du TYPE) |
| CRC16 | 2 octets | CRC-16/CCITT sur TYPE + LENGTH + PAYLOAD (little-endian) |

**Taille max d'une trame : 1 + 1 + 2 + 512 + 2 = 518 octets.**

### Synchronisation

Le parser cherche `0xAA`, puis lit TYPE + LENGTH. Si le CRC ne matche pas, il rejette la trame et cherche le prochain `0xAA`. Pas de byte stuffing — le `0xAA` peut apparaître dans le payload, le parser s'appuie sur LENGTH + CRC pour valider.

### CRC-16/CCITT

Polynôme `0x1021`, valeur initiale `0xFFFF`. Calculé sur les octets TYPE + LENGTH (2B) + PAYLOAD.

```c
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}
```

## Types de messages

### C3 → S3

| Type | Code | Payload | Fréquence |
|------|------|---------|-----------|
| GPS_FIX | 0x01 | Position GPS filtrée | 1 Hz (configurable) |
| LORA_RX | 0x02 | Paquet APRS reçu | Événementiel |
| LORA_TX_ACK | 0x11 | Confirmation de TX | Réponse à TX_REQ |
| STATUS | 0x30 | État radio + GPS | 5s (heartbeat) |
| ERROR | 0x3F | Code erreur | Événementiel |

### S3 → C3

| Type | Code | Payload | Fréquence |
|------|------|---------|-----------|
| LORA_TX_REQ | 0x10 | Paquet APRS à transmettre | Événementiel |
| CONFIG | 0x20 | Paramètres radio/GPS/beacon | Événementiel |
| CONFIG_ACK | 0x21 | — | Réponse à CONFIG |

## Structures payload

Toutes les structures sont **packed** (pas de padding), **little-endian**.

### GPS_FIX (0x01) — 30 octets

```c
typedef struct __attribute__((packed)) {
    int32_t  lat;        // Latitude × 1e7 (ex: 438123456 = 43.8123456°)
    int32_t  lon;        // Longitude × 1e7
    int32_t  alt;        // Altitude en mm (ex: 1234567 = 1234.567 m)
    uint16_t speed;      // Vitesse en cm/s (ex: 1389 = 13.89 m/s = 50 km/h)
    uint16_t heading;    // Cap × 100 (ex: 18045 = 180.45°)
    uint8_t  sats;       // Nombre de satellites
    uint8_t  fix_type;   // 0=no fix, 2=2D, 3=3D
    uint16_t hdop;       // HDOP × 100 (ex: 120 = 1.20)
    uint32_t timestamp;  // UTC epoch (secondes depuis 1970-01-01)
    uint8_t  flags;      // Bit 0: fix valid, Bit 1: DGPS, Bit 2: speed valid
    uint8_t  reserved;   // Padding pour alignement pair
} gps_fix_t;             // 30 octets
```

### LORA_RX (0x02) — 8 octets header + payload variable

```c
typedef struct __attribute__((packed)) {
    int16_t  rssi;       // RSSI en dBm × 10 (ex: -1125 = -112.5 dBm)
    int8_t   snr;        // SNR en dB × 4 (ex: 28 = 7.0 dB)
    uint8_t  cr;         // Coding rate (5-8)
    uint16_t pkt_len;    // Longueur du paquet APRS brut
    uint16_t freq_err;   // Erreur de fréquence en Hz
    uint8_t  data[];     // Paquet APRS brut (pkt_len octets)
} lora_rx_t;             // 8 + pkt_len octets
```

### LORA_TX_REQ (0x10) — 2 octets header + payload variable

```c
typedef struct __attribute__((packed)) {
    uint16_t pkt_len;    // Longueur du paquet APRS à transmettre
    uint8_t  data[];     // Paquet APRS brut (pkt_len octets)
} lora_tx_req_t;         // 2 + pkt_len octets
```

### LORA_TX_ACK (0x11) — 4 octets

```c
typedef struct __attribute__((packed)) {
    uint8_t  status;     // 0=OK, 1=TX_TIMEOUT, 2=SPI_ERROR, 3=BUSY
    uint8_t  reserved;
    uint16_t irq_flags;  // Registre IRQ du SX1262 au moment du TX
} lora_tx_ack_t;         // 4 octets
```

### CONFIG (0x20) — 16 octets

```c
typedef struct __attribute__((packed)) {
    uint32_t freq;       // Fréquence en Hz (ex: 433775000)
    int8_t   tx_power;   // Puissance TX en dBm (-9 à +22)
    uint8_t  sf;         // Spreading Factor (7-12)
    uint8_t  bw;         // Bandwidth index: 0=7.8k, 6=62.5k, 7=125k, 8=250k, 9=500k
    uint8_t  cr;         // Coding Rate (5-8)
    uint16_t preamble;   // Longueur préambule (symboles)
    uint8_t  sb_active;  // SmartBeaconing actif (0/1)
    uint8_t  sb_slow_rate;  // SmartBeacon slow rate (secondes / 10)
    uint8_t  sb_fast_rate;  // SmartBeacon fast rate (secondes)
    uint8_t  sb_min_speed;  // Vitesse min km/h
    uint8_t  sb_max_speed;  // Vitesse max km/h
} config_t;              // 16 octets
```

### STATUS (0x30) — 12 octets

```c
typedef struct __attribute__((packed)) {
    uint8_t  gps_fix;    // 0=no fix, 2=2D, 3=3D
    uint8_t  gps_sats;   // Nombre de satellites
    uint8_t  lora_state; // 0=idle, 1=RX, 2=TX, 3=CAD, 4=sleep
    uint8_t  flags;      // Bit 0: RX pending in FIFO, Bit 1: TX in progress
    uint16_t rx_count;   // Compteur paquets reçus depuis boot
    uint16_t tx_count;   // Compteur paquets émis depuis boot
    uint16_t error_count;// Compteur erreurs depuis boot
    uint16_t uptime;     // Uptime en minutes (max 45 jours)
} status_t;              // 12 octets
```

### ERROR (0x3F) — 4 octets

```c
typedef struct __attribute__((packed)) {
    uint16_t code;       // Code erreur (voir table)
    uint16_t detail;     // Détail (ex: registre SX1262, errno)
} error_t;               // 4 octets
```

Codes d'erreur :

| Code | Description |
|------|-------------|
| 0x0001 | LORA_INIT_FAIL — radio.begin() échoué |
| 0x0002 | LORA_SPI_ERROR — SPI timeout ou CHIP_NOT_FOUND |
| 0x0003 | LORA_XOSC_ERROR — XOSC_START_ERR (bit 5 device errors) |
| 0x0010 | GPS_NO_FIX — pas de fix depuis > 60s |
| 0x0011 | GPS_UART_ERROR — pas de données GNSS depuis > 10s |
| 0x0020 | PROTO_CRC_ERROR — CRC invalide sur trame reçue du S3 |
| 0x0021 | PROTO_OVERFLOW — FIFO C3 pleine, trame droppée |

## Machine d'état du parser

Le parser est identique côté S3 et C3 (même code, deux instances).

```
                  ┌──────────┐
         octet    │          │  octet != 0xAA
     ──────────►  │  IDLE    │ ◄────────────────┐
                  │          │                   │
                  └────┬─────┘                   │
                       │ 0xAA                    │
                       ▼                         │
                  ┌──────────┐                   │
                  │  TYPE    │                   │
                  └────┬─────┘                   │
                       │ 1 octet                 │
                       ▼                         │
                  ┌──────────┐                   │
                  │  LEN_LO  │                   │
                  └────┬─────┘                   │
                       │ 1 octet                 │
                       ▼                         │
                  ┌──────────┐                   │
                  │  LEN_HI  │                   │
                  └────┬─────┘                   │
                       │ length = LO | (HI<<8)   │
                       │ if length > 512 → IDLE ─┘
                       ▼                         │
                  ┌──────────┐                   │
                  │  PAYLOAD │ (length octets)   │
                  └────┬─────┘                   │
                       │ tous reçus              │
                       ▼                         │
                  ┌──────────┐                   │
                  │  CRC_LO  │                   │
                  └────┬─────┘                   │
                       │ 1 octet                 │
                       ▼                         │
                  ┌──────────┐                   │
                  │  CRC_HI  │                   │
                  └────┬─────┘                   │
                       │ CRC OK → dispatch()     │
                       │ CRC KO → IDLE ──────────┘
                       ▼
                  ┌──────────┐
                  │ DISPATCH │ → callback(type, payload, length)
                  └──────────┘
```

## Séquences typiques

### Réception LoRa

```
C3                                    S3
 │  ← SX1262 IRQ (DIO1) ──           │
 │  Lecture paquet SPI                │
 │                                    │
 │  ── LORA_RX (raw + RSSI) ──────►  │
 │                                    │  Parse + affiche popup RX
 │                                    │  Stocke dans msg_utils
```

### Émission LoRa (demandée par l'UI)

```
C3                                    S3
 │                                    │  User compose un message
 │  ◄────── LORA_TX_REQ (raw) ─────  │
 │                                    │
 │  radio.transmit()                  │
 │                                    │
 │  ── LORA_TX_ACK (status) ───────► │
 │                                    │  Affiche popup TX OK/FAIL
```

### SmartBeaconing (autonome C3)

```
C3                                    S3
 │  GPS fix → filtre → SmartBeacon    │
 │  Décision: émettre beacon          │
 │                                    │
 │  radio.transmit(beacon)            │
 │                                    │
 │  ── LORA_TX_ACK (status) ───────► │  (optionnel, pour le compteur TX)
 │  ── GPS_FIX ────────────────────► │  (position mise à jour sur la carte)
```

### Changement de configuration

```
C3                                    S3
 │                                    │  User change fréquence dans Settings
 │  ◄────── CONFIG (freq, power) ──  │
 │                                    │
 │  radio.setFrequency()              │
 │  radio.setOutputPower()            │
 │                                    │
 │  ── CONFIG_ACK ─────────────────► │  (confirmation)
```

### Heartbeat

```
C3                                    S3
 │  (toutes les 5s)                   │
 │  ── STATUS ─────────────────────► │
 │                                    │  Met à jour icônes GPS/LoRa
 │                                    │  Détecte perte C3 si > 15s sans STATUS
```

## Gestion des erreurs

### Trame corrompue (CRC KO)

Le récepteur incrémente un compteur `crc_error_count` et cherche le prochain `0xAA`. Pas de retransmission — APRS est fire-and-forget.

### Timeout réponse

- TX_REQ sans TX_ACK dans les 10 secondes → S3 affiche "TX TIMEOUT" à l'utilisateur.
- CONFIG sans CONFIG_ACK dans les 3 secondes → S3 retente une fois, puis affiche erreur.

### Perte de communication

Si le S3 ne reçoit aucun STATUS pendant 15 secondes → alerte "C3 OFFLINE" sur le dashboard. Le S3 continue de fonctionner en mode dégradé (carte + messages stockés, pas de radio/GPS).

## Implémentation — header commun

```c
// uart_proto.h — partagé entre S3 et C3

#ifndef UART_PROTO_H
#define UART_PROTO_H

#include <stdint.h>
#include <stddef.h>

#define PROTO_SYNC          0xAA
#define PROTO_MAX_PAYLOAD   512
#define PROTO_HEADER_SIZE   4    // SYNC + TYPE + LEN(2)
#define PROTO_CRC_SIZE      2
#define PROTO_MAX_FRAME     (PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD + PROTO_CRC_SIZE)

// Message types C3 → S3
#define MSG_GPS_FIX         0x01
#define MSG_LORA_RX         0x02
#define MSG_LORA_TX_ACK     0x11
#define MSG_STATUS          0x30
#define MSG_ERROR           0x3F

// Message types S3 → C3
#define MSG_LORA_TX_REQ     0x10
#define MSG_CONFIG          0x20
#define MSG_CONFIG_ACK      0x21

// TX ACK status codes
#define TX_OK               0x00
#define TX_TIMEOUT          0x01
#define TX_SPI_ERROR        0x02
#define TX_BUSY             0x03

// Error codes
#define ERR_LORA_INIT       0x0001
#define ERR_LORA_SPI        0x0002
#define ERR_LORA_XOSC       0x0003
#define ERR_GPS_NO_FIX      0x0010
#define ERR_GPS_UART        0x0011
#define ERR_PROTO_CRC       0x0020
#define ERR_PROTO_OVERFLOW  0x0021

// Parser states
typedef enum {
    PARSE_IDLE,
    PARSE_TYPE,
    PARSE_LEN_LO,
    PARSE_LEN_HI,
    PARSE_PAYLOAD,
    PARSE_CRC_LO,
    PARSE_CRC_HI
} parse_state_t;

// Parser context
typedef struct {
    parse_state_t state;
    uint8_t  type;
    uint16_t length;
    uint16_t payload_idx;
    uint8_t  payload[PROTO_MAX_PAYLOAD];
    uint8_t  crc_lo;
    uint16_t crc_error_count;
    uint16_t rx_frame_count;
} proto_parser_t;

// Callback signature
typedef void (*proto_callback_t)(uint8_t type, const uint8_t *payload, uint16_t length);

// --- API ---

// CRC-16/CCITT
uint16_t proto_crc16(const uint8_t *data, size_t len);

// Init parser
void proto_parser_init(proto_parser_t *p);

// Feed bytes one at a time (call from UART ISR or task)
// Calls callback when a complete valid frame is received
void proto_parser_feed(proto_parser_t *p, uint8_t byte, proto_callback_t cb);

// Build a frame into buf (must be >= PROTO_MAX_FRAME)
// Returns total frame size
size_t proto_build_frame(uint8_t *buf, uint8_t type, const uint8_t *payload, uint16_t length);

#endif // UART_PROTO_H
```

## Implémentation — parser + builder

```c
// uart_proto.c — partagé entre S3 et C3

#include "uart_proto.h"

uint16_t proto_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

void proto_parser_init(proto_parser_t *p) {
    p->state = PARSE_IDLE;
    p->crc_error_count = 0;
    p->rx_frame_count = 0;
}

void proto_parser_feed(proto_parser_t *p, uint8_t byte, proto_callback_t cb) {
    switch (p->state) {
    case PARSE_IDLE:
        if (byte == PROTO_SYNC) p->state = PARSE_TYPE;
        break;

    case PARSE_TYPE:
        p->type = byte;
        p->state = PARSE_LEN_LO;
        break;

    case PARSE_LEN_LO:
        p->length = byte;
        p->state = PARSE_LEN_HI;
        break;

    case PARSE_LEN_HI:
        p->length |= (uint16_t)byte << 8;
        if (p->length > PROTO_MAX_PAYLOAD) {
            p->state = PARSE_IDLE; // Frame too large, resync
        } else if (p->length == 0) {
            p->payload_idx = 0;
            p->state = PARSE_CRC_LO; // No payload, go to CRC
        } else {
            p->payload_idx = 0;
            p->state = PARSE_PAYLOAD;
        }
        break;

    case PARSE_PAYLOAD:
        p->payload[p->payload_idx++] = byte;
        if (p->payload_idx >= p->length) {
            p->state = PARSE_CRC_LO;
        }
        break;

    case PARSE_CRC_LO:
        p->crc_lo = byte;
        p->state = PARSE_CRC_HI;
        break;

    case PARSE_CRC_HI: {
        uint16_t rx_crc = p->crc_lo | ((uint16_t)byte << 8);

        // CRC is computed over TYPE + LENGTH(2B) + PAYLOAD
        uint8_t hdr[3] = { p->type, (uint8_t)(p->length), (uint8_t)(p->length >> 8) };
        uint16_t calc_crc = proto_crc16(hdr, 3);
        if (p->length > 0) {
            // Continue CRC over payload
            for (uint16_t i = 0; i < p->length; i++) {
                calc_crc ^= (uint16_t)p->payload[i] << 8;
                for (int j = 0; j < 8; j++) {
                    calc_crc = (calc_crc & 0x8000)
                        ? (calc_crc << 1) ^ 0x1021 : calc_crc << 1;
                }
            }
        }

        if (rx_crc == calc_crc) {
            p->rx_frame_count++;
            if (cb) cb(p->type, p->payload, p->length);
        } else {
            p->crc_error_count++;
        }
        p->state = PARSE_IDLE;
        break;
    }
    }
}

size_t proto_build_frame(uint8_t *buf, uint8_t type,
                         const uint8_t *payload, uint16_t length)
{
    buf[0] = PROTO_SYNC;
    buf[1] = type;
    buf[2] = (uint8_t)(length);
    buf[3] = (uint8_t)(length >> 8);

    if (length > 0 && payload) {
        for (uint16_t i = 0; i < length; i++) {
            buf[4 + i] = payload[i];
        }
    }

    // CRC over TYPE + LENGTH + PAYLOAD
    uint16_t crc = proto_crc16(&buf[1], 3 + length);
    buf[4 + length]     = (uint8_t)(crc);
    buf[4 + length + 1] = (uint8_t)(crc >> 8);

    return 4 + length + 2;
}
```

## Notes d'intégration

### Côté S3 (Arduino)

```cpp
#include "uart_proto.h"

static proto_parser_t parser;

void setup() {
    Serial1.setRxBufferSize(4096);
    Serial1.begin(460800, SERIAL_8N1, RX_PIN, TX_PIN);
    proto_parser_init(&parser);
}

void loop() {
    while (Serial1.available()) {
        proto_parser_feed(&parser, Serial1.read(), on_c3_message);
    }
}

void on_c3_message(uint8_t type, const uint8_t *payload, uint16_t length) {
    switch (type) {
    case MSG_GPS_FIX: {
        const gps_fix_t *fix = (const gps_fix_t *)payload;
        // Mettre à jour gpsFix global
        break;
    }
    case MSG_LORA_RX: {
        const lora_rx_t *rx = (const lora_rx_t *)payload;
        // Passer à msg_utils pour traitement APRS
        break;
    }
    // ...
    }
}
```

### Côté C3 (ESP-IDF)

```c
#include "uart_proto.h"

static proto_parser_t parser;
static uint8_t tx_buf[PROTO_MAX_FRAME];

void uart_link_init(void) {
    // uart_config_t + uart_driver_install(UART_NUM_0, 4096, ...)
    proto_parser_init(&parser);
}

void uart_link_send_gps(const gps_fix_t *fix) {
    size_t len = proto_build_frame(tx_buf, MSG_GPS_FIX,
                                   (const uint8_t *)fix, sizeof(gps_fix_t));
    uart_write_bytes(UART_NUM_0, tx_buf, len);
}

void uart_link_task(void *pv) {
    uint8_t buf[128];
    while (1) {
        int rx = uart_read_bytes(UART_NUM_0, buf, sizeof(buf), pdMS_TO_TICKS(10));
        for (int i = 0; i < rx; i++) {
            proto_parser_feed(&parser, buf[i], on_s3_message);
        }
    }
}
```
