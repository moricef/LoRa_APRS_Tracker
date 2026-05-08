# Architecture Dual MCU : ESP32-S3 (UI) + ESP32-C3 (Radio/GNSS) — PCB maison

## Objectif

Se détacher des cartes commerciales préfabriquées (CrowPanel, Waveshare) et leurs compromis (pas de monitoring batterie, GPIO partagés, SPI SD lent, pas de TXEN/RXEN). PCB custom avec :

- **Écran 7" RGB parallèle** (800x480 ou 1024x600)
- **SDMMC** pour les tuiles NAV (débit x5-x10 vs SPI SD)
- **Dual MCU** : S3 dédié UI + C3 dédié radio/GNSS
- **Power management intégré** : charge Li-ion + monitoring I2C

## Pourquoi dual MCU

Un écran RGB 16-bit consomme **20 GPIO** sur le S3 (16 data + HSYNC + VSYNC + DE + PCLK). Il ne reste pas assez de GPIO pour caser LoRa SPI + GNSS UART + SD sur le même chip.

La séparation S3/C3 élimine ce problème : le S3 gère l'affichage + SD + UI, le C3 gère la radio + GPS de façon autonome. Les deux communiquent par UART.

## Architecture

```
+----------------------------+        UART         +-------------------------+
|   ESP32-S3-WROOM-1-N16R8   | <-----------------> |   ESP32-C3-MINI-1       |
|                             |  TX/RX + IRQ pin    |                         |
|   - Ecran 7" RGB 16-bit    |                      |   - GNSS UART + PPS     |
|   - LVGL UI                 |  C3 TX -> S3 RX     |   - LoRa SPI (SX1262)   |
|   - SDMMC (tuiles NAV)     |  C3 RX <- S3 TX     |   - SmartBeaconing      |
|   - I2C (touch + fuel gauge)|  C3 IRQ -> S3 GPIO  |   - Filtrage GPS        |
|   - WiFi / BLE              |  GND commun          |   - Protocole binaire   |
|   - USB-CDC (debug serial)  |                      |   - Debug via JTAG USB  |
|   - I2S audio (MAX98357A)  |                      |                         |
+----------------------------+                      +-------------------------+
         |                                                     |
    [Li-ion 1S]                                           [Antenne LoRa]
    [TP4056 charge]                                       [Antenne GNSS]
    [MAX17048 I2C]
```

## Budget GPIO ESP32-S3

### Contrainte du module WROOM-1-N16R8

GPIO 26-37 sont utilisées en interne par le module (flash + PSRAM OPI).
GPIO disponibles : **0-21, 38-48** = 33 pins.

GPIO 19/20 = USB (fixe). Avec USB-CDC pour le serial debug, GPIO 43/44 sont libérées.

### Allocation détaillée

| Fonction | GPIO | Nb | Notes |
|----------|------|----|-------|
| **LCD RGB R0-R4** | 14, 21, 47, 48, 45 | 5 | |
| **LCD RGB G0-G5** | 9, 46, 3, 8, 16, 1 | 6 | |
| **LCD RGB B0-B4** | 15, 7, 6, 5, 4 | 5 | |
| **LCD PCLK** | 42 | 1 | |
| **LCD DE** | 40 | 1 | |
| **LCD VSYNC** | 41 | 1 | |
| **LCD HSYNC** | 39 | 1 | |
| **LCD Backlight** | 2 | 1 | PWM |
| **USB D-/D+** | 19, 20 | 2 | USB-CDC (serial debug + flash) |
| **SDMMC CLK** | 10 | 1 | |
| **SDMMC CMD** | 11 | 1 | |
| **SDMMC D0** | 12 | 1 | SDMMC 1-bit minimum |
| **I2C SDA** | 38 | 1 | Touch GT911 + MAX17048 (bus partagé) |
| **I2C SCL** | 43 | 1 | Libéré car USB-CDC remplace UART0 |
| **UART S3←C3 RX** | 44 | 1 | Libéré car USB-CDC remplace UART0 |
| **UART S3→C3 TX** | 0 | 1 | Strapping pin — pull-up externe requis |
| **IRQ C3→S3** | 13 | 1 | Trame APRS en attente |
| **I2S BCLK** | 17 | 1 | Haut-parleur via DAC I2S (ex: MAX98357A) |
| **I2S LRCLK** | 18 | 1 | |
| **I2S DOUT** | 13 | 1 | |
| **Total** | | **34** | Sur 33 dispo — voir arbitrage ci-dessous |

### Arbitrage : 34 pins nécessaires, 33 disponibles

I2S prend 3 GPIO (BCLK, LRCLK, DOUT) au lieu de 1 pour un buzzer PWM. On dépasse de 1 pin.

**Options :**
1. **Sacrifier l'IRQ hardware C3→S3** (GPIO 13 → I2S DOUT) : la signalisation passe par le protocole UART (flag dans le heartbeat STATUS). Ajoute ~1-5 ms de latence, négligeable pour APRS.
2. **SDMMC 1-bit sans spare** : les 3 GPIO I2S prennent 13, 17, 18. Zéro spare. SDMMC 4-bit impossible sans sacrifier l'I2S.
3. **Passer au chip nu ESP32-S3FN16R8** au lieu du module WROOM : GPIO 33-37 récupérables (~5 pins), mais design RF antenne WiFi à gérer sur le PCB.

**Recommandation : option 1.** L'IRQ hardware est un confort, pas une nécessité. Le protocole UART gère déjà la signalisation. On garde I2S + SDMMC 1-bit + 1 spare (GPIO 18 libéré si I2S n'utilise que BCLK=17, LRCLK=13, DOUT=18 — dans ce cas pas de spare non plus).

Allocation finale sans IRQ hardware :

| Fonction | GPIO |
|----------|------|
| I2S BCLK | 17 |
| I2S LRCLK | 18 |
| I2S DOUT | 13 |

Total : **33/33 GPIO utilisées.** Zéro spare sur le S3.

## Budget GPIO ESP32-C3

Le C3 a ~22 GPIO. Allocation simple, marge large.

| Fonction | GPIO | Nb | Notes |
|----------|------|----|-------|
| **LoRa SCK** | 4 | 1 | SPI2 (hors SPI0/1 flash) |
| **LoRa MISO** | 5 | 1 | |
| **LoRa MOSI** | 6 | 1 | |
| **LoRa CS** | 7 | 1 | |
| **LoRa RST** | 3 | 1 | Dédié — plus de partage avec TFT |
| **LoRa BUSY** | 8 | 1 | |
| **LoRa DIO1** | 10 | 1 | IRQ (TX_DONE, RX_DONE) |
| **GNSS TX** | 0 | 1 | UART1 |
| **GNSS RX** | 1 | 1 | UART1 |
| **GNSS PPS** | 2 | 1 | Interrupt |
| **UART C3→S3 TX** | 21 | 1 | UART0 réaffecté (debug via USB JTAG) |
| **UART C3→S3 RX** | 20 | 1 | UART0 réaffecté |
| **Total** | | **12** | |
| **Libre** | 9, 11, 18, 19 | **4** | TXEN, RXEN si module le requiert, LED status, spare |

### Avantage clé : TXEN/RXEN disponibles

Si le module LoRa nécessite TXEN/RXEN (ex: HT-RA62 dans certaines configurations), GPIO 18 et 19 sont disponibles. Plus de compromis "pas connecté sur le PCB".

## Module LoRa recommandé

Pour le PCB maison, choisir un module compatible sans câblage TXEN/RXEN :

| Module | Switch RF | GPIO extras | Recommandé |
|--------|-----------|-------------|------------|
| Module original Elecrow (SX1262 868MHz) | DIO2 interne | Aucun | Oui |
| Seeed Wio-SX1262-LF (433MHz) | DIO2 + RF_SW (1 GPIO) | 1 | Oui |
| Heltec HT-RA62 (433MHz) | DIO2 + TXEN/RXEN | 2 (ou 0 selon schéma) | Possible, GPIO dispo sur C3 |

## Power Management (intégré au PCB)

### Charge batterie

- **TP4056** (ou MCP73831) : charge linéaire 1S Li-ion depuis USB 5V
- Courant de charge : 500mA-1A (résistance PROG)
- LED CHRG + LED STDBY
- Protection : sous-tension (DW01A + FS8205) ou module TP4056 intégré

### Monitoring batterie

- **MAX17048** sur le bus I2C partagé (touch + fuel gauge)
- Adresse 0x36 (pas de conflit avec GT911 0x5D)
- VIN connecté directement au + batterie (pas au 3.3V régulé)
- Lecture SOC + tension depuis le S3

### Régulation

```
USB 5V ──┬── TP4056 ── Batterie 1S (3.0-4.2V)
          │
Batterie ─┴── LDO 3.3V (AMS1117-3.3 ou ME6211) ── S3 + C3 + Périphériques
                                                      │
                                              MAX17048 VIN (mesure cellule)
```

### Autonomie estimée

| Composant                       | Courant typ.     | Notes                   |
|---------------------------------|------------------|-------------------------|
| ESP32-S3 (LVGL actif, WiFi off) | ~80-120 mA       |                         |
| ESP32-C3 (LoRa RX)              | ~25-40 mA        |                         |
| Écran 7" RGB + backlight        | ~100-200 mA      | Dépend de la luminosité |
| GNSS                            | ~25-40 mA        |                         |
| **Total**                       | **~230-400 mA**  |                         |

Avec 2x 18650 1S2P (4400 mAh) : **~11-19h** d'autonomie.

## Protocole série C3 ↔ S3

### Liaison physique

- UART 460800 baud (débit suffisant pour APRS, pas de bottleneck)
- GND commun (même PCB → pas de problème)
- Pas d'IRQ hardware (GPIO S3 saturées par I2S) — signalisation via protocole UART

### Format de trame

```
[SYNC 0xAA] [TYPE 1B] [LEN 2B LE] [PAYLOAD 0-512B] [CRC16]
```

### Types de messages

| Type                 | Direction | Contenu                                              |
|----------------------|-----------|------------------------------------------------------|
| `GPS_FIX` (0x01)     | C3→S3     | lat, lon, alt, speed, heading, sats, hdop, timestamp |
| `LORA_RX` (0x02)     | C3→S3     | Paquet APRS brut + RSSI + SNR                        |
| `LORA_TX_REQ` (0x10) | S3→C3     | Paquet APRS à transmettre                            |
| `LORA_TX_ACK` (0x11) | C3→S3     | Confirmation TX (success/fail + IRQ flags)           |
| `CONFIG` (0x20)      | S3→C3     | Fréquence, puissance, params SmartBeacon             |
| `STATUS` (0x30)      | C3→S3     | Heartbeat : état radio, GPS lock, erreurs            |

### Signalisation sans IRQ hardware

Sans broche IRQ dédiée (GPIO S3 toutes utilisées), la signalisation passe par le protocole UART :

1. C3 reçoit une trame APRS → stocke dans FIFO → envoie directement sur UART
2. S3 reçoit via interrupt UART (ring buffer hardware) → traite quand disponible
3. CRC16 vérifié côté S3
4. Le heartbeat STATUS (périodique) inclut un flag "trames en attente" en cas de non-ACK

Pas de perte si le S3 est occupé : le ring buffer UART hardware (256-512 octets) + FIFO C3 absorbent le délai. Latence ajoutée : ~1-5 ms, négligeable pour APRS.

## Firmware

### C3 — ESP-IDF natif (~1500 lignes)

- Lecture GNSS via UART1 + filtrage anti-gigue (Doppler, sauts)
- SmartBeaconing autonome (cap, vitesse, distance)
- LoRa via SPI2 (RadioLib + EspHal)
- Protocole série binaire vers S3 (FIFO + CRC + IRQ)
- Pas d'UI, pas de LVGL, pas de SD

### S3 — Arduino + LVGL (code existant ~80% réutilisé)

**Fichiers inchangés :**
- `lvgl_ui.cpp` — init LVGL + navigation
- `ui_dashboard.cpp` — écran principal
- `ui_messaging.cpp` — messages, conversations, contacts
- `ui_settings.cpp` — écrans de configuration
- `ui_popups.cpp` — popups TX/RX/notifications
- `ui_map_manager.cpp` + `src/map/` — carte vectorielle
- `msg_utils.cpp` — gestion messages

**Fichiers à adapter (interface radio/GPS) :**
- `gps_utils.cpp` → réception GPS_FIX depuis UART C3 (plus de parsing NMEA)
- `lora_utils.cpp` → envoi/réception UART vers C3 (plus de SPI direct)
- `LoRa_APRS_Tracker.cpp` → simplification main loop
- Nouveau `uart_link.cpp` — protocole série C3↔S3

L'UI ne fait aucun appel SPI/UART hardware direct — elle consomme des structures (`gpsFix`, `lastReceivedPacket`, `Config`). La transition est transparente.

## Avantages vs cartes commerciales

| Point                 | CrowPanel 3.5"            | Waveshare 4.3" | PCB maison 7"            |
|-----------------------|---------------------------|----------------|--------------------------|
| Écran                 | 480x320 SPI               | 800x480 RGB    | 800x480+ RGB             |
| SD                    | SPI (2 MB/s)              | SPI            | SDMMC (12-20 MB/s)       |
| LoRa                  | SPI partagé SD            | Pas de LoRa    | C3 dédié, SPI propre     |
| GNSS                  | UART sur S3               | Pas de GNSS    | C3 dédié, UART propre    |
| Batterie monitoring   | Aucun (BATTERY_PIN=-1)    | ADC si dispo   | MAX17048 I2C intégré     |
| Charge Li-ion         | IC inconnu, non documenté | Variable       | TP4056/MCP73831 maîtrisé |
| TXEN/RXEN             | Non connectés             | N/A            | GPIO C3 disponibles      |
| GPIO libres           | 0                         | Variable       | 3-4 spare S3, 3 spare C3 |

## Limites et risques

1. **Budget GPIO S3 très serré** : 32/33 pins utilisées avec SDMMC 1-bit. Pas de marge pour ajouter des périphériques I/O au S3 sans sacrifier une fonction.
2. **Design PCB RF** : si utilisation du module WROOM (recommandé), l'antenne WiFi/BLE est intégrée. L'antenne LoRa et GNSS sont côté C3 — routage à soigner.
3. **Écran 7" RGB** : consommation backlight élevée (~100-200 mA), impact autonomie.
4. **Firmware C3** : nouveau code ESP-IDF à écrire (~1500 lignes). Pas de réutilisation directe du code Arduino existant pour la partie radio/GPS.
5. **Latence UART** : la communication S3↔C3 ajoute ~1-5 ms de latence. Négligeable pour APRS (beacon toutes les N secondes) mais à mesurer.

## Étapes de développement

1. **Schématique** : KiCad, S3 + C3 + TP4056 + MAX17048 + connecteurs écran/antennes
2. **Routage PCB** : 4 couches recommandé (plan GND, plan 3.3V, 2 couches signal)
3. **Prototype C3 seul** : firmware ESP-IDF sur DevKitC-02, valider LoRa TX/RX + GPS
4. **Prototype S3 seul** : écran 7" sur DevKit, valider RGB + SDMMC + LVGL
5. **Intégration** : liaison UART, protocole série, tests bout en bout
6. **PCB v1** : fabrication + assemblage, tests terrain
