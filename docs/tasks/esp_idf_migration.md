# Plan de migration Arduino → ESP-IDF natif

## État actuel

- Platform : pioarduino (ESP-IDF 5.1 / Arduino Core 3.3.7) ✅ commit `826814d1`
- Framework : `arduino` (encore actif dans platformio.ini)
- Les fonctions de base Arduino (millis, delay, GPIO) appellent déjà ESP-IDF en interne — les wrapper est inutile

## Stratégie révisée

PlatformIO + `framework = espidf` avec Arduino comme composant ESP-IDF (`arduino-esp32`
dans `components/`). Permet de migrer progressivement sans tout casser.

**Ne PAS wrapper les fonctions de base** (millis, delay, pinMode, etc.) — elles appellent
déjà ESP-IDF en interne. Se concentrer sur les vraies dépendances Arduino.

Chaque étape est un commit testable sur device.

## Phase 1 — SD.h → VFS natif (priorité haute)

Le gain de performance principal pour la carte.
- [ ] Remplacer `SD.h` par `esp_vfs_fat` + `sdmmc_host`
- [ ] Fichiers : `storage_utils.cpp`, `map_tiles.cpp`, `map_engine.cpp`, `gpx_writer.cpp`, `sd_logger.cpp`
- [ ] Bus SPI dédié possible si le hardware le permet (T-Deck partage display + SD)
- [ ] Référence : IceNav-v3 de Jordi (VFS natif, perf bien supérieure)

## Phase 2 — UART GPS (NeoGPS)

- [ ] Remplacer `HardwareSerial gpsSerial` par `driver/uart.h` natif
- [ ] Adapter NeoGPS pour lire depuis un ring buffer UART ESP-IDF
- [ ] 2 fichiers : `gps_utils.cpp`, `LoRa_APRS_Tracker.cpp`

## Phase 3 — I2C (clavier, display)

- [ ] Remplacer `Wire.begin/requestFrom/read` par `driver/i2c.h`
- [ ] 2 fichiers : `keyboard_utils.cpp`, `display.cpp`
- [ ] Note : le clavier I2C n'existera pas sur CrowPanel (clavier virtuel LVGL)

## Phase 4 — SPI LoRa (RadioLib)

Le plus gros morceau. RadioLib dépend de `SPI.h` Arduino.
- [ ] Option A : utiliser le HAL ESP-IDF de RadioLib (existe depuis v6)
- [ ] Option B : écrire un wrapper SPI ESP-IDF → interface RadioLib
- [ ] Migrer `lora_utils.cpp`

## Phase 5 — WiFi

- [ ] Remplacer Arduino WiFi par `esp_wifi` natif
- [ ] Adapter `wifi_utils.cpp`, `aprs_is_utils.cpp`, `web_utils.cpp`
- [ ] Plus complexe : callbacks, scan, AP mode, TCP sockets

## Phase 6 — BLE

- [ ] Remplacer `BLEDevice` Arduino par `esp_ble` natif
- [ ] Adapter `ble_utils.cpp`

## Phase 7 — Switch framework PlatformIO + retrait Arduino

- [ ] `platformio.ini` : `framework = espidf`
- [ ] Supprimer le composant Arduino
- [ ] Build pur ESP-IDF

## Inventaire actuel

| API Arduino | Occurrences | Fichiers impactés |
|---|---|---|
| SD.h (fichiers) | ~30 | 5 |
| millis() | 140 | 24 (pas besoin de migrer) |
| delay() | 46 | ~15 (pas besoin de migrer) |
| Wire. (I2C) | 4 | 2 |
| SPI. | 3 | 1 |
| HardwareSerial | 2 | 2 |
| pinMode/digitalWrite | ~20 | ~5 (pas besoin de migrer) |

## Libs tierces

| Lib | Dépendance Arduino | ESP-IDF natif |
|---|---|---|
| LovyanGFX | non (détection auto) | oui |
| LVGL | non | oui |
| RadioLib | oui (SPI, GPIO) | HAL ESP-IDF dispo v6+ |
| NeoGPS | oui (HardwareSerial) | non — wrapper à écrire |
| APRSPacketLib | non | oui |

## Ordre de priorité

Phase 1 (SD VFS) apporte le gain de perf concret pour la carte.
Phases 2-3 sont simples (UART, I2C = drivers ESP-IDF bien documentés).
Phase 4 (RadioLib SPI) est le point critique — vérifier le HAL ESP-IDF de RadioLib v6.
Phases 5-6 (WiFi, BLE) sont les plus complexes et les dernières.

## Leçons apprises

- **Ne pas wrapper les fonctions de base Arduino** : millis(), delay(), pinMode(), etc.
  appellent déjà ESP-IDF en interne. Wrapper = indirection inutile. Phase 0 initiale
  avec compat_millis() a été revertée.
- Se concentrer sur les vraies abstractions Arduino à remplacer : SD, WiFi, SPI, I2C, UART, BLE.

## Référence

- RadioLib ESP-IDF HAL : https://github.com/jgromes/RadioLib/tree/master/examples/NonArduino/ESP-IDF
- LovyanGFX ESP-IDF : détection automatique, pas de modification nécessaire
- PlatformIO ESP-IDF + Arduino component : https://docs.platformio.org/en/latest/frameworks/espidf.html
- IceNav-v3 (Jordi) : référence pour SD VFS natif avec perf optimale
