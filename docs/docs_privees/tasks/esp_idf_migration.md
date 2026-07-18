# Plan de migration Arduino → ESP-IDF natif

## Stratégie

PlatformIO + `framework = espidf` avec Arduino comme composant ESP-IDF (`arduino-esp32`
dans `components/`). Permet de migrer progressivement sans tout casser.

Chaque étape est un commit testable sur device.

## Phase 0 — Préparation (avant de toucher au framework)

- [ ] Créer un wrapper `compat/arduino_compat.h` avec :
  - `millis()` → `esp_timer_get_time() / 1000`
  - `delay(ms)` → `vTaskDelay(pdMS_TO_TICKS(ms))`
  - `pinMode/digitalWrite/digitalRead` → `gpio_set_direction/gpio_set_level/gpio_get_level`
- [ ] Remplacer les 140 `millis()` et 46 `delay()` par les wrappers
- [ ] Remplacer les ~20 `pinMode/digitalWrite/digitalRead` par les wrappers
- [ ] Compiler + tester : tout doit fonctionner identiquement sous Arduino

## Phase 1 — Switch framework PlatformIO

- [ ] `platformio.ini` : `framework = espidf`
- [ ] Ajouter `arduino-esp32` comme composant dans `components/`
- [ ] Résoudre les erreurs de compilation
- [ ] Compiler + tester

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

## Phase 7 — Retrait composant Arduino

- [ ] Supprimer `arduino-esp32` des composants
- [ ] Supprimer `compat/arduino_compat.h`
- [ ] Build pur ESP-IDF

## Inventaire actuel

| API Arduino | Occurrences | Fichiers impactés |
|---|---|---|
| millis() | 140 | 24 |
| delay() | 46 | ~15 |
| Wire. (I2C) | 4 | 2 |
| SPI. | 3 | 1 |
| HardwareSerial | 2 | 2 |
| pinMode/digitalWrite | ~20 | ~5 |

## Libs tierces

| Lib | Dépendance Arduino | ESP-IDF natif |
|---|---|---|
| LovyanGFX | non (détection auto) | oui |
| LVGL | non | oui |
| RadioLib | oui (SPI, GPIO) | HAL ESP-IDF dispo v6+ |
| NeoGPS | oui (HardwareSerial) | non — wrapper à écrire |
| APRSPacketLib | non | oui |

## Ordre de priorité

Phase 0 est faisable immédiatement, sans changer de framework.
Phases 2-3 sont les plus simples (UART, I2C = drivers ESP-IDF bien documentés).
Phase 4 (RadioLib SPI) est le point critique — vérifier le HAL ESP-IDF de RadioLib v6.
Phase 5-6 (WiFi, BLE) sont les plus complexes et les dernières.

## Référence

- RadioLib ESP-IDF HAL : https://github.com/jgromes/RadioLib/tree/master/examples/NonArduino/ESP-IDF
- LovyanGFX ESP-IDF : détection automatique, pas de modification nécessaire
- PlatformIO ESP-IDF + Arduino component : https://docs.platformio.org/en/latest/frameworks/espidf.html
