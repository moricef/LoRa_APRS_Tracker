# Migration Arduino → ESP-IDF (Arduino as component)

## Objectif

`framework = espidf` avec Arduino comme composant ESP-IDF (`arduino-esp32` dans `components/`).
Le code propre appelle ESP-IDF directement. Les libs tierces qui ont besoin d'Arduino
continuent de l'utiliser via le composant. Migration progressive, chaque étape testable.

## État actuel

- Platform : pioarduino (ESP-IDF 5.5.2 / Arduino Core 3.3.7)
- Framework : `arduino` (encore actif dans platformio.ini)
- Branche de travail : `feature/migration-idf5`
- Branche intégrée : `devel-features` (contient tout ce qui est fait)

## Ce qui est fait

| Phase | Description | Gain DRAM | Commit |
|-------|-------------|----------|--------|
| 0 | Migration pioarduino (ESP-IDF 5.5.2) | — | `826814d1` |
| 0 | SPIFFS → LittleFS | — | `553706c7` |
| 0 | NimBLE 1.4 → 2.x, AsyncTCP Carbou | — | `291d2ecf` |
| 0 | Compat ESP-IDF 5.x (WDT, WiFi AP, formats) | — | `b34ff806`→`a1492eb3` |
| 1 | SD.h → POSIX VFS (12 fichiers) | +64 KB | `a639ed23` |
| 2 | PNG/JPEG décodeurs → PSRAM | +109 KB | `e6479b88` |
| — | BLE pause for map, PSRAM queues/stacks | — | divers |

**DRAM libre** : 42 KB → 217 KB au boot, ~106 KB stable en usage.

## Ce qui reste

### Phase 3 — Switch `framework = espidf` + Arduino as component

Le pivot central. Tout le reste en découle.

- [ ] `platformio.ini` : `framework = espidf`
- [ ] Ajouter `arduino-esp32` dans `components/` (ou via `idf_component.yml`)
- [ ] Configurer `sdkconfig` (enfin accessible !)
- [ ] Vérifier que toutes les libs compilent avec ce setup
- [ ] Tester boot complet sur T-Deck Plus

**Gain attendu** : accès sdkconfig complet (buffers WiFi, BT classic off, etc.)

### Phase 4 — sdkconfig tuning (débloqué par phase 3)

Avec `framework = espidf`, les options sdkconfig deviennent modifiables :
- [ ] `CONFIG_BT_CLASSIC_ENABLED=n` (on utilise NimBLE uniquement)
- [ ] Réduire buffers WiFi (`CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM`, etc.)
- [ ] Réduire stack sizes si possible
- [ ] Désactiver composants inutilisés (mDNS si non utilisé, etc.)

### Phase 5 — Migrer le code propre vers ESP-IDF pur (optionnel, progressif)

Ces migrations sont indépendantes et chacune est un commit testable.
Elles réduisent la surface de dépendance au composant Arduino.

| Sous-phase | Fichiers | Difficulté | Gain |
|------------|----------|------------|------|
| UART GPS | `gps_utils.cpp` | ★★ | NeoGPS garde Arduino, mais UART natif |
| I2C clavier/touch | `keyboard_utils.cpp`, `display.cpp` | ★★ | Wire.h éliminé |
| WiFi | `wifi_utils.cpp`, `aprs_is_utils.cpp`, `web_utils.cpp` | ★★★★ | WiFi.h éliminé |
| BLE | `ble_utils.cpp` | ★★★ | NimBLE-Arduino → esp_nimble C direct |

**Ne PAS migrer** : `millis()`, `delay()`, `pinMode()`, `digitalWrite()` — appellent déjà ESP-IDF.

## Libs tierces — inventaire vérifié

### Fonctionnent sans Arduino (rien à faire)

| Lib | Note |
|-----|------|
| LovyanGFX | Auto-détection ESP-IDF/Arduino |
| LVGL | Pur C, pas de dépendance |
| APRSPacketLib | Pur C++ |
| Time | Pas de dépendance directe |
| PNGdec | Callbacks FILE*, quasi prêt |
| Adafruit BME280/BMP280/BME680 | Via BusIO abstraction |

### Nécessitent Arduino as component

| Lib | Dépendance | Alternative ESP-IDF | Effort |
|-----|-----------|---------------------|--------|
| **RadioLib** | `SPI.h`, `Arduino.h` | HAL ESP-IDF existe mais **ESP32 only, pas S3** | ★★★★★ bloquant sans composant Arduino |
| **NeoGPS** | `HardwareSerial` | Parser NMEA custom ou esp_gps | ★★★ |
| **NimBLE-Arduino** | Wrapper C++ sur esp_nimble | esp_nimble C direct | ★★★ |
| **TouchLib** | `Wire.h` | `i2c_master` ESP-IDF | ★★ |
| **OneButton** | millis, digitalRead | Réécrire (~200 lignes) | ★ |
| **AsyncTCP** | Arduino WiFi/lwIP | `esp_http_server` natif | ★★★★ |
| **ESPAsyncWebServer** | dépend d'AsyncTCP | Même problème | ★★★★ |
| **JPEGDEC** | `Arduino.h`, `FS.h` | Fork : retirer includes (callbacks FILE* déjà utilisés) | ★ |

### Le vrai bloquant pour une migration 100% pure

**RadioLib** — le HAL ESP-IDF (`EspHal.h`) a `#error` pour tout sauf ESP32 classique.
L'ESP32-S3 n'est pas supporté. Options :
- Écrire un HAL SPI S3 custom (bus SPI partagé display+SD+LoRa = complexe)
- Contribuer upstream
- **Garder Arduino as component** (solution retenue)

## Leçons apprises

1. **Ne pas wrapper les fonctions de base Arduino** — millis/delay/GPIO appellent déjà ESP-IDF
2. **Les gains DRAM viennent des allocations statiques** (PNGdec BSS, SD.h buffers), pas des wrappers
3. **Les libs pioarduino en mode `framework=arduino` sont précompilées** — sdkconfig inaccessible
4. **Avec `framework=espidf`**, les libs ESP-IDF sont compilées depuis les sources → sdkconfig modifiable
5. **Vérifier la faisabilité 3× avant de proposer une étape**

## Références

- PlatformIO ESP-IDF + Arduino component : https://docs.platformio.org/en/latest/frameworks/espidf.html
- RadioLib ESP-IDF HAL : https://github.com/jgromes/RadioLib/tree/master/examples/NonArduino/ESP-IDF
- LovyanGFX ESP-IDF : détection automatique, pas de modification nécessaire
- IceNav-v3 (Jordi) : référence pour VFS natif + architecture ESP-IDF
