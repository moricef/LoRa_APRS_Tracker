# Migration Arduino → ESP-IDF (Arduino as component)

## Objectif

`framework = espidf` avec Arduino comme composant ESP-IDF (`arduino-esp32` dans `components/`).
Le code propre appelle ESP-IDF directement. Les libs tierces qui ont besoin d'Arduino
continuent de l'utiliser via le composant. Migration progressive, chaque étape testable.

## État actuel

- Platform : pioarduino (ESP-IDF 5.5.2 / Arduino Core 3.3.7)
- Framework : `espidf, arduino` (dual framework actif)
- Branche de travail : `feature/migration-idf5`
- Branche intégrée : `devel-features` (contient tout ce qui est fait)
- **Phase 3 terminée** : compilation OK, test hardware réussi (carte + messages)

## Ce qui est fait

| Phase | Description | Gain DRAM | Commit |
|-------|-------------|----------|--------|
| 0 | Migration pioarduino (ESP-IDF 5.5.2) | — | `826814d1` |
| 0 | SPIFFS → LittleFS | — | `553706c7` |
| 0 | NimBLE 1.4 → 2.x, AsyncTCP Carbou | — | `291d2ecf` |
| 0 | Compat ESP-IDF 5.x (WDT, WiFi AP, formats) | — | `b34ff806`→`a1492eb3` |
| 1 | SD.h → POSIX VFS (12 fichiers) | +64 KB | `a639ed23` |
| 2 | PNG/JPEG décodeurs → PSRAM | +109 KB | `e6479b88` |
| 3 | `framework = espidf, arduino` + stub component + LFN | +0 KB (accès sdkconfig) | `(à venir)` |
| — | BLE pause for map, PSRAM queues/stacks | — | divers |

**DRAM libre** : 42 KB → 217 KB au boot, ~106 KB stable en usage.

## Ce qui reste

### Phase 3 — Switch `framework = espidf` + Arduino as component ✅ **TERMINÉE**

Le pivot central. Tout le reste en découle.

- [x] `platformio.ini` : `framework = espidf, arduino` (dual framework)
- [x] Ajouter `arduino-esp32` dans `components/` (ou via `idf_component.yml`) → **implicite via pioarduino**
- [x] Configurer `sdkconfig` (enfin accessible !) → **`sdkconfig.defaults` activé**
- [x] Vérifier que toutes les libs compilent avec ce setup → **build réussi (1 min 03 s)**
- [x] Tester boot complet sur T‑Deck Plus → **validé (carte + messages fonctionnels)**

**Gain obtenu** : accès sdkconfig complet, activation LFN FatFS, exclusion des composants ESP‑IDF inutiles.

**Solutions techniques** :
- **Stub component local** : `components/espressif__esp_insights/` (CMakeLists.txt minimal) prend priorité sur le managed component, évite l'embarquement du certificat `https_server.crt.S`.
- **Exclusion CMake** : `set(EXCLUDE_COMPONENTS ...)` dans `CMakeLists.txt` racine exclut 25 composants managés inutiles (`esp-modbus`, `esp-sr`, etc.).
- **FatFS LFN** : `CONFIG_FATFS_LFN_HEAP=y` et `CONFIG_FATFS_MAX_LFN=255` dans `sdkconfig.defaults` permet le chemin `LoRa_Tracker` (11 caractères).
- **Chemins VFS** : tous les appels POSIX (`fopen`, `stat`, `opendir`) utilisent `SD_MOUNT_POINT "/sd/LoRa_Tracker/..."`. Les messages de log affichent le chemin complet.
- **Dual framework stable** : `framework = espidf, arduino` fonctionne, RadioLib garde Arduino comme composant (HAL ESP‑IDF non supporté pour ESP32‑S3).

**Statistiques build** :
- RAM : 101 176 B (30.9%)
- Flash : 2 493 709 B (79.3%)
- DRAM libre au boot : ~217 KB

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
6. **Gérer les composants ESP‑IDF indésirables** via stub component local + `EXCLUDE_COMPONENTS` dans CMakeLists.txt, pas seulement `custom_component_remove` de PlatformIO.
7. **Respecter les chemins VFS** : en mode `framework = espidf`, tous les accès SD doivent être préfixés par `SD_MOUNT_POINT`. Les utilitaires de storage abstraient cette différence.

## Solutions validées

- **Stub component local** : créer `components/espressif__esp_insights/CMakeLists.txt` minimal (`idf_component_register(INCLUDE_DIRS "include")`) pour shadow le managed component et éviter l'embarquement forcé du certificat `https_server.crt.S`.
- **Exclusion CMake** : `set(EXCLUDE_COMPONENTS ...)` dans le `CMakeLists.txt` racine (avant `include($ENV{IDF_PATH}/tools/cmake/project.cmake)`) exclut les composants managés inutiles. Noms complets avec namespace (`espressif__esp_insights`, etc.).
- **FatFS LFN** : `CONFIG_FATFS_LFN_HEAP=y` et `CONFIG_FATFS_MAX_LFN=255` dans `sdkconfig.defaults` (chargé automatiquement en mode `framework = espidf`) permet les chemins >8.3 comme `LoRa_Tracker`.
- **Chemins VFS** : tous les appels POSIX directs (`fopen`, `stat`, `opendir`) doivent utiliser `SD_MOUNT_POINT "/sd/LoRa_Tracker/..."`. Les utilitaires (`STORAGE_Utils::fileExists()`, `sdPath()`) ajoutent automatiquement le préfixe.
- **Message de log** : les logs d'erreur doivent afficher le chemin complet (`ESP_LOGE(TAG, "Cannot open %s/LoRa_Tracker/Maps", SD_MOUNT_POINT)`), pas le chemin logique.

## Références

- PlatformIO ESP-IDF + Arduino component : https://docs.platformio.org/en/latest/frameworks/espidf.html
- RadioLib ESP-IDF HAL : https://github.com/jgromes/RadioLib/tree/master/examples/NonArduino/ESP-IDF
- LovyanGFX ESP-IDF : détection automatique, pas de modification nécessaire
- IceNav-v3 (Jordi) : référence pour VFS natif + architecture ESP-IDF
