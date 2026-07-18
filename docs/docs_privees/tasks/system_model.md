# Task : Créer docs/system_model.md

## Objectif

Centraliser le modèle système (mémoire, états, contraintes) dans un fichier unique, synthétique et contraignant.

**Critère de validation :** un dev externe évite 90% des pièges en lisant ce fichier en 2 minutes.

## Problème

Le modèle existe mais est dispersé entre CLAUDE.md, memory/, docs/tasks/, commentaires code, logs série. Ni un humain dans 3 semaines ni un LLM ne peut le reconstituer efficacement.

## Contenu attendu

### A. Budget mémoire

Tableau : composant, DRAM usage, PSRAM usage, contraintes (contigu/non contigu, pool requis).

Chiffres vérifiés par lecture du code et des logs, pas de mémoire.

### B. Contraintes critiques

Forme impérative, pas narrative :
- BLE controller : bloc contigu ≥ X KB requis à `esp_bt_controller_init`
- WiFi + BLE simultané : interdit (raison)
- BLE + Map simultané : interdit (raison)
- WiFi + Map : OK
- PSRAM ≠ solution pour controller BLE (`libbt.a` = `MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA`)
- Ordre d'init impacte la fragmentation

### C. Machine d'état système

États : IDLE, WIFI_ACTIVE, BLE_ACTIVE, MAP_ACTIVE, MAP+WIFI

Transitions autorisées et interdites. Diagramme texte.

### D. Procédures critiques

Séquences strictes (pas de prose) pour :
- Switch WiFi → BLE
- Switch BLE → WiFi
- Entrée carte (BLE pause)
- Sortie carte (BLE resume)

### E. Invariants vérifiables

Logs/asserts que le firmware doit maintenir :
- Heap stats périodiques (déjà en place : toutes les 10s)
- Largest block avant BLE init
- Heap total avant/après WiFi start

## Sources à lire

- `CLAUDE.md` (sections mémoire, patterns techniques, invariants)
- `memory/MEMORY.md` + fichiers memory référencés
- `docs/tasks/reboot_wifi_ble.md` (exclusivité WiFi/BLE/Map, bilan)
- `src/LoRa_APRS_Tracker.cpp` (logs heap, init sequence)
- `src/ui_dashboard.cpp` (BLE pause à l'entrée carte)
- `src/map/map_input.cpp` (BLE resume à la sortie carte)
- `src/map/map_engine.cpp` (queues PSRAM, sprites)
- `variants/ttgo_t_deck_plus_433/platformio.ini` (build flags mémoire)

## Règle

Chaque chiffre du document doit être vérifié dans le code ou les logs. Pas de valeurs inventées ou mémorisées.
