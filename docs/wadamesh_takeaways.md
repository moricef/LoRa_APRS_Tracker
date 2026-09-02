# WadaMesh takeaways

Source reviewed: `/home/fab2/Developpement/wadamesh`

Last reviewed upstream state:
- `origin/main`: `3499ae5` (`beta_66`, 2026-08-19)
- `origin/stable`: `de45b1b` (`beta_65`, 2026-08-13)

This document lists ideas worth studying for LoRa APRS Tracker. It is not a plan
to import WadaMesh wholesale.

## High value

### WiFi/BLE coexistence

WadaMesh does not let WiFi and BLE race each other. The useful pattern is:

- initialize WiFi before BLE so `esp_wifi_init` reserves memory/DMA first;
- use NimBLE instead of Bluedroid where BLE is needed;
- refuse a cold BLE start when internal heap or largest free block is too low;
- keep WiFi and BLE preferences independent;
- never call `WiFi.begin()` directly from UI/Web callbacks;
- UI/Web only requests apply, the main loop performs WiFi state changes;
- pause BLE advertising/connection during critical WiFi association/reconnect,
  then resume BLE after WiFi is stable;
- disable automatic WiFi reconnect when BLE is active and own reconnect in the
  main loop;
- enable `WiFi.setSleep(true)` only after WiFi association, not before scans.

Reference files:
- `/home/fab2/Developpement/wadamesh/src/helpers/esp32/WifiRuntimeStore.cpp`
- `/home/fab2/Developpement/wadamesh/src/helpers/esp32/WifiRuntimeStore.h`
- `/home/fab2/Developpement/wadamesh/src/helpers/esp32/MultiTransportCompanionInterface.cpp`
- `/home/fab2/Developpement/wadamesh/src/main.cpp`

### WiFi scan/reconnect coordination

WadaMesh tracks active scans and prevents reconnect/rebegin while a scan is in
flight. This avoids aborting a scan and returning empty results.

Tracker candidate:
- add a central WiFi state machine;
- add a `scan_active` guard;
- make WebConf/LVGL save credentials and request apply;
- perform disconnect/begin only from the main loop.

### Message storage limits

WadaMesh caps per-chat history, defaulting to a bounded number of messages. This
prevents one busy conversation from slowing every message screen.

Tracker candidate:
- add a configurable per-conversation APRS limit;
- start with a conservative fixed default before exposing settings;
- trim oldest messages when appending, not during UI rendering.

### Non-blocking message persistence

WadaMesh moved chat saves away from UI rendering and avoids tiny writes during
busy traffic.

Tracker candidate:
- avoid rewriting large APRS conversation files from LVGL callbacks;
- batch writes or perform storage work outside render/event callbacks;
- keep UI refresh based on in-memory state, then persist asynchronously where
  practical.

### SD/card and map tile robustness

WadaMesh has useful patterns around tile backend swaps, SD removal/reinsert, tile
queue preservation, and corrupt tile repair.

Tracker candidate:
- support standard OSM tile layouts (`z/x/y`) alongside current regional layout;
- preserve pending tile loads across SD backend changes;
- avoid reading from a backend while it is being swapped or unmounted;
- detect and recover corrupt/missing tiles explicitly.

### SD writes and PSRAM buffers

WadaMesh fixed failures caused by writing to SD from PSRAM-backed buffers.

Tracker candidate:
- for OTA/map/download writes, copy through internal-DMA-safe buffers when needed;
- document which APIs tolerate PSRAM and which do not.

## Medium value

### Sensor throttling

WadaMesh stopped polling BME280 on every home-screen refresh.

Tracker candidate:
- read sensors on a timer;
- cache values for UI/dashboard rendering;
- never perform slow sensor reads from LVGL draw/update paths.

### LoRa radio recovery

WadaMesh has work around radio init failure after update/boot.

Tracker candidate:
- inspect whether our SX126x/SX127x init can fail transiently after flashing;
- add bounded retry and clear user-visible error state if retry succeeds.

### Configuration schema discipline

WadaMesh had a beta regression from inserting a setting into the middle of stored
preferences. The lesson is structural:

- append fields instead of inserting where binary/layout order matters;
- version persisted config explicitly;
- write migrations that repair old layouts;
- test upgrade from previous stable data, not just fresh install.

## Low value for now

### Lua app store

Interesting, but too large for the tracker now. It brings runtime, permissions,
store UI, app packaging, versioning and security surface.

Do not import unless there is a clear product need.

### Full i18n system

Useful long-term, but not urgent. Current priority is stable tracker behavior.

### WadaMesh UI architecture

The whole architecture is not directly portable. Reuse specific patterns, not
large UI subsystems.

## First concrete tracker tasks

1. Build a central WiFi state machine and make WebConf/LVGL request apply only.
2. Add WiFi scan guard so reconnect cannot cancel scans.
3. Add bounded APRS conversation history and trim on append.
4. Audit message file writes from LVGL callbacks.
5. Extend map tile loader to accept standard OSM `z/x/y` layout.
6. Audit SD writes that may use PSRAM buffers.
