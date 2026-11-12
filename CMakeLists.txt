cmake_minimum_required(VERSION 3.16.0)

# Exclusion officielle ESP-IDF des composants managés inutiles qui bloquent la compilation (ex: esp_insights)
set(EXCLUDE_COMPONENTS
    espressif__esp_insights
    espressif__esp_diagnostics
    espressif__esp_diag_data_store
    espressif__esp_rainmaker
    espressif__rmaker_common
    espressif__esp_schedule
    espressif__esp_secure_cert_mgr
    espressif__network_provisioning
    espressif__esp-sr
    espressif__esp-zboss-lib
    espressif__esp-zigbee-lib
    espressif__esp_rcp_update
    espressif__esp-modbus
    espressif__esp_modem
    espressif__esp-serial-flasher
    espressif__qrcode
    espressif__jsmn
    espressif__json_generator
    espressif__json_parser
    espressif__cbor
    espressif__libsodium
    chmorgan__esp-libhelix-mp3
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(LoRa_APRS_Tracker-devel)
