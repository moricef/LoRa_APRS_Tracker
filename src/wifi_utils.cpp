/* Copyright (C) 2025 Ricardo Guzman - CA2RXU
 *
 * This file is part of LoRa APRS Tracker.
 *
 * LoRa APRS Tracker is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LoRa APRS Tracker is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LoRa APRS Tracker. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_task_wdt.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <mbedtls/pkcs5.h>
#include <mbedtls/md.h>
#include "configuration.h"
#include "wifi_utils.h"
#include "web_utils.h"
#include "aprs_is_utils.h"
#include "display.h"
#include "lvgl_ui.h"

// DEBUG AP PMK (temporaire) : fonctions internes d'ESP-IDF, deja liees au firmware
extern "C" {
    uint8_t *esp_wifi_ap_get_prof_pmk_internal(void);
    int esp_fast_psk(const char *password, size_t password_len, const uint8_t *ssid, size_t ssid_len,
                     size_t iterations, uint8_t *output, size_t output_len);
}

extern Configuration        Config;

static const char *TAG = "WiFi";

bool        WiFiConnected           = false;
bool        WiFiStationMode         = false;
bool        WiFiEcoMode             = false;    // WiFi sleeping, waiting for retry (network unavailable)
bool        WiFiUserDisabled        = false;    // User manually disabled WiFi (no retry)
uint32_t    noClientsTime           = 0;
uint32_t    previousWiFiMillis      = 0;
uint32_t    lastWiFiDebug           = 0;
uint32_t    lastWiFiRetry           = 0;
const uint32_t WIFI_RETRY_INTERVAL  = 30 * 60 * 1000;  // 30 minutes (battery saving)
const uint32_t WIFI_CONNECT_TIMEOUT = 10 * 1000;       // 10 seconds per network at boot (non-blocking)
const uint32_t WIFI_RECONNECT_TIMEOUT = 30 * 1000;    // 30 seconds for reconnection attempts
const int      WIFI_MAX_RETRIES     = 3;               // Retry 3 times before eco mode
int            wifiRetryCount       = 0;               // Current retry counter

// Non-blocking connection state
bool        wifiConnecting          = false;    // Currently trying to connect
uint32_t    wifiConnectStartTime    = 0;        // When current attempt started
size_t      wifiCurrentNetworkIndex = 0;        // Which network we're trying
bool        wifiIsReconnecting      = false;    // True if this is a reconnect (uses longer timeout)


// =============================================================================
// Native ESP-IDF Wi-Fi driver layer
//
// The firmware drives esp_wifi / esp_netif directly instead of the Arduino WiFi
// class. The driver is initialised without NVS storage: the only source of the
// Wi-Fi configuration is tracker_conf.json, handed to the driver on every start.
// (A persisted soft-AP profile written by an older ESP-IDF left the driver with
// an empty WPA2 key: every client was rejected during the 4-way handshake.)
// =============================================================================

static bool          wifiDriverReady  = false;      // esp_wifi_init() done, netifs created
static bool          wifiRadioStarted = false;      // esp_wifi_start() done
static esp_netif_t*  staNetif         = nullptr;
static esp_netif_t*  apNetif          = nullptr;
static esp_event_handler_instance_t wifiEventInstance = nullptr;
static esp_event_handler_instance_t ipEventInstance   = nullptr;

// Written by the event task, read by the main loop
static volatile bool    staWantConnected        = false;   // the firmware wants the station link up
static volatile bool    staHasIp                = false;   // associated and DHCP lease obtained
static volatile uint8_t staLastDisconnectReason = 0;

static void wifiEventHandler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_DISCONNECTED) {
            const wifi_event_sta_disconnected_t* ev = (const wifi_event_sta_disconnected_t*)data;
            staHasIp = false;
            staLastDisconnectReason = ev->reason;
            // The driver never retries by itself: ask again for as long as the
            // firmware wants the link (the connection timeouts bound the attempts).
            if (staWantConnected && ev->reason != WIFI_REASON_ASSOC_LEAVE) {
                esp_wifi_connect();
            }
        }
    } else if (base == IP_EVENT) {
        if (id == IP_EVENT_STA_GOT_IP) {
            staHasIp = true;
        } else if (id == IP_EVENT_STA_LOST_IP) {
            staHasIp = false;
        }
    }
}

// TCP/IP stack and default event loop (shared with AsyncTCP and the TCP client)
static bool wifiNetStackInit() {
    static bool ready = false;
    if (ready) return true;

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE("WiFi", "esp_netif_init: %s", esp_err_to_name(err));
        return false;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {   // INVALID_STATE: already created
        ESP_LOGE("WiFi", "esp_event_loop_create_default: %s", esp_err_to_name(err));
        return false;
    }
    ready = true;
    return true;
}

static bool wifiDriverInit() {
    if (wifiDriverReady) return true;
    if (!wifiNetStackInit()) return false;

    if (!staNetif) staNetif = esp_netif_create_default_wifi_sta();
    if (!apNetif)  apNetif  = esp_netif_create_default_wifi_ap();
    if (!staNetif || !apNetif) {
        ESP_LOGE("WiFi", "Wi-Fi netif creation failed");
        return false;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable         = 0;     // no persisted Wi-Fi profile: tracker_conf.json is the source
    // Same buffer layout as the Arduino core used: dynamic TX buffers, 4 static RX
    cfg.static_tx_buf_num  = 0;
    cfg.dynamic_tx_buf_num = 32;
    cfg.tx_buf_type        = 1;
    cfg.cache_tx_buf_num   = 4;
    cfg.static_rx_buf_num  = 4;
    cfg.dynamic_rx_buf_num = 32;

    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE("WiFi", "esp_wifi_init: %s", esp_err_to_name(err));
        return false;
    }
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, nullptr, &wifiEventInstance);
    esp_event_handler_instance_register(IP_EVENT,   ESP_EVENT_ANY_ID, &wifiEventHandler, nullptr, &ipEventInstance);

    wifiDriverReady = true;
    return true;
}

// Radio off, driver kept (eco mode, switch between station and access point)
static void wifiRadioStop() {
    staWantConnected = false;
    if (wifiRadioStarted) {
        esp_wifi_disconnect();      // no-op error when not associated or in AP mode
        esp_wifi_stop();
        wifiRadioStarted = false;
    }
    staHasIp = false;
}

// Full release: the driver and both netifs give their DRAM back (needed by BLE)
static void wifiDriverDeinit() {
    wifiRadioStop();
    if (!wifiDriverReady) return;

    if (wifiEventInstance) {
        esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEventInstance);
        wifiEventInstance = nullptr;
    }
    if (ipEventInstance) {
        esp_event_handler_instance_unregister(IP_EVENT, ESP_EVENT_ANY_ID, ipEventInstance);
        ipEventInstance = nullptr;
    }
    if (apNetif)  { esp_netif_destroy_default_wifi(apNetif);  apNetif  = nullptr; }
    if (staNetif) { esp_netif_destroy_default_wifi(staNetif); staNetif = nullptr; }
    esp_err_t err = esp_wifi_deinit();
    if (err != ESP_OK) {
        ESP_LOGW("WiFi", "esp_wifi_deinit: %s", esp_err_to_name(err));
    }
    wifiDriverReady = false;
}

// Station radio on, not yet associated
static bool wifiStartStation(const char* hostname) {
    if (!wifiDriverInit()) return false;
    wifiRadioStop();

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK && hostname && hostname[0]) {
        esp_err_t herr = esp_netif_set_hostname(staNetif, hostname);
        if (herr != ESP_OK) ESP_LOGW("WiFi", "esp_netif_set_hostname: %s", esp_err_to_name(herr));
    }
    if (err == ESP_OK) err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE("WiFi", "Station start failed: %s", esp_err_to_name(err));
        return false;
    }
    wifiRadioStarted = true;
    return true;
}

// Associate with one network (non-blocking: the result comes through the events)
static bool wifiStationConnect(const char* ssid, const char* password) {
    if (!wifiRadioStarted) return false;

    staWantConnected = false;       // the disconnect below must not trigger a retry
    esp_wifi_disconnect();
    staHasIp = false;

    wifi_config_t conf;
    memset(&conf, 0, sizeof(conf));
    memcpy(conf.sta.ssid, ssid, strnlen(ssid, sizeof(conf.sta.ssid)));
    conf.sta.scan_method    = WIFI_FAST_SCAN;
    conf.sta.sort_method    = WIFI_CONNECT_AP_BY_SIGNAL;
    conf.sta.threshold.rssi = -127;
    conf.sta.pmf_cfg.capable = true;
    if (password && password[0]) {
        memcpy(conf.sta.password, password, strnlen(password, sizeof(conf.sta.password)));
        conf.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &conf);
    if (err == ESP_OK) {
        staWantConnected = true;
        err = esp_wifi_connect();
    }
    if (err != ESP_OK) {
        staWantConnected = false;
        ESP_LOGE("WiFi", "Station connect failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

// WPA2 access point. Order documented by ESP-IDF: mode, configuration, start.
static bool wifiStartAP(const char* ssid, const char* password) {
    const size_t passLen = password ? strlen(password) : 0;
    if (passLen < 8 || passLen > 63) {
        ESP_LOGE("WiFi", "AP password must be 8 to 63 characters (got %u)", (unsigned)passLen);
        return false;
    }
    if (!wifiDriverInit()) return false;
    wifiRadioStop();

    wifi_config_t conf;
    memset(&conf, 0, sizeof(conf));
    const size_t ssidLen = strnlen(ssid, sizeof(conf.ap.ssid));
    memcpy(conf.ap.ssid, ssid, ssidLen);
    conf.ap.ssid_len        = ssidLen;
    memcpy(conf.ap.password, password, passLen);
    conf.ap.channel         = 1;
    conf.ap.max_connection  = 4;
    conf.ap.beacon_interval = 100;
    conf.ap.authmode        = WIFI_AUTH_WPA2_PSK;
    conf.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_AP, &conf);
    if (err == ESP_OK) err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE("WiFi", "AP start failed: %s", esp_err_to_name(err));
        return false;
    }
    wifiRadioStarted = true;
    return true;
}

static String ipToString(uint32_t addr) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%u.%u.%u.%u",
             (unsigned)(addr & 0xFF), (unsigned)((addr >> 8) & 0xFF),
             (unsigned)((addr >> 16) & 0xFF), (unsigned)((addr >> 24) & 0xFF));
    return String(buf);
}

static String netifIpString(esp_netif_t* netif) {
    esp_netif_ip_info_t info;
    if (netif && esp_netif_get_ip_info(netif, &info) == ESP_OK) {
        return ipToString(info.ip.addr);
    }
    return String("0.0.0.0");
}


namespace WIFI_Utils {

    // Forward declarations
    void startConnectionAttempt(size_t networkIndex);
    void handleConnectionTimeout();
    void onConnectionSuccess();
    void onAllNetworksFailed();

    static bool wifiInitialized = false;

    void checkWiFi() {
        if (!wifiInitialized) {
            wifiInitialized = true;
            ESP_LOGI(TAG, "Initialized from config: %s", WiFiUserDisabled ? "disabled" : "enabled");
        }

        // User disabled WiFi manually - do nothing
        if (WiFiUserDisabled) {
            return;
        }

        // Handle non-blocking connection in progress
        if (wifiConnecting) {
            uint32_t timeout = wifiIsReconnecting ? WIFI_RECONNECT_TIMEOUT : WIFI_CONNECT_TIMEOUT;

            if (staHasIp) {
                // Connection succeeded!
                wifiConnecting = false;
                onConnectionSuccess();
            } else if ((millis() - wifiConnectStartTime) >= timeout) {
                // Timeout - try next network or give up
                handleConnectionTimeout();
            }
            return;
        }

        // Eco mode: WiFi is off, waiting for periodic retry
        if (WiFiEcoMode && WiFiStationMode) {
            uint32_t elapsed = millis() - lastWiFiRetry;
            if (elapsed >= WIFI_RETRY_INTERVAL) {
                ESP_LOGI(TAG, "Eco mode: retrying after %u ms", elapsed);
                WiFiEcoMode = false;
                wifiIsReconnecting = true;
                startStationMode();
                lastWiFiRetry = millis();
            }
            return;
        }

        if (WiFiConnected) {
            if (millis() - lastWiFiDebug >= 10000) {
                ESP_LOGD(TAG, "link=%d RSSI=%d", (int)staHasIp, getRSSI());
                lastWiFiDebug = millis();
            }
            if (!staHasIp && ((millis() - previousWiFiMillis) >= 30000)) {
                wifiRetryCount++;
                ESP_LOGW(TAG, "Connection lost (reason %u), reconnecting (attempt %d/%d)...",
                         (unsigned)staLastDisconnectReason, wifiRetryCount, WIFI_MAX_RETRIES);
                wifiIsReconnecting = true;
                wifiCurrentNetworkIndex = 0;
                startConnectionAttempt(0);
                return;  // Will be handled in non-blocking section above
            }
        }
    }

    // Start a non-blocking connection attempt to a specific network
    void startConnectionAttempt(size_t networkIndex) {
        // Find next valid network
        while (networkIndex < Config.wifiAPs.size() && Config.wifiAPs[networkIndex].ssid == "") {
            networkIndex++;
        }

        if (networkIndex >= Config.wifiAPs.size()) {
            // No more networks to try
            onAllNetworksFailed();
            return;
        }

        wifiCurrentNetworkIndex = networkIndex;
        wifiConnecting = true;
        wifiConnectStartTime = millis();

        const WiFi_AP& network = Config.wifiAPs[networkIndex];
        uint32_t timeout = wifiIsReconnecting ? WIFI_RECONNECT_TIMEOUT : WIFI_CONNECT_TIMEOUT;
        ESP_LOGI(TAG, "Connecting to '%s' (timeout %ds, non-blocking)...",
            network.ssid.c_str(), timeout / 1000);
        wifiStationConnect(network.ssid.c_str(), network.password.c_str());
    }

    // Called when connection timeout occurs
    void handleConnectionTimeout() {
        ESP_LOGW(TAG, "Timeout connecting to '%s' (last reason %u)",
            Config.wifiAPs[wifiCurrentNetworkIndex].ssid.c_str(), (unsigned)staLastDisconnectReason);
        staWantConnected = false;
        esp_wifi_disconnect();

        // Try next network
        startConnectionAttempt(wifiCurrentNetworkIndex + 1);
    }

    // Pose les DNS publics sans desactiver DHCP.
    static void setPublicDNS() {
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (!netif) {
            ESP_LOGW(TAG, "STA netif introuvable, DNS inchanges");
            return;
        }
        esp_netif_dns_info_t dns;
        dns.ip.type = ESP_IPADDR_TYPE_V4;

        dns.ip.u_addr.ip4.addr = esp_netif_htonl(0x01010101);   // 1.1.1.1 Cloudflare
        esp_err_t err = esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);

        dns.ip.u_addr.ip4.addr = esp_netif_htonl(0x08080808);   // 8.8.8.8 Google
        if (err == ESP_OK) {
            err = esp_netif_set_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns);
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_netif_set_dns_info: 0x%04x", err);
        }
    }

    // Called when connection succeeds
    void onConnectionSuccess() {
        WiFiConnected = true;
        WiFiEcoMode = false;
        wifiRetryCount = 0;
        wifiIsReconnecting = false;
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // Enable modem sleep for coexistence

        // Use public DNS servers (Cloudflare + Google) for reliable resolution
        // DHCP-provided DNS on some routers is slow or unreliable for the ESP32.
        // Only the DNS servers are overridden: the DHCP client keeps running, so
        // the address follows the network when the tracker moves to another AP.
        setPublicDNS();

        ESP_LOGI(TAG, "Connected to '%s'! IP=%s RSSI=%d dBm DNS=1.1.1.1/8.8.8.8",
            Config.wifiAPs[wifiCurrentNetworkIndex].ssid.c_str(),
            getLocalIP().c_str(), getRSSI());
        WEB_Utils::setup();
    }

    // Called when all networks failed to connect
    void onAllNetworksFailed() {
        wifiConnecting = false;
        wifiRetryCount++;
        ESP_LOGW(TAG, "All networks failed (attempt %d/%d)", wifiRetryCount, WIFI_MAX_RETRIES);

        if (wifiRetryCount >= WIFI_MAX_RETRIES) {
            WiFiConnected = false;
            WiFiEcoMode = true;
            wifiRetryCount = 0;
            lastWiFiRetry = millis();
            ESP_LOGW(TAG, "Max retries reached, entering eco mode (retry in 30 min)");
            #ifdef USE_LVGL_UI
                LVGL_UI::showWiFiEcoMode();
            #else
                displayShow("", " WiFi Eco Mode", "  Retry in 30 min", "", "", "", 1000);
            #endif
            wifiRadioStop();
        } else {
            // Retry from first network
            wifiCurrentNetworkIndex = 0;
            startConnectionAttempt(0);
        }
    }

    // DEBUG AP PMK (temporaire, ne pas committer) : compare la cle WPA2 que le
    // pilote Wi-Fi utilise pour l'AP a celles recalculees ici.
    static void hex32(const uint8_t* p, char* out) {
        static const char d[] = "0123456789abcdef";
        for (int i = 0; i < 32; i++) { out[2*i] = d[p[i] >> 4]; out[2*i+1] = d[p[i] & 15]; }
        out[64] = 0;
    }
    static void logApPmkDiag(const char* ssid, const char* pass) {
        char h[65];
        wifi_config_t conf;
        memset(&conf, 0, sizeof(conf));
        esp_err_t err = esp_wifi_get_config(WIFI_IF_AP, &conf);
        ESP_LOGW(TAG, "[PMK] get_config=%s ssid='%.32s' ssid_len=%u pwd_len=%u auth=%d cipher=%d",
                 esp_err_to_name(err), conf.ap.ssid, (unsigned)conf.ap.ssid_len,
                 (unsigned)strnlen((const char*)conf.ap.password, 64),
                 (int)conf.ap.authmode, (int)conf.ap.pairwise_cipher);

        const uint8_t* drv = esp_wifi_ap_get_prof_pmk_internal();
        if (drv) { hex32(drv, h); ESP_LOGW(TAG, "[PMK] pilote t0      : %s", h); }
        else     { ESP_LOGW(TAG, "[PMK] pilote t0      : (null)"); }
        delay(1500);   // laisse le pilote finir un eventuel calcul differe
        drv = esp_wifi_ap_get_prof_pmk_internal();
        if (drv) { hex32(drv, h); ESP_LOGW(TAG, "[PMK] pilote t+1.5s  : %s", h); }
        else     { ESP_LOGW(TAG, "[PMK] pilote t+1.5s  : (null)"); }

        uint8_t k[32];
        memset(k, 0, sizeof(k));
        int rc = esp_fast_psk(pass, strlen(pass), (const uint8_t*)ssid, strlen(ssid), 4096, k, sizeof(k));
        hex32(k, h); ESP_LOGW(TAG, "[PMK] esp_fast_psk    : %s (rc=%d)", h, rc);

        memset(k, 0, sizeof(k));
        rc = mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA1, (const unsigned char*)pass, strlen(pass),
                                           (const unsigned char*)ssid, strlen(ssid), 4096, sizeof(k), k);
        hex32(k, h); ESP_LOGW(TAG, "[PMK] mbedtls pbkdf2  : %s (rc=%d)", h, rc);

        ESP_LOGW(TAG, "[PMK] attendu (PC)    : 4cf9289e5cabfaba91d3f24a391feb98a7664ff982ab230564c60c7d5189386d"
                      " (LoRa-Tracker-AP / 1234567890)");
    }

    void startBlockingWebConfig() {
        String apName = "LoRa-Tracker-AP";

        displayShow(" LoRa APRS", "    ** WEB-CONF **", "", "WiFiAP: " + apName, "IP    : 192.168.4.1", "", 0);
        ESP_LOGW(TAG, "WebConfiguration Started!");

        if (wifiStartAP(apName.c_str(), Config.wifiAutoAP.password.c_str())) {
            logApPmkDiag(apName.c_str(), Config.wifiAutoAP.password.c_str());
        } else {
            ESP_LOGE(TAG, "Failed to start AP");
        }

        WEB_Utils::setup();

        while (true) {
            if (getAPStationCount() > 0) {
                noClientsTime = 0;
            } else {
                if (noClientsTime == 0) {
                    noClientsTime = millis();
                } else if ((millis() - noClientsTime) > Config.wifiAutoAP.timeout * 60 * 1000) {
                    ESP_LOGW(TAG, "WebConfiguration Stopped!");
                    displayShow("", "", "  STOPPING WiFi AP", "", "", "", 2000);
                    Config.wifiAutoAP.active = false;
                    Config.writeFile();
                    wifiDriverDeinit();
                    ESP.restart();
                }
            }
        }
    }

    void startStationMode() {
        WiFiStationMode = true;

        String hostName = "Tracker-";
        if (Config.beacons.size() > 0) {
            hostName += Config.beacons[0].callsign;
        }

        // Clean radio restart in station mode (also needed after the eco-mode stop)
        if (!wifiStartStation(hostName.c_str())) {
            WiFiConnected = false;
            wifiConnecting = false;
            return;
        }

        // Start non-blocking connection attempt
        wifiCurrentNetworkIndex = 0;
        startConnectionAttempt(0);
    }

    bool needsWebConfig() {
        // Check with bounds safety - if vectors are empty, we need web config
        if (Config.beacons.size() == 0) {
            return true;
        }
        return (Config.wifiAutoAP.active ||
                Config.beacons[0].callsign == "NOCALL-7");
    }

    void setup() {
        // btStop() removed for WiFi/BLE coexistence
        // Modem sleep (WIFI_PS_MIN_MODEM) is enabled after connection

        #ifdef USE_LVGL_UI
            // WiFi no longer starts at boot — manual activation only via Settings.
            // First boot web-conf (NOCALL/no WiFi) is handled by main after LVGL init.
            WiFiUserDisabled = true;
        #else
            // Blocking web-conf mode if enabled, callsign NOCALL, or no WiFi network configured
            if (needsWebConfig()) {
                startBlockingWebConfig();
                // Never returns here - reboot after config
            }
            // Station Mode: connect to the configured WiFi network (if enabled)
            WiFiUserDisabled = !Config.wifiEnabled;
            if (Config.wifiEnabled) {
                startStationMode();
            }
        #endif
    }

    void stop() {
        ESP_LOGI(TAG, "Stopping WiFi");
        APRS_IS_Utils::disconnect();
        WEB_Utils::stop();
        wifiDriverDeinit();     // radio, driver and netifs: gives the DRAM back (BLE needs it)
        WiFiConnected = false;
        wifiConnecting = false;
        WiFiEcoMode = false;
    }

    bool isConnected() {
        // Etat reel de la pile, pas le drapeau WiFiConnected : celui-ci reste vrai
        // pendant toute une perte de liaison (30 s avant reaction, remis a false
        // seulement apres echec de tous les reseaux). Sur un hotspot mobile en
        // deplacement, cela faisait ecrire APRS-IS dans un socket deja mort.
        return staHasIp;
    }

    String getLocalIP() {
        return netifIpString(staNetif);
    }

    int getRSSI() {
        wifi_ap_record_t ap;
        if (wifiRadioStarted && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            return ap.rssi;
        }
        return 0;
    }

    uint8_t getAPStationCount() {
        wifi_sta_list_t list;
        if (wifiRadioStarted && esp_wifi_ap_get_sta_list(&list) == ESP_OK) {
            return (uint8_t)list.num;
        }
        return 0;
    }

    // Blocking DNS lookup through lwIP (IPv4)
    bool resolveHost(const char* host, IPAddress& result) {
        struct addrinfo hints;
        memset(&hints, 0, sizeof(hints));
        hints.ai_family   = AF_INET;
        hints.ai_socktype = SOCK_STREAM;

        struct addrinfo* res = nullptr;
        if (getaddrinfo(host, nullptr, &hints, &res) != 0 || res == nullptr) {
            return false;
        }
        const uint32_t addr = ((struct sockaddr_in*)res->ai_addr)->sin_addr.s_addr;
        freeaddrinfo(res);
        result = IPAddress((uint8_t)(addr & 0xFF), (uint8_t)((addr >> 8) & 0xFF),
                           (uint8_t)((addr >> 16) & 0xFF), (uint8_t)((addr >> 24) & 0xFF));
        return true;
    }

    // Non-blocking AP mode for LVGL UI
    bool startAPModeNonBlocking() {
        String apName = "LoRa-Tracker-AP";

        ESP_LOGW(TAG, "Starting AP Mode: %s", apName.c_str());

        // Leave station mode: the services bound to it must stop first
        APRS_IS_Utils::disconnect();
        WEB_Utils::stop();

        bool success = wifiStartAP(apName.c_str(), Config.wifiAutoAP.password.c_str());
        if (success) {
            logApPmkDiag(apName.c_str(), Config.wifiAutoAP.password.c_str());
            ESP_LOGI(TAG, "AP Started - IP: %s", netifIpString(apNetif).c_str());
            WEB_Utils::setup();
            WiFiConnected = false;
            WiFiStationMode = false;
            return true;
        } else {
            ESP_LOGE(TAG, "Failed to start AP");
            return false;
        }
    }

    String getAPName() {
        return "LoRa-Tracker-AP";
    }

    String getStatusLine() {
        if (!WiFiStationMode) {
            return "";
        }
        if (WiFiConnected) {
            return "WiFi< " + getLocalIP();
        }
        if (WiFiEcoMode) {
            return "WiFi< Eco (sleep)";
        }
        return "WiFi< No connection";
    }

}
