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

#include <LittleFS.h>
#include <SD.h>          // Only for SD.begin() — all file ops use POSIX C via VFS
#include <SPI.h>
#include <vector>
#include <algorithm>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <esp_log.h>
#include <sys/stat.h>
#include <esp_vfs_fat.h>
#include <dirent.h>
#include <unistd.h>
#include <cerrno>
#include "board_pinout.h"
#include "storage_utils.h"

static const char *TAG = "Storage";

static bool sdAvailable = false;

// In-memory contacts cache
static std::vector<Contact> contactsCache;
static bool contactsLoaded = false;

// Root directory for all tracker data on SD card (VFS paths)
static const char* ROOT_DIR      = SD_MOUNT_POINT "/LoRa_Tracker";
static const char* MESSAGES_DIR  = SD_MOUNT_POINT "/LoRa_Tracker/Messages";
static const char* INBOX_DIR     = SD_MOUNT_POINT "/LoRa_Tracker/Messages/inbox";
static const char* OUTBOX_DIR    = SD_MOUNT_POINT "/LoRa_Tracker/Messages/outbox";
static const char* CONTACTS_DIR  = SD_MOUNT_POINT "/LoRa_Tracker/Contacts";
static const char* CONTACTS_FILE = SD_MOUNT_POINT "/LoRa_Tracker/Contacts/contacts.json";
static const char* MAPS_DIR      = SD_MOUNT_POINT "/LoRa_Tracker/Maps";
static const char* SYMBOLS_DIR   = SD_MOUNT_POINT "/LoRa_Tracker/Symbols";

// Helper: check if a VFS path exists
static bool vfs_exists(const char* path) {
    struct stat st;
    return (stat(path, &st) == 0);
}

// Helper: get file size via stat
static long vfs_filesize(const char* path) {
    struct stat st;
    if (stat(path, &st) == 0) return st.st_size;
    return -1;
}

namespace STORAGE_Utils {

    String sdPath(const String& path) {
        if (path.startsWith(SD_MOUNT_POINT)) return path;
        return String(SD_MOUNT_POINT) + path;
    }

    void createDirectoryStructure() {
        if (!sdAvailable) return;

        const char* dirs[] = { ROOT_DIR, MESSAGES_DIR, INBOX_DIR, OUTBOX_DIR,
                               CONTACTS_DIR, MAPS_DIR, SYMBOLS_DIR };
        for (const char* d : dirs) {
            if (!vfs_exists(d)) {
                ::mkdir(d, 0775);
                ESP_LOGI(TAG, "Created %s", d);
            }
        }
    }

    void setup() {
        // Always init LittleFS as fallback (format on fail for first boot)
        if (!LittleFS.begin(true, "/littlefs", 10, "spiffs")) {
            ESP_LOGE(TAG, "LittleFS mount failed. Forcing format...");
            LittleFS.format();
            if (!LittleFS.begin(true, "/littlefs", 10, "spiffs")) {
                ESP_LOGE(TAG, "LittleFS format and mount failed permanently");
            } else {
                ESP_LOGI(TAG, "LittleFS mounted after format");
            }
        } else {
            ESP_LOGI(TAG, "LittleFS mounted");
        }

        #ifdef BOARD_SDCARD_CS
            // Try to init SD card on shared SPI bus
            // SD card uses the same SPI as display/LoRa on T-Deck Plus
            SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);

            if (SD.begin(BOARD_SDCARD_CS, SPI, 10000000)) {  // 10 MHz (was 20 MHz)
                sdAvailable = true;
                uint8_t cardType = SD.cardType();

                if (cardType == CARD_NONE) {
                    ESP_LOGW(TAG, "No SD card inserted");
                    sdAvailable = false;
                } else {
                    const char* typeStr = "UNKNOWN";
                    if (cardType == CARD_MMC) typeStr = "MMC";
                    else if (cardType == CARD_SD) typeStr = "SDSC";
                    else if (cardType == CARD_SDHC) typeStr = "SDHC";

                    ESP_LOGI(TAG, "SD card mounted (%s, %lluMB)",
                        typeStr, SD.cardSize() / (1024 * 1024));


                    // Create directory structure
                    createDirectoryStructure();

                    // Load last 20 frames from SD into RAM cache
                    loadFramesFromSD();
                    }
                    } else {
                    ESP_LOGE(TAG, "SD card init failed, using LittleFS");
                    sdAvailable = false;
                    }
                    #else
                    ESP_LOGW(TAG, "No SD card support, using LittleFS");
                    #endif
                    }

    bool isSDAvailable() {
        return sdAvailable;
    }

    // Get paths to different directories
    String getRootPath() {
        return sdAvailable ? String(ROOT_DIR) : "";
    }

    String getMessagesPath() {
        return sdAvailable ? String(MESSAGES_DIR) : "";
    }

    String getInboxPath() {
        return sdAvailable ? String(INBOX_DIR) : "";
    }

    String getOutboxPath() {
        return sdAvailable ? String(OUTBOX_DIR) : "";
    }

    String getContactsPath() {
        return sdAvailable ? String(CONTACTS_DIR) : "";
    }

    String getMapsPath() {
        return sdAvailable ? String(MAPS_DIR) : "";
    }

    bool fileExists(const String& path) {
        if (sdAvailable) {
            if (path.startsWith(SD_MOUNT_POINT)) {
                return vfs_exists(path.c_str());
            }
            if (path.startsWith("/LoRa_Tracker")) {
                return vfs_exists(sdPath(path).c_str());
            }
            String full = String(MESSAGES_DIR) + path;
            return vfs_exists(full.c_str());
        }
        return LittleFS.exists(path);
    }

    FILE* openFile(const String& path, const char* mode) {
        String fullPath;
        if (sdAvailable) {
            if (path.startsWith(SD_MOUNT_POINT)) {
                fullPath = path;
            } else if (path.startsWith("/LoRa_Tracker")) {
                fullPath = sdPath(path);
            } else {
                fullPath = String(MESSAGES_DIR) + path;
            }
        } else {
            fullPath = String("/littlefs") + path;
        }
        if (strcmp(mode, "r") == 0 && !vfs_exists(fullPath.c_str())) {
            return nullptr;
        }
        return fopen(fullPath.c_str(), mode);
    }

    bool removeFile(const String& path) {
        if (sdAvailable) {
            String fullPath;
            if (path.startsWith(SD_MOUNT_POINT)) {
                fullPath = path;
            } else if (path.startsWith("/LoRa_Tracker")) {
                fullPath = sdPath(path);
            } else {
                fullPath = String(MESSAGES_DIR) + path;
            }
            return (::remove(fullPath.c_str()) == 0);
        }
            return LittleFS.remove(path);
            }

    bool sdMkdir(const String& path) {
        if (sdAvailable) {
            String fullPath;
            if (path.startsWith(SD_MOUNT_POINT)) {
                fullPath = path;
            } else if (path.startsWith("/LoRa_Tracker")) {
                fullPath = sdPath(path);
            } else {
                fullPath = String(MESSAGES_DIR) + path;
            }
            return (::mkdir(fullPath.c_str(), 0775) == 0 || errno == EEXIST);
        }
        return true;
    }

    // List files in a directory (SD only)
    std::vector<String> listFiles(const String& dirPath) {
        std::vector<String> files;
        if (!sdAvailable) return files;

        DIR* dir = opendir(dirPath.c_str());
        if (!dir) return files;

        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (entry->d_type == DT_REG) {
                files.push_back(String(entry->d_name));
            }
        }
        closedir(dir);
        return files;
    }

    // List subdirectories in a directory (SD only)
    std::vector<String> listDirs(const String& dirPath) {
        std::vector<String> dirs;
        if (!sdAvailable) return dirs;

        DIR* dir = opendir(dirPath.c_str());
        if (!dir) return dirs;

        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (entry->d_type == DT_DIR) {
                dirs.push_back(String(entry->d_name));
            }
        }
        closedir(dir);
        return dirs;
    }

    String getStorageType() {
        return sdAvailable ? "SD" : "LittleFS";
    }

    uint64_t getUsedBytes() {
        if (sdAvailable) {
            FATFS* fs;
            DWORD fre_clust;
            if (f_getfree("0:", &fre_clust, &fs) == FR_OK) {
                uint64_t total = (uint64_t)(fs->n_fatent - 2) * fs->csize * 512;
                uint64_t free  = (uint64_t)fre_clust * fs->csize * 512;
                return total - free;
            }
            return 0;
        }
        return LittleFS.usedBytes();
    }

    uint64_t getTotalBytes() {
        if (sdAvailable) {
            FATFS* fs;
            DWORD fre_clust;
            if (f_getfree("0:", &fre_clust, &fs) == FR_OK) {
                return (uint64_t)(fs->n_fatent - 2) * fs->csize * 512;
            }
            return 0;
        }
        return LittleFS.totalBytes();
    }

    // ========== Contacts Management ==========

    std::vector<Contact> loadContacts() {
        if (contactsLoaded) {
            return contactsCache;
        }

        contactsCache.clear();

        if (!sdAvailable) {
            ESP_LOGW(TAG, "No SD card, contacts not available");
            return contactsCache;
        }

        long fsize = vfs_filesize(CONTACTS_FILE);
        if (fsize <= 0) {
            ESP_LOGW(TAG, "No contacts file, starting fresh");
            contactsLoaded = true;
            return contactsCache;
        }

        FILE* f = fopen(CONTACTS_FILE, "r");
        if (!f) {
            contactsLoaded = true;
            return contactsCache;
        }

        char* buf = (char*)malloc(fsize + 1);
        if (!buf) { fclose(f); contactsLoaded = true; return contactsCache; }
        size_t rd = fread(buf, 1, fsize, f);
        fclose(f);
        buf[rd] = '\0';

        DynamicJsonDocument doc(4096);
        DeserializationError error = deserializeJson(doc, buf);
        free(buf);

        if (error) {
            ESP_LOGE(TAG, "JSON parse error: %s", error.c_str());
            contactsLoaded = true;
            return contactsCache;
        }

        JsonArray array = doc.as<JsonArray>();
        for (JsonObject obj : array) {
            Contact c;
            c.callsign = obj["callsign"] | "";
            c.name = obj["name"] | "";
            c.comment = obj["comment"] | "";
            if (c.callsign.length() > 0) {
                contactsCache.push_back(c);
            }
        }

        ESP_LOGI(TAG, "Loaded %d contacts", contactsCache.size());
        contactsLoaded = true;
        return contactsCache;
    }

    bool saveContacts(const std::vector<Contact>& contacts) {
        if (!sdAvailable) {
            ESP_LOGW(TAG, "No SD card, cannot save contacts");
            return false;
        }

        DynamicJsonDocument doc(4096);
        JsonArray array = doc.to<JsonArray>();

        for (const Contact& c : contacts) {
            JsonObject obj = array.createNestedObject();
            obj["callsign"] = c.callsign;
            obj["name"] = c.name;
            obj["comment"] = c.comment;
        }

        FILE* f = fopen(CONTACTS_FILE, "w");
        if (!f) {
            ESP_LOGE(TAG, "Failed to open contacts file for writing");
            return false;
        }

        char buf[4096];
        size_t len = serializeJsonPretty(doc, buf, sizeof(buf));
        fwrite(buf, 1, len, f);
        fclose(f);

        contactsCache = contacts;
        contactsLoaded = true;

        ESP_LOGI(TAG, "Saved %d contacts", contacts.size());
        return true;
    }

    bool addContact(const Contact& contact) {
        // Load existing contacts if not loaded
        if (!contactsLoaded) {
            loadContacts();
        }

        // Check if callsign already exists
        for (const Contact& c : contactsCache) {
            if (c.callsign.equalsIgnoreCase(contact.callsign)) {
                ESP_LOGW(TAG, "Contact %s already exists", contact.callsign.c_str());
                return false;
            }
        }

        // Add to cache
        contactsCache.push_back(contact);

        // Save to file
        return saveContacts(contactsCache);
    }

    bool removeContact(const String& callsign) {
        if (!contactsLoaded) {
            loadContacts();
        }

        for (auto it = contactsCache.begin(); it != contactsCache.end(); ++it) {
            if (it->callsign.equalsIgnoreCase(callsign)) {
                contactsCache.erase(it);
                ESP_LOGI(TAG, "Removed contact %s", callsign.c_str());
                return saveContacts(contactsCache);
            }
        }

        ESP_LOGW(TAG, "Contact %s not found", callsign.c_str());
        return false;
    }

    bool updateContact(const String& callsign, const Contact& newData) {
        if (!contactsLoaded) {
            loadContacts();
        }

        for (Contact& c : contactsCache) {
            if (c.callsign.equalsIgnoreCase(callsign)) {
                c.callsign = newData.callsign;
                c.name = newData.name;
                c.comment = newData.comment;
                ESP_LOGI(TAG, "Updated contact %s", callsign.c_str());
                return saveContacts(contactsCache);
            }
        }

        ESP_LOGW(TAG, "Contact %s not found for update", callsign.c_str());
        return false;
    }

    Contact* findContact(const String& callsign) {
        if (!contactsLoaded) {
            loadContacts();
        }

        for (Contact& c : contactsCache) {
            if (c.callsign.equalsIgnoreCase(callsign)) {
                return &c;
            }
        }
        return nullptr;
    }

    int getContactCount() {
        if (!contactsLoaded) {
            loadContacts();
        }
        return contactsCache.size();
    }

    // ========== Stats Persistence ==========

    static const char* STATS_FILE = SD_MOUNT_POINT "/LoRa_Tracker/stats.json";
    static bool statsSaveNeeded = false;
    static uint32_t lastStatsSave = 0;

    // ========== Raw Frames Logging ==========

    static const char* FRAMES_FILE = SD_MOUNT_POINT "/LoRa_Tracker/frames.log";
    static const char* FRAMES_OLD_FILE = SD_MOUNT_POINT "/LoRa_Tracker/frames.old";
    static const uint32_t MAX_FRAMES_SIZE = 100 * 1024;  // 100 KB

    void checkFramesLogRotation() {
        if (!sdAvailable) return;

        long fsize = vfs_filesize(FRAMES_FILE);
        if (fsize < 0) return;

        if ((uint32_t)fsize >= MAX_FRAMES_SIZE) {
            ESP_LOGI(TAG, "Rotating frames log (%ld bytes)...", fsize);
            if (vfs_exists(FRAMES_OLD_FILE)) {
                ::remove(FRAMES_OLD_FILE);
            }
            ::rename(FRAMES_FILE, FRAMES_OLD_FILE);
            ESP_LOGI(TAG, "Frames log rotated");
        }
    }

    // Memory cache for frames - fixed circular buffer
    static const int FRAMES_CACHE_SIZE = 50;
    static String framesCache[FRAMES_CACHE_SIZE];
    static int framesCacheHead = 0;
    static int framesCacheCount = 0;
    static std::vector<String> framesCacheOrdered;  // For returning ordered list

    // Dirty flags for conditional UI refresh
    static bool framesDirty = false;
    static bool statsDirty = false;

    bool logRawFrame(const String& frame, int rssi, float snr, bool isDirect) {
        // 1. Timestamp preparation
        char timestamp[64];
        if (year() > 2000) {
            snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02d %02d:%02d:%02d",
                year(), month(), day(), hour(), minute(), second());
        } else {
            snprintf(timestamp, sizeof(timestamp), "----/--/-- --:--:--");
        }

        // 2. Create line with [D]irect or [R]elayed marker
        String statusMarker = isDirect ? "[D]" : "[R]";
        String logLine = statusMarker + String(timestamp) + " GMT: " + frame;

        // 3. Store in RAM buffer (for display)
        framesCache[framesCacheHead] = logLine;
        framesCacheHead = (framesCacheHead + 1) % FRAMES_CACHE_SIZE;
        if (framesCacheCount < FRAMES_CACHE_SIZE) framesCacheCount++;
        framesDirty = true;  // Mark for UI refresh

        // 4. Write to SD card
        if (sdAvailable) {
            checkFramesLogRotation();
            FILE* f = fopen(FRAMES_FILE, "a");
            if (f) {
                fprintf(f, "%s\n", logLine.c_str());
                fclose(f);
            }
        }
        return true;
    }
    
const std::vector<String>& getLastFrames(int count) {
    framesCacheOrdered.clear();
    
    if (framesCacheCount == 0) return framesCacheOrdered;

    // Limit count to what we actually have
    int actualCount = (count > framesCacheCount) ? framesCacheCount : count;
    framesCacheOrdered.reserve(actualCount);

    // Get the most recent frames (backwards from head)
    for (int i = 0; i < actualCount; i++) {
        int idx = (framesCacheHead - 1 - i + FRAMES_CACHE_SIZE) % FRAMES_CACHE_SIZE;
        framesCacheOrdered.push_back(framesCache[idx]);
    }
    
    return framesCacheOrdered;
}

    // Load last 20 frames from SD card into RAM cache (called at boot)
    void loadFramesFromSD() {
        if (!sdAvailable) {
            ESP_LOGW(TAG, "No SD card, frames not loaded");
            return;
        }

        FILE* f = fopen(FRAMES_FILE, "r");
        if (!f) {
            ESP_LOGW(TAG, "No frames file, starting fresh");
            return;
        }

        std::vector<String> allLines;
        allLines.reserve(100);
        char lineBuf[512];
        while (fgets(lineBuf, sizeof(lineBuf), f)) {
            // Strip trailing newline
            size_t len = strlen(lineBuf);
            if (len > 0 && lineBuf[len - 1] == '\n') lineBuf[len - 1] = '\0';
            if (strlen(lineBuf) > 0) {
                allLines.push_back(String(lineBuf));
            }
        }
        fclose(f);

        int totalLines = allLines.size();
        int startIdx = (totalLines > 20) ? (totalLines - 20) : 0;
        int loadCount = totalLines - startIdx;

        for (int i = 0; i < loadCount; i++) {
            framesCache[framesCacheHead] = allLines[startIdx + i];
            framesCacheHead = (framesCacheHead + 1) % FRAMES_CACHE_SIZE;
            if (framesCacheCount < FRAMES_CACHE_SIZE) framesCacheCount++;
        }

        ESP_LOGI(TAG, "Loaded %d frames from SD", loadCount);
        framesDirty = true;
    }

    // ========== Link Statistics ==========

    // Initialize with inverted min/max so first value always updates them
    static LinkStats linkStats = {0, 0, 0, 0, -200, 0, 50.0f, -50.0f, 0.0f};
    static std::vector<DigiStats> digiStats;
    static std::vector<StationStats> stationStats;

    // History buffers for charts - fixed size circular buffer
    static int rssiHistory[HISTORY_SIZE];
    static float snrHistory[HISTORY_SIZE];
    static int historyCount = 0;  // Number of valid entries
    static int historyHead = 0;   // Next write position

    // Ordered vectors for returning to UI (rebuilt on request)
    static std::vector<int> rssiHistoryOrdered;
    static std::vector<float> snrHistoryOrdered;

    void resetStats() {
        linkStats = {0, 0, 0, 0, -200, 0, 50.0f, -50.0f, 0.0f};
        digiStats.clear();
        stationStats.clear();
        historyCount = 0;
        historyHead = 0;
        rssiHistoryOrdered.clear();
        snrHistoryOrdered.clear();
        ESP_LOGI(TAG, "Stats reset");
    }

    void updateRxStats(int rssi, float snr) {
        linkStats.rxCount++;
        linkStats.rssiTotal += rssi;
        if (rssi < linkStats.rssiMin) linkStats.rssiMin = rssi;
        if (rssi > linkStats.rssiMax) linkStats.rssiMax = rssi;
        linkStats.snrTotal += snr;
        if (snr < linkStats.snrMin) linkStats.snrMin = snr;
        if (snr > linkStats.snrMax) linkStats.snrMax = snr;

        statsSaveNeeded = true;

        // Add to circular buffer (no memory allocation)
        rssiHistory[historyHead] = rssi;
        snrHistory[historyHead] = snr;
        historyHead = (historyHead + 1) % HISTORY_SIZE;
        if (historyCount < HISTORY_SIZE) historyCount++;
    }

    const std::vector<int>& getRssiHistory() {
        // Rebuild ordered vector from circular buffer
        rssiHistoryOrdered.clear();
        rssiHistoryOrdered.reserve(historyCount);
        int start = (historyCount < HISTORY_SIZE) ? 0 : historyHead;
        for (int i = 0; i < historyCount; i++) {
            rssiHistoryOrdered.push_back(rssiHistory[(start + i) % HISTORY_SIZE]);
        }
        return rssiHistoryOrdered;
    }

    const std::vector<float>& getSnrHistory() {
        // Rebuild ordered vector from circular buffer
        snrHistoryOrdered.clear();
        snrHistoryOrdered.reserve(historyCount);
        int start = (historyCount < HISTORY_SIZE) ? 0 : historyHead;
        for (int i = 0; i < historyCount; i++) {
            snrHistoryOrdered.push_back(snrHistory[(start + i) % HISTORY_SIZE]);
        }
        return snrHistoryOrdered;
    }

    void updateTxStats() {
        linkStats.txCount++;
    }

    void updateAckStats() {
        linkStats.ackCount++;
    }

    void updateDigiStats(const String& path) {
        // Parse path to extract digipeaters
        // Format: DEST,DIGI1*,DIGI2,qAO,IGATE or similar
        // The * indicates which digi actually relayed

        int start = 0;
        int comma = path.indexOf(',');

        while (comma != -1) {
            String segment = path.substring(start, comma);

            // Check if this digi has relayed (marked with *)
            bool relayed = segment.endsWith("*");
            if (relayed) {
                segment = segment.substring(0, segment.length() - 1);
            }

            // Skip q-constructs (qAO, qAR, etc.), WIDE/TRACE, and known generic/service aliases
            if (!segment.startsWith("q") &&
                !segment.startsWith("WIDE") &&
                !segment.startsWith("TRACE") &&
                !segment.equalsIgnoreCase("APLRT1") && // APRS-IS Relay Tracker 1
                !segment.equalsIgnoreCase("APLRG1") && // APRS-IS Relay Gateway 1
                !segment.equalsIgnoreCase("APLRFD") && // APRS-IS Relay Forwarder
                !segment.equalsIgnoreCase("APLRFA") && // APRS-IS Relay Forwarder (alternate)
                !segment.equalsIgnoreCase("APFD11") && // Generic Forwarder
                !segment.equalsIgnoreCase("APWW11") && // Weather station IGate
                segment.length() > 0) {

                // Find or create digi entry
                bool found = false;
                for (auto& d : digiStats) {
                    if (d.callsign.equalsIgnoreCase(segment)) {
                        d.count++;
                        found = true;
                        break;
                    }
                }
                if (!found && digiStats.size() < 50) {  // Limit to 50 digis
                    DigiStats newDigi;
                    newDigi.callsign = segment;
                    newDigi.count = 1;
                    digiStats.push_back(newDigi);
                }
            }

            start = comma + 1;
            comma = path.indexOf(',', start);
        }

        // Handle last segment
        String lastSeg = path.substring(start);
        if (lastSeg.endsWith("*")) {
            lastSeg = lastSeg.substring(0, lastSeg.length() - 1);
        }
        if (!lastSeg.startsWith("q") &&
            !lastSeg.startsWith("WIDE") &&
            !lastSeg.startsWith("TRACE") &&
            !lastSeg.equalsIgnoreCase("APLRT1") && // APRS-IS Relay Tracker 1
            !lastSeg.equalsIgnoreCase("APLRG1") && // APRS-IS Relay Gateway 1
            !lastSeg.equalsIgnoreCase("APLRFD") && // APRS-IS Relay Forwarder
            !lastSeg.equalsIgnoreCase("APLRFA") && // APRS-IS Relay Forwarder (alternate)
            !lastSeg.equalsIgnoreCase("APFD11") && // Generic Forwarder
            !lastSeg.equalsIgnoreCase("APWW11") && // Weather station IGate
            lastSeg.length() > 0) {
            bool found = false;
            for (auto& d : digiStats) {
                if (d.callsign.equalsIgnoreCase(lastSeg)) {
                    d.count++;
                    found = true;
                    break;
                }
            }
            if (!found && digiStats.size() < 50) {
                DigiStats newDigi;
                newDigi.callsign = lastSeg;
                newDigi.count = 1;
                digiStats.push_back(newDigi);
            }
        }
    }

    LinkStats getStats() {
        return linkStats;
    }

    float getAvgRssi() {
        return linkStats.rxCount > 0 ? (float)linkStats.rssiTotal / linkStats.rxCount : 0.0f;
    }

    float getAvgSnr() {
        return linkStats.rxCount > 0 ? linkStats.snrTotal / linkStats.rxCount : 0.0f;
    }

    const std::vector<DigiStats>& getDigiStats() {
        return digiStats;
    }

    // ========== Dashboard Last RX (RAM only, max 4) ==========

    static const int DASHBOARD_RX_SIZE = 4;
    static std::vector<DashboardRxEntry> dashboardRx;

    static void updateDashboardRx(const String& callsign, int rssi, float snr) {
        // Add new entry at front
        DashboardRxEntry entry;
        entry.callsign = callsign;
        entry.rssi = rssi;
        entry.snr = snr;
        entry.timestamp = now(); // Unix timestamp for fixed time display

        dashboardRx.insert(dashboardRx.begin(), entry);

        // Keep only last 4
        if (dashboardRx.size() > DASHBOARD_RX_SIZE) {
            dashboardRx.resize(DASHBOARD_RX_SIZE);
        }
    }

    const std::vector<DashboardRxEntry>& getDashboardLastRx() {
        return dashboardRx;
    }

    // ========== Per-station statistics ==========

    // Per-station statistics
    void updateStationStats(const String& callsign, int rssi, float snr, bool isDirect) {
        // Update dashboard Last RX (RAM only, max 4)
        updateDashboardRx(callsign, rssi, snr);

        // Check if station already exists
        for (auto& s : stationStats) {
            if (s.callsign.equalsIgnoreCase(callsign)) {
                s.count++;
                s.lastRssi = rssi;
                s.lastSnr = snr;
                s.rssiTotal += rssi;
                s.snrTotal += snr;
                s.lastHeard = now(); // Unix timestamp from GPS/RTC
                s.lastIsDirect = isDirect;
                statsDirty = true;
                statsSaveNeeded = true;
                return;
            }
        }

        // New station
        StationStats newStation;
        newStation.callsign = callsign;
        newStation.count = 1;
        newStation.lastRssi = rssi;
        newStation.lastSnr = snr;
        newStation.rssiTotal = rssi;
        newStation.snrTotal = snr;
        newStation.lastHeard = now();
        newStation.lastIsDirect = isDirect;

        if (stationStats.size() >= 20) {
            // Evict the oldest station
            auto oldest = std::min_element(stationStats.begin(), stationStats.end(),
                [](const StationStats& a, const StationStats& b) {
                    return a.lastHeard < b.lastHeard;
                });
            *oldest = newStation;
        } else {
            stationStats.push_back(newStation);
        }
        statsDirty = true;
        statsSaveNeeded = true;
    }

    const std::vector<StationStats>& getStationStats() {
        return stationStats;
    }

    // ========== Stats Persistence Implementation ==========

    void loadStats() {
        if (!sdAvailable) {
            ESP_LOGW(TAG, "No SD card, stats not loaded");
            return;
        }

        long fsize = vfs_filesize(STATS_FILE);
        if (fsize <= 0) {
            ESP_LOGW(TAG, "No stats file, starting fresh");
            return;
        }

        FILE* f = fopen(STATS_FILE, "r");
        if (!f) return;

        char* buf = (char*)malloc(fsize + 1);
        if (!buf) { fclose(f); return; }
        size_t rd = fread(buf, 1, fsize, f);
        fclose(f);
        buf[rd] = '\0';

        DynamicJsonDocument doc(4096);
        DeserializationError error = deserializeJson(doc, buf);
        free(buf);

        if (error) {
            ESP_LOGE(TAG, "Stats JSON parse error: %s", error.c_str());
            return;
        }

        // Restore LinkStats
        if (doc.containsKey("link")) {
            JsonObject link = doc["link"];
            linkStats.rxCount   = link["rx"]       | (uint32_t)0;
            linkStats.txCount   = link["tx"]       | (uint32_t)0;
            linkStats.ackCount  = link["ack"]      | (uint32_t)0;
            linkStats.rssiMin   = link["rssiMin"]  | 0;
            linkStats.rssiMax   = link["rssiMax"]  | -200;
            linkStats.rssiTotal = link["rssiTotal"]| (int32_t)0;
            linkStats.snrMin    = link["snrMin"]   | 50.0f;
            linkStats.snrMax    = link["snrMax"]   | -50.0f;
            linkStats.snrTotal  = link["snrTotal"] | 0.0f;
        }

        // Restore StationStats
        if (doc.containsKey("stations")) {
            JsonArray stations = doc["stations"];
            stationStats.clear();
            for (JsonObject obj : stations) {
                if (stationStats.size() >= 20) break;
                StationStats s;
                s.callsign    = obj["call"].as<String>();
                s.count       = obj["cnt"]    | (uint32_t)0;
                s.lastRssi    = obj["rssi"]   | 0;
                s.lastSnr     = obj["snr"]    | 0.0f;
                s.rssiTotal   = obj["rssiTotal"] | (int32_t)0;
                s.snrTotal    = obj["snrTotal"]  | 0.0f;
                s.lastHeard   = obj["heard"]  | (uint32_t)0;
                s.lastIsDirect= obj["direct"] | false;

                // Migration: initialize totals from last values if missing (old format)
                if (s.rssiTotal == 0 && s.snrTotal == 0.0f && s.count > 0) {
                    s.rssiTotal = s.lastRssi * s.count;
                    s.snrTotal = s.lastSnr * s.count;
                }

                if (s.callsign.length() > 0) {
                    stationStats.push_back(s);
                }
            }
        }

        ESP_LOGI(TAG, "Loaded %d station stats", stationStats.size());
    }

    bool saveStats() {
        if (!sdAvailable) {
            return false;
        }

        DynamicJsonDocument doc(4096);

        // Serialize LinkStats
        JsonObject link = doc.createNestedObject("link");
        link["rx"]        = linkStats.rxCount;
        link["tx"]        = linkStats.txCount;
        link["ack"]       = linkStats.ackCount;
        link["rssiMin"]   = linkStats.rssiMin;
        link["rssiMax"]   = linkStats.rssiMax;
        link["rssiTotal"] = linkStats.rssiTotal;
        link["snrMin"]    = linkStats.snrMin;
        link["snrMax"]    = linkStats.snrMax;
        link["snrTotal"]  = linkStats.snrTotal;

        // Serialize StationStats
        JsonArray stations = doc.createNestedArray("stations");
        for (const StationStats& s : stationStats) {
            JsonObject obj = stations.createNestedObject();
            obj["call"]   = s.callsign;
            obj["cnt"]    = s.count;
            obj["rssi"]   = s.lastRssi;
            obj["snr"]    = s.lastSnr;
            obj["rssiTotal"] = s.rssiTotal;
            obj["snrTotal"]  = s.snrTotal;
            obj["heard"]  = s.lastHeard;
            obj["direct"] = s.lastIsDirect;
        }

        FILE* f = fopen(STATS_FILE, "w");
        if (!f) {
            ESP_LOGE(TAG, "Failed to open stats file for writing");
            return false;
        }

        char buf[4096];
        size_t len = serializeJson(doc, buf, sizeof(buf));
        fwrite(buf, 1, len, f);
        fclose(f);

        ESP_LOGI(TAG, "Saved stats (%d stations)", stationStats.size());
        return true;
    }

    void checkStatsSave() {
        if (statsSaveNeeded && (millis() - lastStatsSave > 300000)) {
            saveStats();
            statsSaveNeeded = false;
            lastStatsSave = millis();
        }
    }

    // Dirty flag accessors for conditional UI refresh
    bool isFramesDirty() {
        return framesDirty;
    }

    void clearFramesDirty() {
        framesDirty = false;
    }

    bool isStatsDirty() {
        return statsDirty;
    }

    void clearStatsDirty() {
        statsDirty = false;
    }

}
