/* trace_sd.cpp — Binary trace persistence on SD card */

#ifdef USE_LVGL_UI

#include "trace_sd.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <NMEAGPS.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include "storage_utils.h"

static const char* TAG = "TraceSD";

extern SemaphoreHandle_t spiMutex;
extern gps_fix gpsFix;

static char currentFilePath[64] = "";
static bool initialized = false;

static void updateFilePath() {
    if (gpsFix.valid.date) {
        snprintf(currentFilePath, sizeof(currentFilePath),
                 SD_MOUNT_POINT "/LoRa_Tracker/trace/trace_%04d%02d%02d.bin",
                 2000 + gpsFix.dateTime.year, gpsFix.dateTime.month, gpsFix.dateTime.date);
    } else {
        strncpy(currentFilePath, SD_MOUNT_POINT "/LoRa_Tracker/trace/trace_nodate.bin",
                sizeof(currentFilePath));
    }
}

namespace TraceSD {

    void clearPreviousTrace() {
        if (spiMutex == NULL || xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            return;
        }
        struct stat st;
        if (stat(SD_MOUNT_POINT "/LoRa_Tracker/trace", &st) != 0) {
            mkdir(SD_MOUNT_POINT "/LoRa_Tracker/trace", 0775);
            xSemaphoreGiveRecursive(spiMutex);
            return;
        }
        DIR* dir = opendir(SD_MOUNT_POINT "/LoRa_Tracker/trace");
        if (dir) {
            struct dirent* entry;
            while ((entry = readdir(dir)) != nullptr) {
                if (entry->d_name[0] == '.') continue;
                char path[80];
                snprintf(path, sizeof(path), SD_MOUNT_POINT "/LoRa_Tracker/trace/%s", entry->d_name);
                remove(path);
                ESP_LOGI(TAG, "Cleared old trace: %s", path);
            }
            closedir(dir);
        }
        xSemaphoreGiveRecursive(spiMutex);
    }

    void init() {
        if (!initialized) {
            if (spiMutex == NULL || xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
                return;
            }
            struct stat st;
            if (stat(SD_MOUNT_POINT "/LoRa_Tracker/trace", &st) != 0) {
                mkdir(SD_MOUNT_POINT "/LoRa_Tracker/trace", 0775);
            }
            xSemaphoreGiveRecursive(spiMutex);
        }

        updateFilePath();
        initialized = true;
        ESP_LOGI(TAG, "Trace SD initialized: %s", currentFilePath);
    }

    void appendPoint(float lat, float lon, uint32_t time_ms) {
        if (!initialized) return;

        updateFilePath();

        if (spiMutex == NULL || xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
            return;
        }

        FILE* f = fopen(currentFilePath, "ab");
        if (f) {
            TraceRecord rec = { lat, lon, time_ms };
            fwrite(&rec, sizeof(rec), 1, f);
            fclose(f);
        }
        xSemaphoreGiveRecursive(spiMutex);
    }

    int readViewport(float minLat, float maxLat, float minLon, float maxLon,
                     TraceRecord* outBuf, int maxPoints) {
        if (!initialized || maxPoints <= 0) return 0;

        updateFilePath();

        if (spiMutex == NULL || xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
            return 0;
        }

        FILE* f = fopen(currentFilePath, "rb");
        if (!f) {
            xSemaphoreGiveRecursive(spiMutex);
            return 0;
        }

        int count = 0;
        TraceRecord rec;
        while (count < maxPoints && fread(&rec, sizeof(rec), 1, f) == 1) {
            if (rec.lat >= minLat && rec.lat <= maxLat &&
                rec.lon >= minLon && rec.lon <= maxLon) {
                outBuf[count++] = rec;
            }
        }

        fclose(f);
        xSemaphoreGiveRecursive(spiMutex);
        return count;
    }

    int getTodayPointCount() {
        if (!initialized) return 0;

        updateFilePath();

        if (spiMutex == NULL || xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
            return 0;
        }

        struct stat st;
        int count = 0;
        if (stat(currentFilePath, &st) == 0) {
            count = (int)(st.st_size / sizeof(TraceRecord));
        }
        xSemaphoreGiveRecursive(spiMutex);
        return count;
    }

}  // namespace TraceSD

#endif // USE_LVGL_UI
