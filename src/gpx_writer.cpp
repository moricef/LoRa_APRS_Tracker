#ifdef USE_LVGL_UI

#include <esp_log.h>
#include "gpx_writer.h"
#include "storage_utils.h"
#include <NMEAGPS.h>
#include <freertos/semphr.h>
#include <sys/stat.h>

static const char *TAG = "GPX";

extern gps_fix gpsFix;
extern SemaphoreHandle_t spiMutex;

namespace GPXWriter {

    static bool recording = false;
    static char currentFilePath[64] = "";

    bool isRecording() {
        return recording;
    }

    bool startRecording() {
        if (recording) return true;

        // Build filename from GPS date/time (UTC)
        char filename[64];
        if (gpsFix.valid.date && gpsFix.valid.time) {
            snprintf(filename, sizeof(filename),
                     "/LoRaTracker/gpx/track_%04d-%02d-%02d_%02d%02d.gpx",
                     2000 + gpsFix.dateTime.year, gpsFix.dateTime.month, gpsFix.dateTime.date,
                     gpsFix.dateTime.hours, gpsFix.dateTime.minutes);
        } else {
            snprintf(filename, sizeof(filename),
                     "/LoRaTracker/gpx/track_%u.gpx", (unsigned)(millis() / 1000));
        }

        if (spiMutex == NULL || xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to acquire SPI mutex");
            return false;
        }

        // Ensure directory exists
        String gpxDir = STORAGE_Utils::sdPath("/LoRaTracker/gpx");
        struct stat st;
        if (stat(gpxDir.c_str(), &st) != 0) {
            ::mkdir(gpxDir.c_str(), 0775);
        }

        String fullPath = STORAGE_Utils::sdPath(filename);
        FILE* file = fopen(fullPath.c_str(), "w");
        if (!file) {
            xSemaphoreGiveRecursive(spiMutex);
            ESP_LOGE(TAG, "Failed to create file: %s", filename);
            return false;
        }

        fprintf(file, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        fprintf(file, "<gpx version=\"1.1\" creator=\"LoRa_APRS_Tracker\"\n");
        fprintf(file, "  xmlns=\"http://www.topografix.com/GPX/1/1\">\n");
        fprintf(file, "  <trk>\n");
        fprintf(file, "    <name>LoRa APRS Track</name>\n");
        fprintf(file, "    <trkseg>\n");
        fclose(file);
        xSemaphoreGiveRecursive(spiMutex);

        strncpy(currentFilePath, fullPath.c_str(), sizeof(currentFilePath));
        recording = true;
        ESP_LOGI(TAG, "Recording started: %s", filename);
        return true;
    }

    void stopRecording() {
        if (!recording) return;

        if (spiMutex != NULL && xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            FILE* file = fopen(currentFilePath, "a");
            if (file) {
                fprintf(file, "    </trkseg>\n");
                fprintf(file, "  </trk>\n");
                fprintf(file, "</gpx>\n");
                fclose(file);
            }
            xSemaphoreGiveRecursive(spiMutex);
        }

        ESP_LOGI(TAG, "Recording stopped: %s", currentFilePath);
        recording = false;
        currentFilePath[0] = '\0';
    }

    void addPoint(float lat, float lon, float alt, float hdop, float speed) {
        if (!recording) return;

        // Build timestamp from GPS
        char timestamp[32] = "";
        if (gpsFix.valid.date && gpsFix.valid.time) {
            snprintf(timestamp, sizeof(timestamp),
                     "%04d-%02d-%02dT%02d:%02d:%02dZ",
                     2000 + gpsFix.dateTime.year, gpsFix.dateTime.month, gpsFix.dateTime.date,
                     gpsFix.dateTime.hours, gpsFix.dateTime.minutes, gpsFix.dateTime.seconds);
        }

        if (spiMutex == NULL || xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
            return;
        }

        FILE* file = fopen(currentFilePath, "a");
        if (file) {
            fprintf(file, "      <trkpt lat=\"%.6f\" lon=\"%.6f\">\n", lat, lon);
            fprintf(file, "        <ele>%.1f</ele>\n", alt);
            if (timestamp[0])
                fprintf(file, "        <time>%s</time>\n", timestamp);
            fprintf(file, "        <hdop>%.1f</hdop>\n", hdop);
            fprintf(file, "        <speed>%.1f</speed>\n", speed);
            fprintf(file, "      </trkpt>\n");
            fclose(file);
        }
        xSemaphoreGiveRecursive(spiMutex);
    }

} // namespace GPXWriter

#endif // USE_LVGL_UI
