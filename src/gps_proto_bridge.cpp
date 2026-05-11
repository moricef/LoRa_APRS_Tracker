/* GPS Proto Bridge — converts C3 proto gps_fix_t to NeoGPS gps_fix.
 *
 * Registered as UartLink GPS_FIX handler. Called from UartLink::loop().
 * Writes directly to the NeoGPS global gpsFix — same structure that the
 * NMEA path populates, making the rest of the app unaware of the source.
 */

#ifdef USE_LVGL_UI

#if defined(WAVESHARE_S3_TOUCH_LCD_7)

#include "uart_link.h"
#include "gps_utils.h"
#include "board_pinout.h"
#include <Arduino.h>
#include <time.h>
#include <esp_log.h>

static const char* TAG = "GpsBridge";

extern gps_fix gpsFix;

static void onGpsFix(const gps_fix_t* pf)
{
    // Clear all validity flags (C++ value-init, safe for bitfields)
    gpsFix.valid = gps_fix::valid_t{};

    // --- location (1e-7 deg → NeoGPS Location_t, same format) ---
    gpsFix.location.lat(pf->lat);
    gpsFix.location.lon(pf->lon);

    // --- altitude (mm → meters + cm for whole_frac) ---
    int32_t alt_mm = pf->alt;
    gpsFix.alt.whole = (int16_t)(alt_mm / 1000);
    gpsFix.alt.frac  = (int16_t)((alt_mm % 1000) / 10);

    // --- speed (cm/s → 0.001 knots for NeoGPS whole_frac) ---
    // 1 knot = 0.514444 m/s = 51.4444 cm/s
    float knots = (float)pf->speed / 51.44444f;
    int32_t knots_1000 = (int32_t)(knots * 1000.0f + 0.5f);
    gpsFix.spd.whole = (int16_t)(knots_1000 / 1000);
    gpsFix.spd.frac  = (int16_t)(knots_1000 % 1000);

    // --- heading (0.01 deg → whole_frac: whole=deg, frac=0.01 deg) ---
    gpsFix.hdg.whole = (int16_t)(pf->heading / 100);
    gpsFix.hdg.frac  = (int16_t)(pf->heading % 100);

    // --- HDOP (×100 → ×1000 for NeoGPS) ---
    gpsFix.hdop = (uint16_t)(pf->hdop) * 10;

    // --- satellites ---
    gpsFix.satellites = pf->sats;

    // --- date/time from epoch ---
    time_t epoch = (time_t)pf->timestamp;
    struct tm* t = gmtime(&epoch);
    if (t) {
        gpsFix.dateTime.hours   = t->tm_hour;
        gpsFix.dateTime.minutes = t->tm_min;
        gpsFix.dateTime.seconds = t->tm_sec;
        gpsFix.dateTime.date    = t->tm_mday;
        gpsFix.dateTime.month   = t->tm_mon + 1;
        gpsFix.dateTime.year    = t->tm_year - 100;   // Y2K offset
    }

    // --- validity ---
    uint8_t ft = pf->fix_type;                         // 0=none, 2=2D, 3=3D
    bool hasFix = (pf->flags & 0x01) && (ft >= 2);

    gpsFix.valid.status   = hasFix;
    gpsFix.valid.location = hasFix && (ft >= 2);
    gpsFix.valid.altitude = hasFix && (ft == 3);
    gpsFix.valid.speed    = (pf->flags & 0x04) != 0;
    gpsFix.valid.heading  = hasFix;
    gpsFix.valid.hdop     = hasFix;
    gpsFix.valid.satellites = pf->sats > 0;
    gpsFix.valid.time     = hasFix;
    gpsFix.valid.date     = hasFix;

    GPS_Utils::setNewFixAvailable();
}

void gpsProtoBridgeInit()
{
    UartLink::setGpsFixHandler(onGpsFix);
    GPS_Utils::enableProtoMode();
    ESP_LOGI(TAG, "GPS proto bridge registered");
}

#endif // LORA_ON_C3
#endif // USE_LVGL_UI
