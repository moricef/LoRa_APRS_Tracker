// map_gps_filter.cpp
#include "map_gps_filter.h"
#include <cmath>     // for fabs, fmax, cosf
#include <esp_log.h>

// Gestion de millis() pour tests unitaires
#ifdef UNIT_TEST
#include "mock_arduino.h"
#define MILLIS MockArduino::millis
#else
#include <Arduino.h> // for millis() en production
#define MILLIS millis
#endif

static const char* TAG = "GPSFilter";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MapGPSFilter::MapGPSFilter() {
    reset();
}

void MapGPSFilter::updateFilteredOwnPosition(TinyGPSPlus& gps) {
    // Original logic preserved exactly
    
    if (!gps.location.isValid()) {
        ESP_LOGV(TAG, "GPS location invalid");
        return;
    }
    
    float lat = gps.location.lat();
    float lon = gps.location.lng();
    int sats = gps.satellites.value();
    
    // Level 1: icon display (≥3 sats = 2D fix minimum)
    if (sats >= MIN_SATS_ICON) {
        iconGpsLat = lat;
        iconGpsLon = lon;
        iconGpsValid = true;
        ESP_LOGV(TAG, "Icon GPS updated: %.6f, %.6f (%d sats)", lat, lon, sats);
    }
    
    // Level 2: filtered position for trace + recentrage (≥6 sats)
    if (sats < MIN_SATS_FILTERED) {
        ESP_LOGV(TAG, "Insufficient satellites for filtered position: %d < %d", sats, MIN_SATS_FILTERED);
        return;
    }
    
    if (!filteredOwnValid) {
        // First valid filtered position
        filteredOwnLat = lat;
        filteredOwnLon = lon;
        filteredOwnValid = true;
        iconCentroidLat = lat;
        iconCentroidLon = lon;
        iconCentroidCount = 1;
        ESP_LOGD(TAG, "First filtered position: %.6f, %.6f", lat, lon);
        return;
    }
    
    // Update running centroid with every GPS reading
    float alpha = (iconCentroidCount < 10) ? 1.0f / (iconCentroidCount + 1) : 0.1f;
    iconCentroidLat += alpha * (lat - iconCentroidLat);
    iconCentroidLon += alpha * (lon - iconCentroidLon);
    iconCentroidCount++;
    
    // Threshold: 15m min, +5m per HDOP unit (original values)
    float hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 2.0f;
    float thresholdM = fmax(MIN_DISTANCE_M, hdop * HDOP_FACTOR);
    float thresholdLat = thresholdM / 111320.0f;
    float thresholdLon = thresholdM / (111320.0f * cosf(lat * M_PI / 180.0f));
    
    // Compare to centroid — only update if real movement
    if (fabs(lat - iconCentroidLat) > thresholdLat || fabs(lon - iconCentroidLon) > thresholdLon) {
        filteredOwnLat = lat;
        filteredOwnLon = lon;
        iconCentroidLat = lat;
        iconCentroidLon = lon;
        iconCentroidCount = 1;
        ESP_LOGD(TAG, "Filtered position updated: %.6f, %.6f (threshold: %.1fm)", 
                lat, lon, thresholdM);
    }
}

void MapGPSFilter::addOwnTracePoint() {
    // Original logic preserved exactly
    
    if (!filteredOwnValid) {
        ESP_LOGV(TAG, "No valid filtered position for trace");
        return; // No valid smoothed position yet
    }
    
    float lat = filteredOwnLat;
    float lon = filteredOwnLon;
    
    // Ensure we only add a new point if we moved enough from the last trace point.
    // This prevents the buffer from filling up with identical points during standing updates.
    if (ownTraceCount > 0) {
        int lastIdx = (ownTraceHead - 1 + TRACE_MAX_POINTS) % TRACE_MAX_POINTS;
        float lastLat = ownTrace[lastIdx].lat;
        float lastLon = ownTrace[lastIdx].lon;
        
        // Threshold: 0.0001 degrees is roughly 11 meters (original value)
        if (fabs(lat - lastLat) < TRACE_DISTANCE_THRESHOLD && 
            fabs(lon - lastLon) < TRACE_DISTANCE_THRESHOLD) {
            ESP_LOGV(TAG, "Trace point skipped: insufficient movement");
            return; // Hasn't moved enough from the last recorded trace point
        }
    }
    
    // Add point to circular buffer
    ownTrace[ownTraceHead].lat = lat;
    ownTrace[ownTraceHead].lon = lon;
    ownTrace[ownTraceHead].time = MILLIS();
    
    ownTraceHead = (ownTraceHead + 1) % TRACE_MAX_POINTS;
    if (ownTraceCount < TRACE_MAX_POINTS) {
        ownTraceCount++;
    }
    
    ESP_LOGD(TAG, "Own trace point added: %.6f, %.6f (count=%d)", lat, lon, ownTraceCount);
}

bool MapGPSFilter::getUiPosition(float* lat, float* lon) const {
    // Original logic preserved exactly
    if (filteredOwnValid) {
        *lat = filteredOwnLat;
        *lon = filteredOwnLon;
        return true;
    } else if (iconGpsValid) {
        *lat = iconGpsLat;
        *lon = iconGpsLon;
        return true;
    }
    return false;
}

void MapGPSFilter::reset() {
    iconGpsLat = 0.0f;
    iconGpsLon = 0.0f;
    iconGpsValid = false;
    
    filteredOwnLat = 0.0f;
    filteredOwnLon = 0.0f;
    filteredOwnValid = false;
    
    ownTraceCount = 0;
    ownTraceHead = 0;
    
    iconCentroidLat = 0.0f;
    iconCentroidLon = 0.0f;
    iconCentroidCount = 0;
    
    ESP_LOGD(TAG, "GPS filter reset");
}