// map_gps_filter.h
#ifndef MAP_GPS_FILTER_H
#define MAP_GPS_FILTER_H

#include <cstdint>
#include "TinyGPS++.h"
#include "station_utils.h" // For TRACE_MAX_POINTS and TracePoint

// TracePoint structure is already defined in station_utils.h
// Use the existing definition

class MapGPSFilter {
public:
    // Default constructor - initializes all state
    MapGPSFilter();
    
    // Update filtered position from GPS data (faithful to original logic)
    void updateFilteredOwnPosition(TinyGPSPlus& gps);
    
    // Add current position to trace history (faithful to original logic)
    void addOwnTracePoint();
    
    // Get UI position (filtered if available, otherwise raw) - faithful to original
    bool getUiPosition(float* lat, float* lon) const;
    
    // Getters for raw GPS data
    float getIconGpsLat() const { return iconGpsLat; }
    float getIconGpsLon() const { return iconGpsLon; }
    bool isIconGpsValid() const { return iconGpsValid; }
    
    // Getters for filtered position
    float getFilteredOwnLat() const { return filteredOwnLat; }
    float getFilteredOwnLon() const { return filteredOwnLon; }
    bool isFilteredValid() const { return filteredOwnValid; }
    
    // Getters for trace history with proper circular buffer handling
    const TracePoint* getOwnTrace() const { return ownTrace; }
    int getOwnTraceCount() const { return ownTraceCount; }
    int getOwnTraceHead() const { return ownTraceHead; }
    
    // Helper to get trace point at index with circular buffer wrapping
    const TracePoint& getOwnTracePoint(int index) const {
        // Handle circular buffer: index is logical position (0 = oldest)
        int bufferIndex = (ownTraceHead - ownTraceCount + index + TRACE_MAX_POINTS) % TRACE_MAX_POINTS;
        return ownTrace[bufferIndex];
    }
    
    // Reset all state
    void reset();
    
private:
    // Configuration constants (from original code)
    static constexpr int MIN_SATS_ICON = 3;      // Minimum satellites for icon display
    static constexpr int MIN_SATS_FILTERED = 6;  // Minimum satellites for filtered position
    static constexpr float MIN_DISTANCE_M = 15.0f; // Minimum movement threshold (meters)
    static constexpr float HDOP_FACTOR = 5.0f;   // HDOP multiplier for threshold
    static constexpr float TRACE_DISTANCE_THRESHOLD = 0.0001f; // ~11 meters in degrees
    
    // Raw GPS position for UI (≥3 satellites)
    float iconGpsLat = 0.0f;
    float iconGpsLon = 0.0f;
    bool iconGpsValid = false;
    
    // Filtered (centroid-based) position for trace/recentering (≥6 satellites)
    float filteredOwnLat = 0.0f;
    float filteredOwnLon = 0.0f;
    bool filteredOwnValid = false;
    
    // Trace history (circular buffer) - fixed size from station_utils.h
    TracePoint ownTrace[TRACE_MAX_POINTS];
    int ownTraceCount = 0;
    int ownTraceHead = 0;
    
    // Centroid tracking for movement detection (internal state)
    float iconCentroidLat = 0.0f;
    float iconCentroidLon = 0.0f;
    uint32_t iconCentroidCount = 0;
};
#endif // MAP_GPS_FILTER_H