#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include <cstdint>

namespace UIMapManager {

// Global Tile Header (22 bytes)
struct NavTileHeader {
    char magic[4];          // 'NAV1'
    uint16_t featureCount;
    int32_t minLon;
    int32_t minLat;
    int32_t maxLon;
    int32_t maxLat;
} __attribute__((packed));

// Feature Header (12 bytes aligned)
struct NavFeatureHeader {
    uint8_t geomType;       // 1=Point, 2=Line, 3=Polygon
    uint16_t colorRgb565;
    uint8_t zoomPriority;
    uint8_t widthPixels;
    uint8_t bbox[4];        // x1, y1, x2, y2
    uint16_t coordCount;
    uint8_t padding;        // Alignment byte
} __attribute__((packed));

// Edge structure for Active Edge List (AEL) algorithm
struct Edge {
    int32_t yMax;
    int32_t xVal;           // 16.16 fixed-point
    int32_t slope;          // 16.16 fixed-point
    int nextInBucket;
    int nextActive;
};

} // namespace UIMapManager
#endif
