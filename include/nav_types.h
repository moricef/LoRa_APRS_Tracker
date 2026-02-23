#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include <cstdint>
#include <cstdlib>
#include "esp_heap_caps.h"

// Allocator that places vectors in PSRAM to preserve scarce DRAM
// for WiFi, BLE, LoRa, LVGL, and OS stacks.
// Uses MALLOC_CAP_SPIRAM with fallback to default heap.
template <typename T>
struct PSRAMAllocator {
    using value_type = T;

    PSRAMAllocator() noexcept = default;
    template <typename U>
    PSRAMAllocator(const PSRAMAllocator<U>&) noexcept {}

    T* allocate(std::size_t n) {
        void* p = heap_caps_malloc(n * sizeof(T), MALLOC_CAP_SPIRAM);
        if (!p) p = malloc(n * sizeof(T)); // fallback to default heap
        return static_cast<T*>(p);
    }
    void deallocate(T* p, std::size_t) noexcept {
        heap_caps_free(p);
    }

    template <typename U>
    bool operator==(const PSRAMAllocator<U>&) const noexcept { return true; }
    template <typename U>
    bool operator!=(const PSRAMAllocator<U>&) const noexcept { return false; }
};

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

// Feature Header (13 bytes packed)
// Followed by: payload of payloadSize bytes
// Lines/Polygons: coords encoded as Delta+ZigZag+VarInt
// Polygons: ring data (uint16 ringCount + ringCount × uint16 ringEnds) at end of payload
// Points: 1 coord encoded as ZigZag+VarInt (delta from 0)
// Text (type 4): int16 px, int16 py, uint8 textLen, text bytes (NOT VarInt)
struct NavFeatureHeader {
    uint8_t geomType;       // 1=Point, 2=Line, 3=Polygon, 4=Text
    uint16_t colorRgb565;
    uint8_t zoomPriority;   // High nibble = minZoom, low nibble = priority
    uint8_t widthPixels;
    uint8_t bbox[4];        // x1, y1, x2, y2 (tile-relative, /16)
    uint16_t coordCount;
    uint16_t payloadSize;   // Total payload size in bytes (replaces padding)
} __attribute__((packed));

// Edge structure for Active Edge List (AEL) algorithm
struct Edge {
    int32_t yMax;
    int32_t xVal;           // 16.16 fixed-point
    int32_t slope;          // 16.16 fixed-point
    int nextInBucket;
    int nextActive;
};

// NPK1 pack file index entry (12 bytes)
// Pack format: "NPK1" magic + uint32 tile_count + tile_count × NpkIndexEntry + NAV1 blobs
// Index sorted by x then y for binary search
struct NpkIndexEntry {
    uint32_t x;
    uint32_t y;
    uint32_t offset;   // from start of file
    uint32_t size;     // NAV1 blob size
} __attribute__((packed));

} // namespace UIMapManager
#endif
