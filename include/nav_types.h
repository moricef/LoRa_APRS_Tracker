#ifndef NAV_TYPES_H
#define NAV_TYPES_H

#include <cstdint>
#include <cstdlib>
#include "esp_heap_caps.h"

// Allocator that forces vectors into internal RAM (fast SRAM)
// instead of PSRAM (slow, fragmentation-prone).
// Pattern from IceNav-v3.
template <typename T>
struct InternalAllocator {
    using value_type = T;

    InternalAllocator() noexcept = default;
    template <typename U>
    InternalAllocator(const InternalAllocator<U>&) noexcept {}

    T* allocate(std::size_t n) {
        void* p = heap_caps_malloc(n * sizeof(T), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!p) p = malloc(n * sizeof(T)); // fallback to default heap
        return static_cast<T*>(p);
    }
    void deallocate(T* p, std::size_t) noexcept {
        heap_caps_free(p);
    }

    template <typename U>
    bool operator==(const InternalAllocator<U>&) const noexcept { return true; }
    template <typename U>
    bool operator!=(const InternalAllocator<U>&) const noexcept { return false; }
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
