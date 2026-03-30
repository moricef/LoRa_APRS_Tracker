/* Internal shared declarations for map_engine modules.
 * Not part of the public API — only included by map_engine.cpp and map_nav_render.cpp.
 */
#ifndef MAP_INTERNAL_H
#define MAP_INTERNAL_H

#ifdef USE_LVGL_UI

#include "map_engine.h"
#include "ui_map_manager.h"
#include <esp_log.h>
#include <vector>
#include <cmath>
#include <climits>
#include <esp_task_wdt.h>
#include <SD.h>

namespace MapEngine {

    // --- Shared tag ---
    extern const char* TAG_ENGINE;

    // --- Geometry type constants ---
    static constexpr uint8_t GEOM_POINT      = 1;
    static constexpr uint8_t GEOM_LINE       = 2;
    static constexpr uint8_t GEOM_POLYGON    = 3;
    static constexpr uint8_t GEOM_TEXT       = 4;
    static constexpr uint8_t GEOM_TEXT_LINE  = 5;

    // --- Feature reference for zero-copy rendering ---
    struct FeatureRef {
        uint8_t* ptr;
        uint8_t geomType;
        uint16_t payloadSize;
        uint16_t coordCount;
        int16_t tileOffsetX;
        int16_t tileOffsetY;
    };

    // --- Decoded feature coords result ---
    struct DecodedFeature {
        uint32_t coordsIdx;
        uint16_t ringCount;
        uint16_t* ringEnds;
    };

    // --- Hilbert curve functions for NPK3 ---
    static uint64_t xyToHilbert(uint32_t x, uint32_t y, uint8_t z) {
        uint32_t rx, ry;
        uint64_t d = 0;
        uint32_t n = 1 << z;
        for (uint32_t s = n / 2; s > 0; s /= 2) {
            rx = (x & s) > 0;
            ry = (y & s) > 0;
            d += static_cast<uint64_t>(s) * s * ((3 * rx) ^ ry);
            hilbertRot(s, &x, &y, rx, ry);
        }
        return d;
    }

    static void hilbertRot(uint32_t n, uint32_t* x, uint32_t* y, uint32_t rx, uint32_t ry) {
        if (ry == 0) {
            if (rx == 1) {
                *x = n - 1 - *x;
                *y = n - 1 - *y;
            }
            uint32_t t = *x;
            *x = *y;
            *y = t;
        }
    }

    // --- NPK slot (needed by renderNavViewport) ---
    struct NpkSlot {
        File file;
        UIMapManager::Npk2Header header;  // header common (magic, zoom)
        uint8_t version;  // 2 for NPK2, 3 for NPK3
        // NPK2 fields
        UIMapManager::Npk2YEntry* yTable;
        // NPK3 fields
        uint32_t tileCount3;
        uint32_t indexOffset3;
        // common fields
        char region[64];
        uint8_t zoom;
        uint8_t splitIdx;
        bool active;
        uint32_t lastAccess;
    };

    // --- NAV cache entry ---
    struct NavCacheEntry {
        uint8_t* data;
        size_t   size;
        int      tileX;
        int      tileY;
        uint32_t lastAccess;
        uint8_t  regionIdx;
        uint8_t  zoom;
    };

    #define NAV_CACHE_SIZE 12
    #define NPK_MAX_REGIONS 8

    // --- Shared static variables (defined in map_engine.cpp, used by map_nav_render.cpp) ---
    extern std::vector<UIMapManager::Edge, PSRAMAllocator<UIMapManager::Edge>> edgePool;
    extern std::vector<int, PSRAMAllocator<int>> edgeBuckets;
    extern std::vector<int, PSRAMAllocator<int>> proj32X;
    extern std::vector<int, PSRAMAllocator<int>> proj32Y;
    extern std::vector<int16_t, PSRAMAllocator<int16_t>> decodedCoords;
    extern std::vector<FeatureRef, PSRAMAllocator<FeatureRef>> globalLayers[16];
    extern std::vector<NavCacheEntry> navCache;
    extern uint32_t navCacheAccessCounter;

    extern lgfx::VLWfont vlwFont;
    extern bool vlwFontLoaded;
    extern volatile bool renderActive_;
    extern volatile bool deferredClearRequested;

    // Waterway label buffers
    static constexpr int WLABEL_MAX_PTS = 256;
    extern int*   wlScreenX;
    extern int*   wlScreenY;
    extern float* wlArcLen;
    extern LGFX_Sprite* glyphSprite;
    extern int glyphSpriteW, glyphSpriteH;

    extern NpkSlot npkSlots[NPK_MAX_REGIONS];

    // --- Functions shared between modules ---

    // VarInt decoding
    uint32_t readVarInt(const uint8_t* buf, uint32_t& offset, uint32_t limit);
    int16_t zigzagDecode(uint32_t n);
    bool decodeFeatureCoords(const uint8_t* payload, uint16_t coordCount,
                             uint16_t payloadSize, uint8_t geomType,
                             DecodedFeature& out);

    // NAV cache
    int findNavCache(uint8_t regionIdx, uint8_t zoom, int tileX, int tileY);
    void addNavCache(uint8_t regionIdx, uint8_t zoom, int tileX, int tileY,
                     uint8_t* data, size_t size,
                     const std::vector<uint8_t*>* inUse = nullptr);
    bool evictUnusedNavCache(const std::vector<uint8_t*>& inUse, size_t needed);

    // NPK
    NpkSlot* openNpkRegion(const char* region, uint8_t zoom, uint32_t tileY = 0);
    bool findNpkTileInSlot(NpkSlot* slot, uint32_t x, uint32_t y,
                           UIMapManager::Npk2IndexEntry* outEntry);
    bool readNpkTileData(NpkSlot* slot, const UIMapManager::Npk2IndexEntry* entry,
                         uint8_t** outData, size_t* outSize);

} // namespace MapEngine

#endif // USE_LVGL_UI
#endif // MAP_INTERNAL_H
