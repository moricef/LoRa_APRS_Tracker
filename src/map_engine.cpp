/* Map rendering engine for T-Deck Plus
 * Handles vector tile parsing, rendering, and caching.
 */
#ifdef USE_LVGL_UI

#include "map_engine.h"
#include "ui_map_manager.h"
#include <JPEGDEC.h>
#undef INTELSHORT
#undef INTELLONG
#undef MOTOSHORT
#undef MOTOLONG
#include <PNGdec.h>
#include "storage_utils.h"
#include "OpenSansBold6pt7b.h"
#include <SD.h>
#include <esp_task_wdt.h>
#include <algorithm>
#include <climits>
#include <cmath>

// Global sprite pointer for raster decoder callbacks
static LGFX_Sprite* targetSprite_ = nullptr;

namespace MapEngine {

    // VLW Unicode font for map labels
    static lgfx::VLWfont vlwFont;
    static bool vlwFontLoaded = false;

    // Handles for the asynchronous rendering system
    QueueHandle_t mapRenderQueue = nullptr;
    SemaphoreHandle_t spriteMutex = nullptr;
    static TaskHandle_t mapRenderTaskHandle = nullptr;
    static lv_obj_t* canvas_to_invalidate_ = nullptr;

    // Static vectors for AEL polygon filler (PSRAM to preserve DRAM)
    static std::vector<UIMapManager::Edge, PSRAMAllocator<UIMapManager::Edge>> edgePool;
    static std::vector<int, PSRAMAllocator<int>> edgeBuckets;

    // Static vectors for coordinate projection (PSRAM, pre-reserved)
    // IceNav-v3 pattern: separate int16_t buffers for lines, int buffers for polygons
    static std::vector<int16_t, PSRAMAllocator<int16_t>> proj16X;
    static std::vector<int16_t, PSRAMAllocator<int16_t>> proj16Y;
    static std::vector<int, PSRAMAllocator<int>> proj32X;
    static std::vector<int, PSRAMAllocator<int>> proj32Y;

    // Feature reference for zero-copy rendering (IceNav-v3 pattern: pointer into tile buffer)
    struct FeatureRef {
        uint8_t* ptr;           // Pointer to feature header in data buffer
        uint8_t geomType;       // 1=Point, 2=Line, 3=Polygon
        uint16_t ringCount;     // For polygons (uint8_t on disk, stored as uint16_t)
        uint16_t coordCount;    // Number of coordinates
        int16_t tileOffsetX;    // Pixel offset of tile top-left in viewport (IceNav: tilePixelOffsetX)
        int16_t tileOffsetY;    // Pixel offset of tile top-left in viewport (IceNav: tilePixelOffsetY)
    };
    // 16 priority layers (IceNav-v3 pattern: dispatch by getPriority() low nibble)
    static std::vector<FeatureRef, PSRAMAllocator<FeatureRef>> globalLayers[16];

    // Tile cache system
    #define TILE_CACHE_SIZE 40  // Number of tiles to cache (40 × 128KB = 5.2MB PSRAM)
    static std::vector<CachedTile> tileCache;
    static size_t maxCachedTiles = TILE_CACHE_SIZE;
    static uint32_t cacheAccessCounter = 0;

    // LVGL async call to invalidate the map canvas from another thread
    static void invalidate_map_canvas_cb(void* user_data) {
        lv_obj_t* canvas = (lv_obj_t*)user_data;
        if (canvas) {
            lv_obj_invalidate(canvas);
        }
    }

    // --- RASTER DECODING ENGINE ---
    static PNG png;
    static JPEGDEC jpeg;

    // Generic file callbacks for raster decoders
    static void* rasterOpenFile(const char* filename, int32_t* size) {
        File* file = new File(SD.open(filename, FILE_READ));
        if (!file || !*file) {
            delete file;
            return nullptr;
        }
        *size = file->size();
        return file;
    }

    static void rasterCloseFile(void* handle) {
        File* file = (File*)handle;
        if (file) {
            file->close();
            delete file;
        }
    }

    static int32_t rasterReadFile(void* handle, uint8_t* pBuf, int32_t iLen) {
        File* file = (File*)handle;
        return file->read(pBuf, iLen);
    }
    
    static int32_t rasterReadFileJPEG(JPEGFILE* pFile, uint8_t* pBuf, int32_t iLen) {
        return rasterReadFile(pFile->fHandle, pBuf, iLen);
    }

    static int32_t rasterReadFilePNG(PNGFILE* pFile, uint8_t* pBuf, int32_t iLen) {
        return rasterReadFile(pFile->fHandle, pBuf, iLen);
    }

    static int32_t rasterSeekFile(void* handle, int32_t iPosition) {
        File* file = (File*)handle;
        return file->seek(iPosition);
    }

    static int32_t rasterSeekFileJPEG(JPEGFILE* pFile, int32_t iPosition) {
        return rasterSeekFile(pFile->fHandle, iPosition);
    }

    static int32_t rasterSeekFilePNG(PNGFILE* pFile, int32_t iPosition) {
        return rasterSeekFile(pFile->fHandle, iPosition);
    }

    // JPEG draw callback
    static int jpegDrawCallback(JPEGDRAW* pDraw) {
        if (!targetSprite_) return 0;
        targetSprite_->pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
        return 1;
    }

    // PNG draw callback
    static int pngDrawCallback(PNGDRAW* pDraw) {
        if (!targetSprite_) return 0;
        uint16_t* pfb = (uint16_t*)targetSprite_->getBuffer();
        if(pfb) {
            uint16_t* pLine = pfb + (pDraw->y * MAP_TILE_SIZE);
            png.getLineAsRGB565(pDraw, pLine, PNG_RGB565_LITTLE_ENDIAN, 0xffffffff);
        }
        return 1;
    }

    // Raster renderers
    static bool renderJPGRaster(const char* path, LGFX_Sprite& map) {
        targetSprite_ = &map;
        bool success = false;
        if (xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (jpeg.open(path, rasterOpenFile, rasterCloseFile, rasterReadFileJPEG, rasterSeekFileJPEG, jpegDrawCallback) == 1) {
                jpeg.setPixelType(RGB565_LITTLE_ENDIAN);
                if (jpeg.decode(0, 0, 0) == 1) success = true;
                jpeg.close();
            }
            xSemaphoreGiveRecursive(spiMutex);
        }
        targetSprite_ = nullptr;
        return success;
    }

    static bool renderPNGRaster(const char* path, LGFX_Sprite& map) {
        targetSprite_ = &map;
        bool success = false;
        if (xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (png.open(path, rasterOpenFile, rasterCloseFile, rasterReadFilePNG, rasterSeekFilePNG, pngDrawCallback) == 1) {
                if (png.decode(nullptr, 0) == 1) success = true;
                png.close();
            }
            xSemaphoreGiveRecursive(spiMutex);
        }
        targetSprite_ = nullptr;
        return success;
    }

    // Background task to render map tiles on Core 0
    static void mapRenderTask(void* param) {
        RenderRequest request;
        Serial.println("[MAP] Render task started on Core 0");

        while (true) {
            if (xQueueReceive(mapRenderQueue, &request, portMAX_DELAY) == pdTRUE) {
                if (request.targetSprite) {
                    bool success = false;
                    // The renderTile function handles its own mutex for file access.
                    // We only need to lock when accessing the shared sprite, which renderTile does internally.
                    // No, wait, renderTile takes a reference to the sprite. We should lock here.
                    if (xSemaphoreTake(spriteMutex, portMAX_DELAY) == pdTRUE) {
                        success = renderTile(request.path, request.xOffset, request.yOffset, *request.targetSprite, (uint8_t)request.zoom);
                        xSemaphoreGive(spriteMutex);
                    }

                    if (success) {
                        // Caching is done here, after the sprite is successfully rendered
                        addToCache(request.path, request.zoom, request.tileX, request.tileY, request.targetSprite);
                    } else {
                        // If render failed, delete the sprite to prevent memory leaks
                        Serial.printf("[MAP] Render failed for %s, cleaning up sprite.\n", request.path);
                        request.targetSprite->deleteSprite();
                        delete request.targetSprite;
                    }

                    // Request a redraw on the LVGL thread regardless of success
                    // to show either the new tile or a cleared area.
                    lv_async_call(invalidate_map_canvas_cb, canvas_to_invalidate_);
                }
            }
        }
    }

    void stopRenderTask() {
        if (mapRenderTaskHandle) {
            vTaskDelete(mapRenderTaskHandle);
            mapRenderTaskHandle = nullptr;
        }
        if (mapRenderQueue) {
            vQueueDelete(mapRenderQueue);
            mapRenderQueue = nullptr;
        }
        if (spriteMutex) {
            vSemaphoreDelete(spriteMutex);
            spriteMutex = nullptr;
        }
        canvas_to_invalidate_ = nullptr;
        Serial.println("[MAP] Render task stopped.");
    }

    void startRenderTask(lv_obj_t* canvas_to_invalidate) {
        if (mapRenderTaskHandle) return;

        canvas_to_invalidate_ = canvas_to_invalidate;
        spriteMutex = xSemaphoreCreateMutex();
        mapRenderQueue = xQueueCreate(10, sizeof(RenderRequest));

        xTaskCreatePinnedToCore(
            mapRenderTask,
            "MapRender",
            16384,  // Increased for feature index + sort + AEL
            NULL,
            1, // Low priority
            &mapRenderTaskHandle,
            0  // Core 0
        );
    }

    // Initialize tile cache and pre-reserve render buffers
    void initTileCache() {
        for (auto& cachedTile : tileCache) {
            if (cachedTile.sprite) {
                cachedTile.sprite->deleteSprite();
                delete cachedTile.sprite;
            }
        }
        tileCache.clear();
        tileCache.reserve(maxCachedTiles);
        cacheAccessCounter = 0;

        // Pre-reserve AEL, projection, and feature index buffers in PSRAM
        // (IceNav-v3 constructor pattern: maps.cpp:57-63)
        edgePool.reserve(1024);
        edgeBuckets.reserve(768);
        proj16X.reserve(1024);
        proj16Y.reserve(1024);
        proj32X.reserve(1024);
        proj32Y.reserve(1024);
        for (int i = 0; i < 16; i++) globalLayers[i].reserve(256);

        Serial.printf("[MAP] Cache %d tiles, render buffers pre-reserved (PSRAM)\n", maxCachedTiles);
    }

    void clearTileCache() {
        initTileCache();
    }

    // Shrink projection buffers to baseline capacity to prevent memory bloat
    void shrinkProjectionBuffers() {
        const size_t BASELINE_CAPACITY = 1024;
        if (proj16X.capacity() > BASELINE_CAPACITY * 2) {
            proj16X.clear();
            proj16X.shrink_to_fit();
            proj16X.reserve(BASELINE_CAPACITY);
        }
        if (proj16Y.capacity() > BASELINE_CAPACITY * 2) {
            proj16Y.clear();
            proj16Y.shrink_to_fit();
            proj16Y.reserve(BASELINE_CAPACITY);
        }
        if (proj32X.capacity() > BASELINE_CAPACITY * 2) {
            proj32X.clear();
            proj32X.shrink_to_fit();
            proj32X.reserve(BASELINE_CAPACITY);
        }
        if (proj32Y.capacity() > BASELINE_CAPACITY * 2) {
            proj32Y.clear();
            proj32Y.shrink_to_fit();
            proj32Y.reserve(BASELINE_CAPACITY);
        }
    }

    // Load VLW Unicode font for map labels from SD card
    static uint8_t* vlwFontData = nullptr;
    static lgfx::PointerWrapper vlwFontWrapper;  // Must outlive vlwFont (VLWfont keeps _fontData pointer)

    bool loadMapFont() {
        if (vlwFontLoaded) return true;

        const char* fontPath = "/LoRa_Tracker/fonts/OpenSans-Bold-12.vlw";
        if (!SD.exists(fontPath)) {
            Serial.printf("[MAP] VLW font not found: %s (will use fallback GFX font)\n", fontPath);
            return false;
        }

        File file = SD.open(fontPath, FILE_READ);
        if (!file) {
            Serial.printf("[MAP] Failed to open VLW font: %s\n", fontPath);
            return false;
        }

        size_t fileSize = file.size();
        vlwFontData = (uint8_t*)heap_caps_malloc(fileSize, MALLOC_CAP_SPIRAM);
        if (!vlwFontData) {
            Serial.printf("[MAP] Failed to allocate %d bytes in PSRAM for VLW font\n", fileSize);
            file.close();
            return false;
        }

        size_t bytesRead = file.read(vlwFontData, fileSize);
        file.close();

        if (bytesRead != fileSize) {
            Serial.printf("[MAP] Failed to read VLW font: read %d/%d bytes\n", bytesRead, fileSize);
            heap_caps_free(vlwFontData);
            vlwFontData = nullptr;
            return false;
        }

        vlwFontWrapper.set(vlwFontData, fileSize);
        if (vlwFont.loadFont(&vlwFontWrapper)) {
            vlwFontLoaded = true;
            Serial.printf("[MAP] Loaded VLW font: %s (%d bytes in PSRAM)\n", fontPath, fileSize);
            return true;
        }

        Serial.printf("[MAP] VLW font validation failed\n");
        heap_caps_free(vlwFontData);
        vlwFontData = nullptr;
        return false;
    }

    // Find a tile in cache by its coordinates, returns index or -1
    int findCachedTile(int zoom, int tileX, int tileY) {
        uint32_t tileHash = (static_cast<uint32_t>(zoom) << 28) | (static_cast<uint32_t>(tileX) << 14) | static_cast<uint32_t>(tileY);

        for (int i = 0; i < tileCache.size(); ++i) {
            if (tileCache[i].isValid && tileCache[i].tileHash == tileHash) {
                tileCache[i].lastAccess = ++cacheAccessCounter;
                return i;
            }
        }
        return -1; // Not found
    }

    LGFX_Sprite* getCachedTileSprite(int index) {
        if (index >= 0 && index < tileCache.size()) {
            return tileCache[index].sprite;
        }
        return nullptr;
    }

    // Remove least recently used tile from cache
    static void evictLRUTile() {
        if (tileCache.empty()) return;

        auto lruIt = tileCache.begin();
        for (auto it = tileCache.begin(); it != tileCache.end(); ++it) {
            if (it->lastAccess < lruIt->lastAccess) {
                lruIt = it;
            }
        }

        Serial.printf("[CACHE] Evicting tile: %s\n", lruIt->filePath);
        if (lruIt->sprite) {
            lruIt->sprite->deleteSprite();
            delete lruIt->sprite;
            lruIt->sprite = nullptr;
        }
        tileCache.erase(lruIt);
    }

    // Add a rendered tile sprite to the cache
    void addToCache(const char* filePath, int zoom, int tileX, int tileY, LGFX_Sprite* sourceSprite) {
        if (maxCachedTiles == 0 || !sourceSprite) {
            if(sourceSprite) { // Don't leak memory if cache is disabled
                sourceSprite->deleteSprite();
                delete sourceSprite;
            }
            return;
        }

        if (tileCache.size() >= maxCachedTiles) {
            evictLRUTile();
        }

        CachedTile newEntry;
        newEntry.sprite = sourceSprite;

        strncpy(newEntry.filePath, filePath, sizeof(newEntry.filePath) - 1);
        newEntry.filePath[sizeof(newEntry.filePath) - 1] = '\0';
        newEntry.lastAccess = ++cacheAccessCounter;
        newEntry.isValid = true;
        newEntry.tileHash = (static_cast<uint32_t>(zoom) << 28) | (static_cast<uint32_t>(tileX) << 14) | static_cast<uint32_t>(tileY);

        tileCache.push_back(newEntry);
        Serial.printf("[CACHE] Added tile: %s\n", filePath);
        Serial.printf("[MAP] Cache size: %d, Free PSRAM: %u\n", tileCache.size(), ESP.getFreePsram());
    }
    
    // Helper function to safely copy a sprite to the canvas with clipping
    void copySpriteToCanvasWithClip(lv_obj_t* canvas, LGFX_Sprite* sprite, int offsetX, int offsetY) {
        if (!canvas || !sprite || spriteMutex == NULL) return;

        if (xSemaphoreTake(spriteMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (sprite->getBuffer() == nullptr) {
                xSemaphoreGive(spriteMutex);
                return;
            }

            int src_x = 0;
            int src_y = 0;
            int dest_x = offsetX;
            int dest_y = offsetY;
            int copy_w = MAP_TILE_SIZE;
            int copy_h = MAP_TILE_SIZE;

            if (dest_x < 0) {
                src_x = -dest_x;
                copy_w += dest_x;
                dest_x = 0;
            }
            if (dest_y < 0) {
                src_y = -dest_y;
                copy_h += dest_y;
                dest_y = 0;
            }
            if (dest_x + copy_w > MAP_CANVAS_WIDTH) {
                copy_w = MAP_CANVAS_WIDTH - dest_x;
            }
            if (dest_y + copy_h > MAP_CANVAS_HEIGHT) {
                copy_h = MAP_CANVAS_HEIGHT - dest_y;
            }

            if (copy_w > 0 && copy_h > 0) {
                uint16_t* src_buf = (uint16_t*)sprite->getBuffer();
                lv_color_t* dest_buf = UIMapManager::map_canvas_buf;

                for (int y = 0; y < copy_h; y++) {
                    uint16_t* src_ptr = src_buf + ((src_y + y) * MAP_TILE_SIZE) + src_x;
                    lv_color_t* dest_ptr = dest_buf + ((dest_y + y) * MAP_CANVAS_WIDTH) + dest_x;
#if LV_COLOR_16_SWAP
                    // LGFX sprites are little-endian RGB565, LVGL canvas is big-endian
                    uint16_t* dp = (uint16_t*)dest_ptr;
                    for (int x = 0; x < copy_w; x++) {
                        uint16_t px = src_ptr[x];
                        dp[x] = (px >> 8) | (px << 8);
                    }
#else
                    memcpy(dest_ptr, src_ptr, copy_w * sizeof(lv_color_t));
#endif
                }
            }
            xSemaphoreGive(spriteMutex);
        }
    }

    static bool fillPolygons = true;

    uint16_t darkenRGB565(const uint16_t color, const float amount) {
        uint16_t r = (color >> 11) & 0x1F;
        uint16_t g = (color >> 5) & 0x3F;
        uint16_t b = color & 0x1F;
        r = (uint16_t)(r * (1.0f - amount));
        g = (uint16_t)(g * (1.0f - amount));
        b = (uint16_t)(b * (1.0f - amount));
        return (r << 11) | (g << 5) | b;
    }

    // AEL polygon filler with fast-forward optimization (IceNav-v3 pattern).
    // Takes high-precision (HP) coordinates (0-4096), iterates pixel-space scanlines (0-255).
    // Supports multi-ring polygons (exterior + holes).
    void fillPolygonGeneral(LGFX_Sprite &map, const int *px_hp, const int *py_hp, const int numPoints, const uint16_t color, const int xOffset, const int yOffset, uint16_t ringCount, uint16_t* ringEnds)
    {
        if (numPoints < 3) return;

        // 1. Find Y bounds in pixel space
        int minY_px = INT_MAX, maxY_px = INT_MIN;
        for (int i = 0; i < numPoints; i++) {
            int y_px = py_hp[i] >> 4;
            if (y_px < minY_px) minY_px = y_px;
            if (y_px > maxY_px) maxY_px = y_px;
        }

        int spriteH = map.height();
        if (maxY_px < 0 || minY_px >= spriteH) return;

        // 2. Set up pixel-space edge buckets
        edgePool.clear();
        int bucketCount = maxY_px - minY_px + 1;
        if (bucketCount <= 0) return;
        edgeBuckets.assign(bucketCount, -1);

        uint16_t count = (ringCount == 0) ? 1 : ringCount;
        uint16_t defaultEnds[1] = { (uint16_t)numPoints };
        uint16_t* ends = (ringEnds == nullptr) ? defaultEnds : ringEnds;

        int ringStart = 0;
        for (uint16_t r = 0; r < count; r++) {
            int ringEnd = ends[r];
            int ringNumPoints = ringEnd - ringStart;
            if (ringNumPoints < 3) {
                ringStart = ringEnd;
                continue;
            }

            // 3. Populate buckets with edges using HP math for precision
            for (int i = 0; i < ringNumPoints; i++) {
                int next = (i + 1) % ringNumPoints;
                int x1_hp = px_hp[ringStart + i], y1_hp = py_hp[ringStart + i];
                int x2_hp = px_hp[ringStart + next], y2_hp = py_hp[ringStart + next];

                int y1_px = y1_hp >> 4;
                int y2_px = y2_hp >> 4;

                if (y1_px == y2_px) continue;

                UIMapManager::Edge e;
                e.nextActive = -1;

                if (y1_hp < y2_hp) {
                    e.yMax = y2_px;
                    e.slope = ((int64_t)(x2_hp - x1_hp) << 16) / (y2_hp - y1_hp);
                    e.xVal = ((int64_t)x1_hp << 16);
                    if (y1_px - minY_px >= 0 && (size_t)(y1_px - minY_px) < edgeBuckets.size()) {
                        e.nextInBucket = edgeBuckets[y1_px - minY_px];
                        edgePool.push_back(e);
                        edgeBuckets[y1_px - minY_px] = edgePool.size() - 1;
                    }
                } else {
                    e.yMax = y1_px;
                    e.slope = ((int64_t)(x1_hp - x2_hp) << 16) / (y1_hp - y2_hp);
                    e.xVal = ((int64_t)x2_hp << 16);
                    if (y2_px - minY_px >= 0 && (size_t)(y2_px - minY_px) < edgeBuckets.size()) {
                        e.nextInBucket = edgeBuckets[y2_px - minY_px];
                        edgePool.push_back(e);
                        edgeBuckets[y2_px - minY_px] = edgePool.size() - 1;
                    }
                }
            }
            ringStart = ringEnd;
        }

        int activeHead = -1;
        int spriteW = map.width();
        // Clip Y range accounting for offset (IceNav-v3 pattern: maps.cpp:953-954)
        int startY_px = std::max(minY_px, -yOffset);
        int endY_px = std::min(maxY_px, spriteH - 1 - yOffset);

        // 4. Fast-forward: process buckets before visible range, jump edge xVal
        //    directly to startY_px (IceNav-v3 pattern — skip invisible scanlines)
        if (startY_px > minY_px) {
            for (int y = minY_px; y < startY_px; y++) {
                if ((size_t)(y - minY_px) >= edgeBuckets.size()) break;
                int eIdx = edgeBuckets[y - minY_px];
                while (eIdx != -1) {
                    int nextIdx = edgePool[eIdx].nextInBucket;
                    // Jump xVal to startY_px: (startY_px - y) pixel scanlines × 16 HP units each
                    edgePool[eIdx].xVal += (int64_t)edgePool[eIdx].slope * 16 * (startY_px - y);
                    edgePool[eIdx].nextActive = activeHead;
                    activeHead = eIdx;
                    eIdx = nextIdx;
                }
            }
            // Remove edges that finish before reaching the visible range
            int* pCurrIdx = &activeHead;
            while (*pCurrIdx != -1) {
                if (edgePool[*pCurrIdx].yMax <= startY_px)
                    *pCurrIdx = edgePool[*pCurrIdx].nextActive;
                else
                    pCurrIdx = &(edgePool[*pCurrIdx].nextActive);
            }
        }

        // 5. Main scanline loop (visible range only)
        int scanlineCount = 0;
        for (int y_px = startY_px; y_px <= endY_px; y_px++) {
            if (++scanlineCount >= 32) {
                esp_task_wdt_reset();
                scanlineCount = 0;
            }

            // Add new edges from bucket
            if ((size_t)(y_px - minY_px) < edgeBuckets.size()) {
                int eIdx = edgeBuckets[y_px - minY_px];
                while (eIdx != -1) {
                    int nextIdx = edgePool[eIdx].nextInBucket;
                    edgePool[eIdx].nextActive = activeHead;
                    activeHead = eIdx;
                    eIdx = nextIdx;
                }
            }

            // Remove finished edges
            int* pCurrIdx = &activeHead;
            while (*pCurrIdx != -1) {
                if (edgePool[*pCurrIdx].yMax <= y_px)
                    *pCurrIdx = edgePool[*pCurrIdx].nextActive;
                else
                    pCurrIdx = &(edgePool[*pCurrIdx].nextActive);
            }

            if (activeHead == -1) continue;

            // Sort AEL by xVal (insertion sort into linked list)
            int sorted = -1;
            int active = activeHead;
            while (active != -1) {
                int nextActive = edgePool[active].nextActive;
                if (sorted == -1 || edgePool[active].xVal < edgePool[sorted].xVal) {
                    edgePool[active].nextActive = sorted;
                    sorted = active;
                } else {
                    int s = sorted;
                    while (edgePool[s].nextActive != -1 && edgePool[edgePool[s].nextActive].xVal < edgePool[active].xVal)
                        s = edgePool[s].nextActive;
                    edgePool[active].nextActive = edgePool[s].nextActive;
                    edgePool[s].nextActive = active;
                }
                active = nextActive;
            }
            activeHead = sorted;

            // Draw horizontal spans
            int left = activeHead;
            while (left != -1 && edgePool[left].nextActive != -1) {
                int right = edgePool[left].nextActive;
                int xStart = (edgePool[left].xVal >> 16) >> 4;
                int xEnd = (edgePool[right].xVal >> 16) >> 4;
                if (xStart < 0) xStart = 0;
                if (xEnd > spriteW) xEnd = spriteW;
                if (xEnd > xStart)
                    map.drawFastHLine(xStart + xOffset, y_px + yOffset, xEnd - xStart, color);
                left = edgePool[right].nextActive;
            }

            // Update xVal for next pixel scanline (step = slope × 16 HP units)
            for (int a = activeHead; a != -1; a = edgePool[a].nextActive)
                edgePool[a].xVal += (int64_t)edgePool[a].slope * 16;
        }
    }

    // =========================================================================
    // Viewport-based NAV rendering (IceNav-v3 renderNavViewport pattern).
    // Loads ALL visible tiles, dispatches features to 16 priority layers,
    // renders in a single pass with per-feature setClipRect to tile boundaries.
    // This ensures correct z-ordering across tile boundaries.
    // =========================================================================
    bool renderNavViewport(float centerLat, float centerLon, uint8_t zoom,
                           LGFX_Sprite &map, const char* region) {
        esp_task_wdt_reset();
        uint64_t startTime = esp_timer_get_time();

        int viewportW = map.width();
        int viewportH = map.height();

        // Compute tile grid (IceNav-v3 pattern: maps.cpp:1478-1494)
        const double latRad = (double)centerLat * M_PI / 180.0;
        const double n = pow(2.0, (double)zoom);
        const float centerTileX = (float)((centerLon + 180.0) / 360.0 * n);
        const float centerTileY = (float)((1.0 - log(tan(latRad) + 1.0 / cos(latRad)) / M_PI) / 2.0 * n);

        const int centerTileIdxX = (int)floorf(centerTileX);
        const int centerTileIdxY = (int)floorf(centerTileY);

        // Sub-tile pixel offset (fractional part → pixel position within center tile)
        float fracX = centerTileX - centerTileIdxX;
        float fracY = centerTileY - centerTileIdxY;
        int centerTileOriginX = viewportW / 2 - (int)(fracX * MAP_TILE_SIZE);
        int centerTileOriginY = viewportH / 2 - (int)(fracY * MAP_TILE_SIZE);

        // Determine tile range to cover entire viewport
        int minDx = -(centerTileOriginX / MAP_TILE_SIZE + 1);
        int maxDx = (viewportW - centerTileOriginX + MAP_TILE_SIZE - 1) / MAP_TILE_SIZE;
        int minDy = -(centerTileOriginY / MAP_TILE_SIZE + 1);
        int maxDy = (viewportH - centerTileOriginY + MAP_TILE_SIZE - 1) / MAP_TILE_SIZE;

        // --- Load all tiles and dispatch features (IceNav-v3 pattern: maps.cpp:1498-1543) ---
        std::vector<uint8_t*> tileBuffers;
        for (int i = 0; i < 16; i++) globalLayers[i].clear();
        // One-time PSRAM reserve: reduces reallocations on first render
        static bool layersReserved = false;
        if (!layersReserved) {
            for (int i = 0; i < 16; i++) globalLayers[i].reserve(256);
            layersReserved = true;
        }
        // Text labels collected separately — rendered last, on top of all geometry
        std::vector<FeatureRef, PSRAMAllocator<FeatureRef>> textRefs;
        textRefs.reserve(128);

        uint16_t bgColor = 0xF7BE;  // Default OSM beige (0xF2EFE9) if no tiles loaded
        bool bgColorExtracted = false;
        bool psramExhausted = false;

        // Build tile list sorted center-outward so PSRAM exhaustion
        // degrades edges first instead of cutting a whole quadrant
        struct TileSlot { int dx, dy; int distSq; };
        TileSlot tileOrder[36];  // 6×6 max
        int tileCount = 0;
        for (int dy = minDy; dy <= maxDy; dy++) {
            for (int dx = minDx; dx <= maxDx; dx++) {
                if (tileCount < 36) {
                    tileOrder[tileCount++] = { dx, dy, dx*dx + dy*dy };
                }
            }
        }
        std::sort(tileOrder, tileOrder + tileCount,
                  [](const TileSlot& a, const TileSlot& b) { return a.distSq < b.distSq; });

        for (int ti = 0; ti < tileCount && !psramExhausted; ti++) {
                int dx = tileOrder[ti].dx;
                int dy = tileOrder[ti].dy;
                esp_task_wdt_reset();

                int tileX = centerTileIdxX + dx;
                int tileY = centerTileIdxY + dy;

                // Pixel offset of this tile's top-left in the viewport
                int16_t tileOffsetX = (int16_t)(centerTileOriginX + dx * MAP_TILE_SIZE);
                int16_t tileOffsetY = (int16_t)(centerTileOriginY + dy * MAP_TILE_SIZE);

                // Skip tiles entirely outside viewport
                bool skipX = (tileOffsetX + MAP_TILE_SIZE <= 0 || tileOffsetX >= viewportW);
                bool skipY = (tileOffsetY + MAP_TILE_SIZE <= 0 || tileOffsetY >= viewportH);
                if (skipX || skipY) {
                    Serial.printf("[NAV-SKIP] Tile %d/%d: offset=(%d,%d) skipX=%d skipY=%d\n",
                                  tileX, tileY, tileOffsetX, tileOffsetY, skipX, skipY);
                    continue;
                }

                char tilePath[128];
                snprintf(tilePath, sizeof(tilePath), "/LoRa_Tracker/VectMaps/%s/%d/%d/%d.nav",
                         region, zoom, tileX, tileY);

                // Read tile file under SPI mutex
                uint8_t* data = nullptr;
                size_t fileSize = 0;
                bool mutexOk = false, fileOk = false, sizeOk = false, allocOk = false;

                if (spiMutex != NULL && xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    mutexOk = true;
                    File file = SD.open(tilePath, FILE_READ);
                    if (file) {
                        fileOk = true;
                        fileSize = file.size();
                        if (fileSize >= 22) {
                            sizeOk = true;
                            data = (uint8_t*)ps_malloc(fileSize);
                            if (data) {
                                allocOk = true;
                                file.read(data, fileSize);
                            }
                        }
                        file.close();
                    }
                    xSemaphoreGiveRecursive(spiMutex);
                }

                if (!data) {
                    Serial.printf("[NAV-FAIL] Tile %d/%d: mutex=%d file=%d size=%d(%u) alloc=%d\n",
                                  tileX, tileY, mutexOk, fileOk, sizeOk, (unsigned)fileSize, allocOk);
                    // If file exists but ps_malloc failed → PSRAM exhausted, stop loading
                    if (sizeOk && !allocOk) {
                        Serial.printf("[NAV] PSRAM exhausted (needed %u KB, largest block: %u KB) — stopping tile load\n",
                                      (unsigned)(fileSize / 1024),
                                      (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024));
                        psramExhausted = true;
                    }
                    continue;
                }
                if (memcmp(data, "NAV1", 4) != 0) {
                    Serial.printf("[NAV-FAIL] Tile %d/%d: invalid header (not NAV1)\n", tileX, tileY);
                    free(data);
                    continue;
                }

                // Extract background color from first feature (background polygon) on first tile
                if (!bgColorExtracted && fileSize >= 24) {
                    // First feature at offset 22: type(1) + color(2) + ...
                    memcpy(&bgColor, data + 23, 2);  // RGB565 at offset 23 (after type byte)
                    bgColorExtracted = true;
                    Serial.printf("[NAV-BG] Background color extracted: 0x%04X\n", bgColor);
                }

                tileBuffers.push_back(data);

                uint16_t feature_count;
                memcpy(&feature_count, data + 4, 2);

                Serial.printf("[NAV-DBG] Tile %d/%d: %u features, %u bytes\n",
                              tileX, tileY, feature_count, (unsigned)fileSize);

                // Parse features and dispatch to priority layers (IceNav-v3 zero-copy pattern)
                uint8_t* p = data + 22;
                int typeCounts[5] = {0, 0, 0, 0, 0};  // types 0-4
                int skippedZoom = 0;
                int skippedOverflow = 0;

                for (uint16_t i = 0; i < feature_count; i++) {
                    if ((i & 63) == 0) esp_task_wdt_reset();
                    if (p + 12 > data + fileSize) {
                        Serial.printf("[NAV-DBG]   BREAK at feature %d: header overflow (p=%u, end=%u)\n",
                                      i, (unsigned)(p - data), (unsigned)fileSize);
                        skippedOverflow = feature_count - i;
                        break;
                    }

                    uint8_t geomType = p[0];
                    uint8_t zoomPriority = p[3];
                    uint16_t coordCount;
                    memcpy(&coordCount, p + 9, 2);

                    // Skip first feature (background polygon) - already used for fillSprite
                    if (i == 0) {
                        uint32_t bg_data_size = coordCount * 4;
                        if (geomType == 3) {  // Polygon
                            uint8_t* ring_ptr = p + 12 + bg_data_size;
                            if (ring_ptr + 2 <= data + fileSize) {
                                uint16_t bgRingCount;
                                memcpy(&bgRingCount, ring_ptr, 2);
                                if (ring_ptr + 2 + (bgRingCount * 2) <= data + fileSize)
                                    bg_data_size += 2 + (bgRingCount * 2);
                            }
                        }
                        p += 12 + bg_data_size;
                        continue;
                    }

                    uint32_t feature_data_size = coordCount * 4;
                    uint16_t ringCount = 0;

                    // Polygon ring data: uint16 ringCount (2 bytes) + ringCount × uint16 ringEnds
                    // (matching Python tile_generator.py: struct.pack('<H', ring_count))
                    if (geomType == 3) {
                        uint8_t* ring_ptr = p + 12 + feature_data_size;
                        if (ring_ptr + 2 <= data + fileSize) {
                            memcpy(&ringCount, ring_ptr, 2);
                            if (ring_ptr + 2 + (ringCount * 2) <= data + fileSize)
                                feature_data_size += 2 + (ringCount * 2);
                            else
                                ringCount = 0;
                        }
                    }

                    if (p + 12 + feature_data_size > data + fileSize) {
                        Serial.printf("[NAV-DBG]   BREAK at feature %d: data overflow (type=%d, coords=%d, size=%u, p=%u, end=%u)\n",
                                      i, geomType, coordCount, feature_data_size, (unsigned)(p - data), (unsigned)fileSize);
                        skippedOverflow = feature_count - i;
                        break;
                    }

                    // Zoom filtering (IceNav-v3 NavReader pattern)
                    uint8_t minZoom = zoomPriority >> 4;
                    if (minZoom > zoom) {
                        skippedZoom++;
                        p += 12 + feature_data_size;
                        continue;
                    }

                    if (geomType <= 4) typeCounts[geomType]++;

                    uint8_t priority = zoomPriority & 0x0F;
                    if (priority >= 16) priority = 15;

                    FeatureRef ref;
                    ref.ptr = p;
                    ref.geomType = geomType;
                    ref.ringCount = ringCount;
                    ref.coordCount = coordCount;
                    ref.tileOffsetX = tileOffsetX;
                    ref.tileOffsetY = tileOffsetY;
                    // Text labels rendered last (separate pass) so they appear above all geometry
                    // Safety: when vector is at capacity, push_back doubles the allocation.
                    // New buffer = capacity*2*sizeof(FeatureRef), allocated BEFORE old is freed.
                    // Check largest contiguous PSRAM block can hold the new buffer.
                    if (geomType == 4) {
                        if (textRefs.size() == textRefs.capacity()) {
                            size_t needed = (textRefs.capacity() < 1 ? 1 : textRefs.capacity() * 2) * sizeof(FeatureRef);
                            if (heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) < needed) {
                                skippedOverflow++;
                                p += 12 + feature_data_size;
                                continue;
                            }
                        }
                        textRefs.push_back(ref);
                    } else {
                        auto& layer = globalLayers[priority];
                        if (layer.size() == layer.capacity()) {
                            size_t needed = (layer.capacity() < 1 ? 1 : layer.capacity() * 2) * sizeof(FeatureRef);
                            if (heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) < needed) {
                                skippedOverflow++;
                                p += 12 + feature_data_size;
                                continue;
                            }
                        }
                        layer.push_back(ref);
                    }

                    p += 12 + feature_data_size;
                }

                Serial.printf("[NAV-DBG]   Kept: pt=%d ln=%d poly=%d txt=%d | skipped: zoom=%d overflow=%d\n",
                              typeCounts[1], typeCounts[2], typeCounts[3], typeCounts[4],
                              skippedZoom, skippedOverflow);
        }

        // Log load stats (IceNav-v3 pattern: maps.cpp:1582-1587)
        uint64_t loadEnd = esp_timer_get_time();
        int totalFeatures = 0;
        for (int i = 0; i < 16; i++) totalFeatures += globalLayers[i].size();
        Serial.printf("[NAV] Load: %llu ms, tiles: %d, features: %d, grid: [%d..%d]x[%d..%d]\n",
                      (loadEnd - startTime) / 1000, (int)tileBuffers.size(), totalFeatures,
                      minDx, maxDx, minDy, maxDy);

        // --- Render all layers (IceNav-v3 pattern: maps.cpp:1546-1569) ---
        // Fill background with color from NAV background polygon
        map.fillSprite(bgColor);

        map.startWrite();

        struct LabelRect { int16_t x, y, w, h; };
        std::vector<LabelRect> placedLabels;
        placedLabels.reserve(128);

        int featureCount = 0;
        for (int pri = 0; pri < 16; pri++) {
            if (globalLayers[pri].empty()) continue;

            for (const auto& ref : globalLayers[pri]) {
                // WDT reset every 32 features during rendering (prevents WDT timeout)
                if ((++featureCount & 31) == 0) esp_task_wdt_reset();

                // Yield to LVGL every 512 features to keep touch responsive
                // (~100ms chunks instead of 2.5s blocking)
                if ((featureCount & 511) == 0) {
                    map.endWrite();
                    lv_timer_handler();
                    map.startWrite();
                }

                uint8_t* fp = ref.ptr;

                // Per-feature setClipRect to tile boundaries (IceNav-v3: maps.cpp:1561)
                map.setClipRect(ref.tileOffsetX, ref.tileOffsetY, MAP_TILE_SIZE, MAP_TILE_SIZE);

                // BBox culling against viewport (IceNav-v3: maps.cpp:1554-1559)
                uint8_t bx1 = fp[5], by1 = fp[6], bx2 = fp[7], by2 = fp[8];
                int16_t minX = ref.tileOffsetX + bx1;
                int16_t minY = ref.tileOffsetY + by1;
                int16_t maxX = ref.tileOffsetX + bx2;
                int16_t maxY = ref.tileOffsetY + by2;
                if (maxX < 0 || minX > viewportW || maxY < 0 || minY > viewportH) continue;

                // Read colorRgb565 directly (LE, no byte swap — IceNav-v3 pattern)
                uint16_t colorRgb565;
                memcpy(&colorRgb565, fp + 1, 2);

                // Render feature by geometry type (mixed per layer)
                switch (ref.geomType) {
                    case 3: { // Polygon (IceNav-v3: renderNavPolygon)
                        if (ref.coordCount < 3) break;
                        int16_t* coords = (int16_t*)(fp + 12);
                        // ringEnds starts after 1-byte ringCount (not 2)
                        uint16_t* ringEnds = (ref.ringCount > 0)
                            ? (uint16_t*)(fp + 12 + ref.coordCount * 4 + 2) : nullptr;

                        if (proj32X.capacity() < ref.coordCount) proj32X.reserve(ref.coordCount * 3 / 2);
                        if (proj32Y.capacity() < ref.coordCount) proj32Y.reserve(ref.coordCount * 3 / 2);
                        proj32X.resize(ref.coordCount);
                        proj32Y.resize(ref.coordCount);

                        int* px_hp = proj32X.data();
                        int* py_hp = proj32Y.data();
                        if (!px_hp || !py_hp) break;

                        for (uint16_t j = 0; j < ref.coordCount; j++) {
                            // Clamp HP coords to [0, 4096] — values outside this range
                            // cause aberrant AEL shapes (disk/circle artefacts)
                            px_hp[j] = std::max(0, std::min(4096, (int)coords[j * 2]));
                            py_hp[j] = std::max(0, std::min(4096, (int)coords[j * 2 + 1]));
                        }

                        if (fillPolygons) {
                            fillPolygonGeneral(map, px_hp, py_hp, ref.coordCount,
                                colorRgb565, ref.tileOffsetX, ref.tileOffsetY,
                                ref.ringCount, ringEnds);
                        }

                        // Building outline (bit 7 of fp[4])
                        if ((fp[4] & 0x80) != 0) {
                            uint16_t outlineColor = darkenRGB565(colorRgb565, 0.35f);
                            uint16_t ringStart = 0;
                            uint16_t numRings = (ref.ringCount > 0) ? ref.ringCount : 1;
                            for (uint16_t r = 0; r < numRings; r++) {
                                uint16_t ringEnd = (ringEnds && r < ref.ringCount) ? ringEnds[r] : ref.coordCount;
                                if (ringEnd > ref.coordCount) ringEnd = ref.coordCount;
                                for (uint16_t j = ringStart; j < ringEnd; j++) {
                                    uint16_t next = (j + 1 < ringEnd) ? j + 1 : ringStart;
                                    int x0 = (px_hp[j] >> 4) + ref.tileOffsetX;
                                    int y0 = (py_hp[j] >> 4) + ref.tileOffsetY;
                                    int x1 = (px_hp[next] >> 4) + ref.tileOffsetX;
                                    int y1 = (py_hp[next] >> 4) + ref.tileOffsetY;
                                    map.drawLine(x0, y0, x1, y1, outlineColor);
                                }
                                ringStart = ringEnd;
                            }
                        }

                        break;
                    }
                    case 2: { // LineString (IceNav-v3: renderNavLineString with dedup + bbox)
                        if (ref.coordCount < 2) break;
                        bool hasCasing = (fp[4] & 0x80) != 0;  // bit 7 = casing flag
                        uint8_t widthPixels = fp[4] & 0x7F;     // bits 6-0 = actual width
                        if (widthPixels == 0) widthPixels = 1;
                        int16_t* coords = (int16_t*)(fp + 12);

                        // Pre-project all coords with dedup (IceNav-v3: renderNavLineString L1287-1324)
                        size_t numCoords = ref.coordCount;
                        if (proj16X.capacity() < numCoords) proj16X.reserve(numCoords * 3 / 2);
                        if (proj16Y.capacity() < numCoords) proj16Y.reserve(numCoords * 3 / 2);
                        proj16X.resize(numCoords);
                        proj16Y.resize(numCoords);

                        int16_t* pxArr = proj16X.data();
                        int16_t* pyArr = proj16Y.data();

                        int16_t minPx = INT16_MAX, maxPx = INT16_MIN;
                        int16_t minPy = INT16_MAX, maxPy = INT16_MIN;
                        size_t validPoints = 0;
                        int16_t lastPx = -32768, lastPy = -32768;

                        // Simplification: skip vertices within widthPixels/2 of last kept
                        int distThreshSq = (widthPixels * widthPixels) / 4;
                        if (distThreshSq < 1) distThreshSq = 1;

                        for (size_t j = 0; j < numCoords; j++) {
                            int16_t px = (coords[j * 2] >> 4) + ref.tileOffsetX;
                            int16_t py = (coords[j * 2 + 1] >> 4) + ref.tileOffsetY;

                            if (validPoints > 0) {
                                int dx = px - lastPx;
                                int dy = py - lastPy;
                                // Skip if too close, but always keep the last vertex
                                if ((dx*dx + dy*dy) < distThreshSq && j < numCoords - 1) continue;
                            }

                            pxArr[validPoints] = px;
                            pyArr[validPoints] = py;
                            if (px < minPx) minPx = px;
                            if (px > maxPx) maxPx = px;
                            if (py < minPy) minPy = py;
                            if (py > maxPy) maxPy = py;
                            lastPx = px;
                            lastPy = py;
                            validPoints++;
                        }

                        // Bbox check on projected line (IceNav-v3 L1326)
                        if (validPoints < 2 || maxPx < 0 || minPx >= viewportW ||
                            maxPy < 0 || minPy >= viewportH) break;

                        for (size_t j = 1; j < validPoints; j++) {
                            if (widthPixels <= 2) {
                                map.drawLine(pxArr[j-1], pyArr[j-1], pxArr[j], pyArr[j], colorRgb565);
                            } else {
                                map.drawWideLine(pxArr[j-1], pyArr[j-1], pxArr[j], pyArr[j],
                                                 widthPixels, colorRgb565);
                                map.setClipRect(ref.tileOffsetX, ref.tileOffsetY, MAP_TILE_SIZE, MAP_TILE_SIZE);
                            }
                        }
                        break;
                    }
                    case 1: { // Point (IceNav-v3: renderNavPoint with bounds check)
                        if (ref.coordCount < 1) break;
                        int16_t* coords = (int16_t*)(fp + 12);
                        int px = (coords[0] >> 4) + ref.tileOffsetX;
                        int py = (coords[1] >> 4) + ref.tileOffsetY;
                        // Bounds check (IceNav-v3: renderNavPoint L1418)
                        if (px >= 0 && px < viewportW && py >= 0 && py < viewportH)
                            map.fillCircle(px, py, 3, colorRgb565);
                        break;
                    }
                    case 4: { // Text label (GEOM_TEXT)
                        uint8_t fontSize = fp[4];
                        int16_t* coords = (int16_t*)(fp + 12);
                        int px = (coords[0] >> 4) + ref.tileOffsetX;
                        int py = (coords[1] >> 4) + ref.tileOffsetY;
                        uint8_t textLen = *(fp + 12 + 4);
                        if (textLen > 0 && textLen < 128) {
                            char textBuf[128];
                            memcpy(textBuf, fp + 12 + 5, textLen);
                            textBuf[textLen] = '\0';

                            // Use VLW Unicode font if loaded, fallback to GFX font
                            if (vlwFontLoaded) {
                                map.setFont(&vlwFont);
                                // Scale VLW font based on fontSize (0=small, 1=medium, 2=large)
                                float scale = (fontSize == 0) ? 0.8f : (fontSize == 1) ? 1.0f : 1.2f;
                                map.setTextSize(scale);
                            } else {
                                map.setFont((lgfx::GFXfont*)&OpenSans_Bold6pt7b);
                                map.setTextSize(1);
                            }

                            // Measure label bbox for collision detection
                            int tw = map.textWidth(textBuf);
                            int th = map.fontHeight();
                            int lx = px - tw / 2;  // center horizontally
                            int ly = py - th;       // above the point
                            const int PAD = 4;

                            // Viewport bounds check (label must be at least partially visible)
                            if (lx + tw < 0 || lx >= viewportW || ly + th < 0 || ly >= viewportH) break;

                            // Check collision with already placed labels
                            bool collision = false;
                            for (const auto& r : placedLabels) {
                                if (lx - PAD < r.x + r.w && lx + tw + PAD > r.x &&
                                    ly - PAD < r.y + r.h && ly + th + PAD > r.y) {
                                    collision = true;
                                    break;
                                }
                            }
                            if (collision) break;

                            // Lift tile clipRect so labels can span tile boundaries
                            map.clearClipRect();

                            // Draw label
                            map.setTextColor(colorRgb565);
                            map.setTextDatum(lgfx::top_center);
                            map.drawString(textBuf, px, ly);
                            map.setTextDatum(lgfx::top_left); // restore default

                            // Restore tile clipRect for subsequent features
                            map.setClipRect(ref.tileOffsetX, ref.tileOffsetY, MAP_TILE_SIZE, MAP_TILE_SIZE);

                            placedLabels.push_back({(int16_t)lx, (int16_t)ly, (int16_t)tw, (int16_t)th});
                        }
                        break;
                    }
                }
            }
            globalLayers[pri].clear();
            taskYIELD();
        }

        // Label pass — rendered after all geometry so labels appear on top
        map.clearClipRect();
        map.setFont((lgfx::GFXfont*)&OpenSans_Bold6pt7b);
        map.setTextSize(1);
        // Set VLW Unicode font for text labels (must be before textWidth/drawString)
        if (vlwFontLoaded) {
            map.setFont(&vlwFont);
            map.setTextSize(1.0f);
        } else {
            map.setFont((lgfx::GFXfont*)&OpenSans_Bold6pt7b);
            map.setTextSize(1);
        }

        for (const auto& ref : textRefs) {
            if ((++featureCount & 31) == 0) esp_task_wdt_reset();
            uint8_t* fp = ref.ptr;
            uint16_t colorRgb565;
            memcpy(&colorRgb565, fp + 1, 2);
            int16_t* coords = (int16_t*)(fp + 12);
            int px = (coords[0] >> 4) + ref.tileOffsetX;
            int py = (coords[1] >> 4) + ref.tileOffsetY;
            uint8_t fontSize = fp[4];
            uint8_t textLen = *(fp + 12 + 4);
            if (textLen == 0 || textLen >= 128) continue;
            char textBuf[128];
            memcpy(textBuf, fp + 12 + 5, textLen);
            textBuf[textLen] = '\0';

            // Scale VLW font based on fontSize (0=small, 1=medium, 2=large)
            if (vlwFontLoaded) {
                float scale = (fontSize == 0) ? 0.8f : (fontSize == 1) ? 1.0f : 1.2f;
                map.setTextSize(scale);
            }

            int tw = map.textWidth(textBuf);
            int th = map.fontHeight();
            int lx = px - tw / 2;
            int ly = py - th;
            const int PAD = 4;
            if (lx + tw < 0 || lx >= viewportW || ly + th < 0 || ly >= viewportH) continue;
            bool collision = false;
            for (const auto& r : placedLabels) {
                if (lx - PAD < r.x + r.w && lx + tw + PAD > r.x &&
                    ly - PAD < r.y + r.h && ly + th + PAD > r.y) {
                    collision = true; break;
                }
            }
            if (collision) continue;
            map.setTextColor(colorRgb565);
            map.setTextDatum(lgfx::top_center);
            map.drawString(textBuf, px, ly);
            map.setTextDatum(lgfx::top_left);
            placedLabels.push_back({(int16_t)lx, (int16_t)ly, (int16_t)tw, (int16_t)th});
        }
        textRefs.clear();

        map.endWrite();

        // Free all tile buffers (IceNav-v3: maps.cpp:1574-1577)
        for (auto* buf : tileBuffers) free(buf);

        uint64_t endTime = esp_timer_get_time();
        Serial.printf("[NAV] Viewport: %llu ms (load %llu ms), %d features, PSRAM free: %u\n",
                      (endTime - startTime) / 1000, (loadEnd - startTime) / 1000,
                      totalFeatures, ESP.getFreePsram());

        return !tileBuffers.empty();
    }

    // Render a NAV1 vector tile using IceNav-v3 patterns:
    // - 16 priority layers (dispatch by low nibble of zoomPriority)
    // - Mixed geometry types per layer (not separated by type)
    // - Per-feature setClipRect to tile boundaries
    // - startWrite()/endWrite() for SPI batching
    // - taskYIELD() between priority layers
    // - ringCount read as uint8_t (1 byte, matching Python tile_generator.py)
    static bool renderNavTile(const char* path, int16_t xOffset, int16_t yOffset, LGFX_Sprite &map, uint8_t currentZoom) {
        esp_task_wdt_reset();
        uint8_t* data = nullptr;
        size_t fileSize = 0;

        // Read tile file under SPI mutex
        if (spiMutex != NULL && xSemaphoreTakeRecursive(spiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            File file = SD.open(path, FILE_READ);
            if (file) {
                fileSize = file.size();
                if (fileSize >= 22) {
                    data = (uint8_t*)ps_malloc(fileSize);
                    if (data) {
                        file.read(data, fileSize);
                    } else {
                        Serial.println("[MAP] Failed to allocate memory for tile");
                    }
                }
                file.close();
            } else {
                Serial.printf("[MAP] Failed to open tile: %s\n", path);
            }
            xSemaphoreGiveRecursive(spiMutex);
        }

        if (!data) return false;

        if (memcmp(data, "NAV1", 4) != 0) {
            free(data);
            Serial.printf("[MAP] Invalid NAV1 magic for tile: %s\n", path);
            return false;
        }

        uint16_t feature_count;
        memcpy(&feature_count, data + 4, 2);

        map.fillSprite(map.color565(0xF2, 0xEF, 0xE9));  // OSM-style beige background

        // --- Dispatch features to 16 priority layers (IceNav-v3 pattern) ---
        for (int i = 0; i < 16; i++) globalLayers[i].clear();
        std::vector<FeatureRef, PSRAMAllocator<FeatureRef>> textRefs;
        textRefs.reserve(32);

        uint8_t* p = data + 22;
        for (uint16_t i = 0; i < feature_count; i++) {
            if ((i & 63) == 0) esp_task_wdt_reset();
            if (p + 12 > data + fileSize) break;

            uint8_t geomType = p[0];
            uint8_t zoomPriority = p[3];
            uint16_t coordCount;
            memcpy(&coordCount, p + 9, 2);

            uint32_t feature_data_size = coordCount * 4;
            uint16_t ringCount = 0;

            // Polygon ring data: uint16 ringCount (2 bytes) + ringCount × uint16 ringEnds
            // (matching Python tile_generator.py: struct.pack('<H', ring_count))
            if (geomType == 3) {
                uint8_t* ring_ptr = p + 12 + feature_data_size;
                if (ring_ptr + 2 <= data + fileSize) {
                    memcpy(&ringCount, ring_ptr, 2);
                    if (ring_ptr + 2 + (ringCount * 2) <= data + fileSize) {
                        feature_data_size += 2 + (ringCount * 2);
                    } else {
                        ringCount = 0;
                    }
                }
            }

            if (p + 12 + feature_data_size > data + fileSize) break;

            // Zoom filtering (IceNav-v3 NavReader pattern):
            // High nibble = minZoom required. Skip features too detailed for current zoom.
            uint8_t minZoom = zoomPriority >> 4;
            if (minZoom > currentZoom) {
                p += 12 + feature_data_size;
                continue;
            }

            // Dispatch to priority layer (low nibble, IceNav-v3 getPriority() pattern)
            uint8_t priority = zoomPriority & 0x0F;
            if (priority >= 16) priority = 15;

            FeatureRef ref;
            ref.ptr = p;
            ref.geomType = geomType;
            ref.ringCount = ringCount;
            ref.coordCount = coordCount;
            if (geomType == 4)
                textRefs.push_back(ref);
            else
                globalLayers[priority].push_back(ref);

            p += 12 + feature_data_size;
        }

        // --- Render all layers (IceNav-v3 pattern) ---
        map.startWrite();

        int featureCount = 0;
        for (int pri = 0; pri < 16; pri++) {
            if (globalLayers[pri].empty()) continue;

            for (const auto& ref : globalLayers[pri]) {
                // WDT reset every 32 features during rendering (prevents WDT timeout)
                if ((++featureCount & 31) == 0) esp_task_wdt_reset();

                uint8_t* fp = ref.ptr;

                // Per-feature setClipRect to tile boundaries (IceNav-v3 pattern)
                map.setClipRect(0, 0, MAP_TILE_SIZE, MAP_TILE_SIZE);

                // BBox culling against tile
                uint8_t bx1 = fp[5], by1 = fp[6], bx2 = fp[7], by2 = fp[8];
                if ((int)bx2 + xOffset < 0 || (int)bx1 + xOffset > MAP_TILE_SIZE - 1 ||
                    (int)by2 + yOffset < 0 || (int)by1 + yOffset > MAP_TILE_SIZE - 1) continue;

                // Read colorRgb565 directly (LE, no byte swap — IceNav-v3 pattern)
                uint16_t colorRgb565;
                memcpy(&colorRgb565, fp + 1, 2);

                // Render feature by geometry type (mixed per layer, IceNav-v3 pattern)
                switch (ref.geomType) {
                    case 3: { // Polygon
                        if (ref.coordCount < 3) break;
                        int16_t* coords = (int16_t*)(fp + 12);
                        // ringEnds starts after 1-byte ringCount (not 2)
                        uint16_t* ringEnds = (ref.ringCount > 0)
                            ? (uint16_t*)(fp + 12 + ref.coordCount * 4 + 2)
                            : nullptr;

                        if (proj32X.capacity() < ref.coordCount) proj32X.reserve(ref.coordCount * 3 / 2);
                        if (proj32Y.capacity() < ref.coordCount) proj32Y.reserve(ref.coordCount * 3 / 2);
                        proj32X.resize(ref.coordCount);
                        proj32Y.resize(ref.coordCount);

                        int* px_hp = proj32X.data();
                        int* py_hp = proj32Y.data();

                        for (uint16_t j = 0; j < ref.coordCount; j++) {
                            px_hp[j] = std::max(0, std::min(4096, (int)coords[j * 2]));
                            py_hp[j] = std::max(0, std::min(4096, (int)coords[j * 2 + 1]));
                        }

                        if (fillPolygons) {
                            fillPolygonGeneral(map, px_hp, py_hp, ref.coordCount,
                                colorRgb565, xOffset, yOffset, ref.ringCount, ringEnds);
                        }

                        // Building outline (bit 7 of fp[4])
                        if ((fp[4] & 0x80) != 0) {
                            uint16_t outlineColor = darkenRGB565(colorRgb565, 0.35f);
                            uint16_t ringStart = 0;
                            uint16_t numRings = (ref.ringCount > 0) ? ref.ringCount : 1;
                            for (uint16_t r = 0; r < numRings; r++) {
                                uint16_t ringEnd = (ringEnds && r < ref.ringCount) ? ringEnds[r] : ref.coordCount;
                                if (ringEnd > ref.coordCount) ringEnd = ref.coordCount;
                                for (uint16_t j = ringStart; j < ringEnd; j++) {
                                    uint16_t next = (j + 1 < ringEnd) ? j + 1 : ringStart;
                                    int x0 = (px_hp[j] >> 4) + xOffset;
                                    int y0 = (py_hp[j] >> 4) + yOffset;
                                    int x1 = (px_hp[next] >> 4) + xOffset;
                                    int y1 = (py_hp[next] >> 4) + yOffset;
                                    map.drawLine(x0, y0, x1, y1, outlineColor);
                                }
                                ringStart = ringEnd;
                            }
                        }

                        break;
                    }
                    case 2: { // LineString (IceNav-v3: renderNavLineString with dedup + bbox)
                        if (ref.coordCount < 2) break;
                        bool hasCasing = (fp[4] & 0x80) != 0;  // bit 7 = casing flag
                        uint8_t widthPixels = fp[4] & 0x7F;     // bits 6-0 = actual width
                        if (widthPixels == 0) widthPixels = 1;
                        int16_t* coords = (int16_t*)(fp + 12);

                        // Pre-project with dedup (IceNav-v3 pattern)
                        size_t numCoords = ref.coordCount;
                        if (proj16X.capacity() < numCoords) proj16X.reserve(numCoords * 3 / 2);
                        if (proj16Y.capacity() < numCoords) proj16Y.reserve(numCoords * 3 / 2);
                        proj16X.resize(numCoords);
                        proj16Y.resize(numCoords);

                        int16_t* pxArr = proj16X.data();
                        int16_t* pyArr = proj16Y.data();

                        int16_t minPx = INT16_MAX, maxPx = INT16_MIN;
                        int16_t minPy = INT16_MAX, maxPy = INT16_MIN;
                        size_t validPoints = 0;
                        int16_t lastPx = -32768, lastPy = -32768;

                        // Simplification: skip vertices within widthPixels/2 of last kept
                        int distThreshSq2 = (widthPixels * widthPixels) / 4;
                        if (distThreshSq2 < 1) distThreshSq2 = 1;

                        for (size_t j = 0; j < numCoords; j++) {
                            int16_t px = (coords[j * 2] >> 4) + xOffset;
                            int16_t py = (coords[j * 2 + 1] >> 4) + yOffset;
                            if (validPoints > 0) {
                                int dx = px - lastPx;
                                int dy = py - lastPy;
                                if ((dx*dx + dy*dy) < distThreshSq2 && j < numCoords - 1) continue;
                            }
                            pxArr[validPoints] = px;
                            pyArr[validPoints] = py;
                            if (px < minPx) minPx = px;
                            if (px > maxPx) maxPx = px;
                            if (py < minPy) minPy = py;
                            if (py > maxPy) maxPy = py;
                            lastPx = px;
                            lastPy = py;
                            validPoints++;
                        }

                        if (validPoints < 2 || maxPx < 0 || minPx >= MAP_TILE_SIZE ||
                            maxPy < 0 || minPy >= MAP_TILE_SIZE) break;

                        for (size_t j = 1; j < validPoints; j++) {
                            if (widthPixels <= 2) {
                                map.drawLine(pxArr[j-1], pyArr[j-1], pxArr[j], pyArr[j], colorRgb565);
                            } else {
                                map.drawWideLine(pxArr[j-1], pyArr[j-1], pxArr[j], pyArr[j],
                                                 widthPixels, colorRgb565);
                                map.setClipRect(xOffset, yOffset, MAP_TILE_SIZE, MAP_TILE_SIZE);
                            }
                        }
                        break;
                    }
                    case 1: { // Point (IceNav-v3: renderNavPoint with bounds check)
                        if (ref.coordCount < 1) break;
                        int16_t* coords = (int16_t*)(fp + 12);
                        int px = (coords[0] >> 4) + xOffset;
                        int py = (coords[1] >> 4) + yOffset;
                        if (px >= 0 && px < MAP_TILE_SIZE && py >= 0 && py < MAP_TILE_SIZE)
                            map.fillCircle(px, py, 3, colorRgb565);
                        break;
                    }
                    case 4: { // Text label (GEOM_TEXT)
                        uint8_t fontSize = fp[4];
                        int16_t* coords = (int16_t*)(fp + 12);
                        int px = (coords[0] >> 4) + xOffset;
                        int py = (coords[1] >> 4) + yOffset;
                        uint8_t textLen = *(fp + 12 + 4);
                        if (textLen > 0 && textLen < 128) {
                            char textBuf[128];
                            memcpy(textBuf, fp + 12 + 5, textLen);
                            textBuf[textLen] = '\0';

                            // Use VLW Unicode font if loaded, fallback to GFX font
                            if (vlwFontLoaded) {
                                map.setFont(&vlwFont);
                                // Scale VLW font based on fontSize (0=small, 1=medium, 2=large)
                                float scale = (fontSize == 0) ? 0.8f : (fontSize == 1) ? 1.0f : 1.2f;
                                map.setTextSize(scale);
                            } else {
                                map.setFont((lgfx::GFXfont*)&OpenSans_Bold6pt7b);
                                map.setTextSize(1);
                            }
                            map.setTextColor(colorRgb565);
                            map.drawString(textBuf, px, py);
                        }
                        break;
                    }
                }
            }
            globalLayers[pri].clear();
            taskYIELD();
        }

        // Label pass — on top of all geometry
        map.clearClipRect();
        if (vlwFontLoaded) {
            map.setFont(&vlwFont);
            map.setTextSize(1.0f);
        } else {
            map.setFont((lgfx::GFXfont*)&OpenSans_Bold6pt7b);
            map.setTextSize(1);
        }
        for (const auto& ref : textRefs) {
            uint8_t* fp = ref.ptr;
            uint16_t colorRgb565;
            memcpy(&colorRgb565, fp + 1, 2);
            int16_t* coords = (int16_t*)(fp + 12);
            int px = (coords[0] >> 4) + xOffset;
            int py = (coords[1] >> 4) + yOffset;
            uint8_t fontSize = fp[4];
            uint8_t textLen = *(fp + 12 + 4);
            if (textLen == 0 || textLen >= 128) continue;
            char textBuf[128];
            memcpy(textBuf, fp + 12 + 5, textLen);
            textBuf[textLen] = '\0';
            if (vlwFontLoaded) {
                float scale = (fontSize == 0) ? 0.8f : (fontSize == 1) ? 1.0f : 1.2f;
                map.setTextSize(scale);
            }
            map.setTextColor(colorRgb565);
            map.drawString(textBuf, px, py);
        }

        map.endWrite();

        free(data);
        return true;
    }

    // Public render dispatcher
    bool renderTile(const char* path, int16_t xOffset, int16_t yOffset, LGFX_Sprite &map, uint8_t zoom) {
        String pathStr(path);
        if (pathStr.endsWith(".nav")) {
            return renderNavTile(path, xOffset, yOffset, map, zoom);
        } else if (pathStr.endsWith(".png")) {
            return renderPNGRaster(path, map);
        } else if (pathStr.endsWith(".jpg")) {
            return renderJPGRaster(path, map);
        }
        Serial.printf("[MAP] Unknown tile type for path: %s\n", path);
        return false;
    }

}
#endif
