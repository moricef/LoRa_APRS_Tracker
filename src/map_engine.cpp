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
#include <SD.h>
#include <esp_task_wdt.h>
#include <algorithm>
#include <climits>

// Global sprite pointer for raster decoder callbacks
static LGFX_Sprite* targetSprite_ = nullptr;

namespace MapEngine {

    // Handles for the asynchronous rendering system
    QueueHandle_t mapRenderQueue = nullptr;
    SemaphoreHandle_t spriteMutex = nullptr;
    static TaskHandle_t mapRenderTaskHandle = nullptr;
    static lv_obj_t* canvas_to_invalidate_ = nullptr;

    // Static vectors for AEL polygon filler (internal RAM for speed)
    static std::vector<UIMapManager::Edge, InternalAllocator<UIMapManager::Edge>> edgePool;
    static std::vector<int, InternalAllocator<int>> edgeBuckets;

    // Static vectors for coordinate projection (internal RAM, pre-reserved)
    static std::vector<int, InternalAllocator<int>> proj32X;
    static std::vector<int, InternalAllocator<int>> proj32Y;

    // Feature index entry for priority-sorted rendering with BBox culling
    struct FeatureIndex {
        uint32_t offset;        // Byte offset from start of data
        uint32_t dataSize;      // Total feature data size (after header)
        uint8_t geomType;       // 1=Point, 2=Line, 3=Polygon
        uint8_t priority;       // zoomPriority (0=background, 15=foreground)
        uint8_t ringCount;      // For polygons
    };
    static std::vector<FeatureIndex, InternalAllocator<FeatureIndex>> featureIdx;

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
                        success = renderTile(request.path, request.xOffset, request.yOffset, *request.targetSprite);
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
            8192,
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

        // Pre-reserve AEL, projection, and feature index buffers in internal RAM
        // This avoids repeated allocations during rendering
        edgePool.reserve(512);
        edgeBuckets.reserve(256);
        proj32X.reserve(1024);
        proj32Y.reserve(1024);
        featureIdx.reserve(512);

        Serial.printf("[MAP] Tile cache initialized with %d tiles capacity\n", maxCachedTiles);
        Serial.printf("[MAP] Render buffers pre-reserved (internal RAM)\n");
    }

    void clearTileCache() {
        initTileCache();
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

                // Use memcpy for fast row-by-row buffer copy
                for (int y = 0; y < copy_h; y++) {
                    uint16_t* src_ptr = src_buf + ((src_y + y) * MAP_TILE_SIZE) + src_x;
                    lv_color_t* dest_ptr = dest_buf + ((dest_y + y) * MAP_CANVAS_WIDTH) + dest_x;
                    memcpy(dest_ptr, src_ptr, copy_w * sizeof(lv_color_t));
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

    void fillPolygonGeneral(LGFX_Sprite &map, const int *px_hp, const int *py_hp, const int numPoints, const uint16_t color, const int xOffset, const int yOffset, uint16_t ringCount, uint16_t* ringEnds)
    {
        // This AEL implementation takes high-precision (HP) coordinates (0-4096) and iterates over pixel-space scanlines (0-255)
        if (numPoints < 3) return;

        // 1. Find Y bounds in pixel space
        int minY_px = INT_MAX, maxY_px = INT_MIN;
        for (int i = 0; i < numPoints; i++) {
            int y_px = py_hp[i] >> 4;
            if (y_px < minY_px) minY_px = y_px;
            if (y_px > maxY_px) maxY_px = y_px;
        }

        if (maxY_px < 0 || minY_px >= MAP_TILE_SIZE) return;

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

                if (y1_px == y2_px) continue; // Skip horizontal lines in pixel space

                UIMapManager::Edge e;
                e.nextActive = -1;

                if (y1_hp < y2_hp) {
                    e.yMax = y2_px; // Max Y in pixels
                    e.slope = ((int64_t)(x2_hp - x1_hp) << 16) / (y2_hp - y1_hp); // HP slope
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
        // Loop iterates over PIXEL scanlines, clipped to the sprite
        int startY_px = std::max(minY_px, 0);
        int endY_px = std::min(maxY_px, MAP_TILE_SIZE - 1);
        
        uint32_t lastYieldMs = millis();

        for (int y_px = startY_px; y_px <= endY_px; y_px++) {
            uint32_t now = millis();
            if (now - lastYieldMs > 20) { // Yield every 20ms of continuous work
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(1)); // Force context switch
                lastYieldMs = now;
            }

            // 4. Add new edges from bucket to Active Edge List (AEL)
            if (y_px - minY_px >= 0 && (size_t)(y_px - minY_px) < edgeBuckets.size()) {
                int eIdx = edgeBuckets[y_px - minY_px];
                while (eIdx != -1) {
                    int nextIdx = edgePool[eIdx].nextInBucket;
                    edgePool[eIdx].nextActive = activeHead;
                    activeHead = eIdx;
                    eIdx = nextIdx;
                }
            }

            // 5. Remove finished edges from AEL
            int* pCurrIdx = &activeHead;
            while (*pCurrIdx != -1) {
                if (edgePool[*pCurrIdx].yMax <= y_px) {
                    *pCurrIdx = edgePool[*pCurrIdx].nextActive;
                } else {
                    pCurrIdx = &(edgePool[*pCurrIdx].nextActive);
                }
            }

            if (activeHead == -1) continue;

            // 6. Sort AEL by xVal
            int sorted = -1;
            int active = activeHead;
            while (active != -1) {
                int nextActive = edgePool[active].nextActive;
                if (sorted == -1 || edgePool[active].xVal < edgePool[sorted].xVal) {
                    edgePool[active].nextActive = sorted;
                    sorted = active;
                } else {
                    int s = sorted;
                    while (edgePool[s].nextActive != -1 && edgePool[edgePool[s].nextActive].xVal < edgePool[active].xVal) {
                        s = edgePool[s].nextActive;
                    }
                    edgePool[active].nextActive = edgePool[s].nextActive;
                    edgePool[s].nextActive = active;
                }
                active = nextActive;
            }
            activeHead = sorted;

            // 7. Draw horizontal spans
            int left = activeHead;
            while (left != -1 && edgePool[left].nextActive != -1) {
                int right = edgePool[left].nextActive;
                
                // Scale HP fixed-point xVal down to pixels JUST before drawing
                int xStart = (edgePool[left].xVal >> 16) >> 4;
                int xEnd = (edgePool[right].xVal >> 16) >> 4;
                
                if (xEnd > xStart) {
                    map.drawFastHLine(xStart + xOffset, y_px + yOffset, xEnd - xStart, color);
                }
                left = edgePool[right].nextActive;
            }
            
            // 8. Update xVal for next PIXEL scanline (step is 16 HP units)
            for (int a = activeHead; a != -1; a = edgePool[a].nextActive) {
                edgePool[a].xVal += (int64_t)edgePool[a].slope * 16;
            }
        }
    }

    static bool renderNavTile(const char* path, int16_t xOffset, int16_t yOffset, LGFX_Sprite &map) {
        esp_task_wdt_reset();
        uint8_t* data = nullptr;
        size_t fileSize = 0;

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

        if (!data) {
            return false;
        }

        if (memcmp(data, "NAV1", 4) != 0) {
            free(data);
            Serial.printf("[MAP] Invalid NAV1 magic for tile: %s\n", path);
            return false;
        }

        uint16_t feature_count;
        memcpy(&feature_count, data + 4, 2);

        map.fillSprite(TFT_WHITE);
        map.setClipRect(0, 0, 256, 256);

        // --- Build feature index with BBox culling ---
        // Pre-scan all features: parse headers, skip features outside viewport
        featureIdx.clear();
        if (featureIdx.capacity() < feature_count) featureIdx.reserve(feature_count);

        uint8_t* p = data + 22;
        for (uint16_t i = 0; i < feature_count; i++) {
            if (p + 12 > data + fileSize) break;

            uint8_t geomType = p[0];
            uint8_t zoomPriority = p[3];
            uint16_t coordCount;
            memcpy(&coordCount, p + 9, 2);

            uint32_t feature_data_size = coordCount * 4;
            uint8_t ringCount = 0;

            if (geomType == 3) {
                uint8_t* ring_ptr = p + 12 + feature_data_size;
                if (ring_ptr + 1 <= data + fileSize) {
                    ringCount = ring_ptr[0];
                    if (ring_ptr + 1 + (ringCount * 2) <= data + fileSize) {
                        feature_data_size += 1 + (ringCount * 2);
                    } else {
                        ringCount = 0;
                    }
                }
            }

            if (p + 12 + feature_data_size > data + fileSize) break;

            // BBox culling: skip features entirely outside the 256x256 sprite viewport
            // bbox values are in pixel space (0-255) relative to tile origin
            uint8_t bx1 = p[5], by1 = p[6], bx2 = p[7], by2 = p[8];
            int fx1 = (int)bx1 + xOffset;
            int fy1 = (int)by1 + yOffset;
            int fx2 = (int)bx2 + xOffset;
            int fy2 = (int)by2 + yOffset;

            if (fx2 < 0 || fx1 > 255 || fy2 < 0 || fy1 > 255) {
                // Feature entirely outside viewport — skip
                p += 12 + feature_data_size;
                continue;
            }

            FeatureIndex fi;
            fi.offset = (uint32_t)(p - data);
            fi.dataSize = feature_data_size;
            fi.geomType = geomType;
            fi.priority = zoomPriority;
            fi.ringCount = ringCount;
            featureIdx.push_back(fi);

            p += 12 + feature_data_size;
        }

        // --- Sort features by priority (ascending: 0=background first) ---
        std::sort(featureIdx.begin(), featureIdx.end(),
            [](const FeatureIndex& a, const FeatureIndex& b) {
                return a.priority < b.priority;
            });

        uint32_t lastYieldMs = millis();

        // --- Pass 1: Polygons (sorted by priority, background first) ---
        for (const auto& fi : featureIdx) {
            if (fi.geomType != 3) continue;

            uint32_t now = millis();
            if (now - lastYieldMs > 20) {
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(1));
                lastYieldMs = now;
            }

            uint8_t* fp = data + fi.offset;
            uint16_t coordCount;
            memcpy(&coordCount, fp + 9, 2);

            if (coordCount < 3) continue;

            uint16_t rawColor;
            memcpy(&rawColor, fp + 1, 2);
            uint16_t colorRgb565 = (rawColor << 8) | (rawColor >> 8);
            int16_t* coords = (int16_t*)(fp + 12);
            uint16_t* ringEnds = (fi.ringCount > 0) ? (uint16_t*)(fp + 12 + coordCount * 4 + 2) : nullptr;

            if (proj32X.capacity() < coordCount) proj32X.reserve(coordCount);
            if (proj32Y.capacity() < coordCount) proj32Y.reserve(coordCount);
            proj32X.resize(coordCount);
            proj32Y.resize(coordCount);

            int* px_hp = proj32X.data();
            int* py_hp = proj32Y.data();

            for (uint16_t j = 0; j < coordCount; j++) {
                px_hp[j] = coords[j * 2];
                py_hp[j] = coords[j * 2 + 1];
            }

            if (fillPolygons) {
                fillPolygonGeneral(map, px_hp, py_hp, coordCount, colorRgb565, xOffset, yOffset, fi.ringCount, ringEnds);
            }

            uint16_t outerRingEnd = (fi.ringCount > 0) ? ringEnds[0] : coordCount;
            if (outerRingEnd >= 2) {
                uint16_t borderColor = darkenRGB565(colorRgb565, 0.15f);
                for (int k = 0; k < outerRingEnd; k++) {
                    int next = (k + 1 == outerRingEnd) ? 0 : k + 1;
                    int x0 = (px_hp[k] >> 4) + xOffset;
                    int y0 = (py_hp[k] >> 4) + yOffset;
                    int x1 = (px_hp[next] >> 4) + xOffset;
                    int y1 = (py_hp[next] >> 4) + yOffset;
                    map.drawLine(x0, y0, x1, y1, borderColor);
                }
            }
        }

        // --- Pass 2: Lines (sorted by priority, minor roads first) ---
        for (const auto& fi : featureIdx) {
            if (fi.geomType != 2) continue;

            uint32_t now = millis();
            if (now - lastYieldMs > 20) {
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(1));
                lastYieldMs = now;
            }

            uint8_t* fp = data + fi.offset;
            uint16_t coordCount;
            memcpy(&coordCount, fp + 9, 2);

            if (coordCount < 2) continue;

            uint16_t rawColor;
            memcpy(&rawColor, fp + 1, 2);
            uint16_t colorRgb565 = (rawColor << 8) | (rawColor >> 8);
            uint8_t widthPixels = fp[4];
            int16_t* coords = (int16_t*)(fp + 12);

            for (uint16_t j = 1; j < coordCount; j++) {
                int x0 = (coords[(j - 1) * 2] >> 4) + xOffset;
                int y0 = (coords[(j - 1) * 2 + 1] >> 4) + yOffset;
                int x1 = (coords[j * 2] >> 4) + xOffset;
                int y1 = (coords[j * 2 + 1] >> 4) + yOffset;

                if (widthPixels <= 1) {
                    map.drawLine(x0, y0, x1, y1, colorRgb565);
                } else {
                    map.drawWideLine(x0, y0, x1, y1, widthPixels, colorRgb565);
                }
            }
        }

        // --- Pass 3: Points (on top of everything) ---
        for (const auto& fi : featureIdx) {
            if (fi.geomType != 1) continue;

            uint8_t* fp = data + fi.offset;
            uint16_t coordCount;
            memcpy(&coordCount, fp + 9, 2);

            if (coordCount < 1) continue;

            uint16_t rawColor;
            memcpy(&rawColor, fp + 1, 2);
            uint16_t colorRgb565 = (rawColor << 8) | (rawColor >> 8);
            int16_t* coords = (int16_t*)(fp + 12);
            int px = (coords[0] >> 4) + xOffset;
            int py = (coords[1] >> 4) + yOffset;
            map.fillCircle(px, py, 3, colorRgb565);
        }

        map.clearClipRect();
        free(data);
        return true;
    }

    // Public render dispatcher
    bool renderTile(const char* path, int16_t xOffset, int16_t yOffset, LGFX_Sprite &map) {
        String pathStr(path);
        if (pathStr.endsWith(".nav")) {
            return renderNavTile(path, xOffset, yOffset, map);
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
