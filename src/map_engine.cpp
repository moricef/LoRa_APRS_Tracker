/* Map rendering engine for T-Deck Plus
 * Handles vector tile parsing, rendering, and caching.
 */
#ifdef USE_LVGL_UI

#include "map_engine.h"
#include "ui_map_manager.h"
#include "storage_utils.h"
#include <SD.h>
#include <esp_task_wdt.h>
#include <algorithm>
#include <climits>

namespace MapEngine {

    // Handles for the asynchronous rendering system
    QueueHandle_t mapRenderQueue = nullptr;
    SemaphoreHandle_t spriteMutex = nullptr;
    static TaskHandle_t mapRenderTaskHandle = nullptr;
    static lv_obj_t* canvas_to_invalidate_ = nullptr;

    // Static vectors for AEL polygon filler
    static std::vector<UIMapManager::Edge> edgePool;
    static std::vector<int> edgeBuckets;

    // Static vectors for coordinate projection (to avoid re-allocation)
    static std::vector<int> proj32X;
    static std::vector<int> proj32Y;

    // Tile cache system
    #define TILE_CACHE_SIZE 15  // Number of tiles to cache
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

    // Background task to render map tiles on Core 0
    static void mapRenderTask(void* param) {
        RenderRequest request;
        Serial.println("[MAP] Render task started on Core 0");

        while (true) {
            if (xQueueReceive(mapRenderQueue, &request, portMAX_DELAY) == pdTRUE) {
                if (request.targetSprite) {
                    // Lock the sprite for drawing
                    if (xSemaphoreTake(spriteMutex, portMAX_DELAY) == pdTRUE) {
                        renderTile(request.path, request.xOffset, request.yOffset, *request.targetSprite);
                        xSemaphoreGive(spriteMutex);
                    }
                    // Request a redraw on the LVGL thread
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
            4096,
            NULL,
            1, // Low priority
            &mapRenderTaskHandle,
            0  // Core 0
        );
    }

    // Initialize tile cache
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
        Serial.printf("[MAP] Tile cache initialized with %d tiles capacity\n", maxCachedTiles);
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
                uint16_t* fb = (uint16_t*)sprite->getBuffer();
                for (int y = 0; y < copy_h; y++) {
                    uint16_t* src_row_ptr = fb + ((src_y + y) * MAP_TILE_SIZE) + src_x;
                    lv_canvas_copy_buf(canvas, src_row_ptr, dest_x, dest_y + y, copy_w, 1);
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

    void fillPolygonGeneral(LGFX_Sprite &map, const int *px, const int *py, const int numPoints, const uint16_t color, const int xOffset, const int yOffset, uint16_t ringCount, uint16_t* ringEnds)
    {
        if (numPoints < 3) return;

        int minY = INT_MAX, maxY = INT_MIN;
        for (int i = 0; i < numPoints; i++) {
            if (py[i] < minY) minY = py[i];
            if (py[i] > maxY) maxY = py[i];
        }

        if (maxY < 0 || minY >= MAP_TILE_SIZE) return;

        edgePool.clear();
        int bucketCount = maxY - minY + 1;
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

            for (int i = 0; i < ringNumPoints; i++) {
                int next = (i + 1) % ringNumPoints;
                int x1 = px[ringStart + i], y1 = py[ringStart + i];
                int x2 = px[ringStart + next], y2 = py[ringStart + next];
                if (y1 == y2) continue;

                UIMapManager::Edge e;
                e.nextActive = -1;
                if (y1 < y2) {
                    e.yMax = y2;
                    e.xVal = x1 << 16;
                    e.slope = ((x2 - x1) << 16) / (y2 - y1);
                    if (y1 - minY >= 0 && y1 - minY < edgeBuckets.size()) {
                        e.nextInBucket = edgeBuckets[y1 - minY];
                        edgePool.push_back(e);
                        edgeBuckets[y1 - minY] = edgePool.size() - 1;
                    }
                } else {
                    e.yMax = y1;
                    e.xVal = x2 << 16;
                    e.slope = ((x1 - x2) << 16) / (y1 - y2);
                    if (y2 - minY >= 0 && y2 - minY < edgeBuckets.size()) {
                        e.nextInBucket = edgeBuckets[y2 - minY];
                        edgePool.push_back(e);
                        edgeBuckets[y2 - minY] = edgePool.size() - 1;
                    }
                }
            }
            ringStart = ringEnd;
        }

        int activeHead = -1;
        int startY = std::max(minY, -yOffset);
        int endY = std::min(maxY, MAP_TILE_SIZE - 1 - yOffset);

        if (startY > minY) {
            for (int y = minY; y < startY; y++) {
                if (y - minY < 0 || y - minY >= edgeBuckets.size()) continue;
                int eIdx = edgeBuckets[y - minY];
                while (eIdx != -1) {
                    int nextIdx = edgePool[eIdx].nextInBucket;
                    edgePool[eIdx].xVal += edgePool[eIdx].slope * (startY - y);
                    edgePool[eIdx].nextActive = activeHead;
                    activeHead = eIdx;
                    eIdx = nextIdx;
                }
            }
            int* pCurrIdx = &activeHead;
            while (*pCurrIdx != -1) {
                if (edgePool[*pCurrIdx].yMax <= startY) {
                    *pCurrIdx = edgePool[*pCurrIdx].nextActive;
                } else {
                    pCurrIdx = &(edgePool[*pCurrIdx].nextActive);
                }
            }
        }

        for (int y = startY; y <= endY; y++) {
            if ((y & 0x1F) == 0) { 
                esp_task_wdt_reset(); 
                vTaskDelay(pdMS_TO_TICKS(1)); 
            }

            if (y - minY >= 0 && y - minY < edgeBuckets.size()) {
                int eIdx = edgeBuckets[y - minY];
                while (eIdx != -1) {
                    int nextIdx = edgePool[eIdx].nextInBucket;
                    edgePool[eIdx].nextActive = activeHead;
                    activeHead = eIdx;
                    eIdx = nextIdx;
                }
            }

            int* pCurrIdx = &activeHead;
            while (*pCurrIdx != -1) {
                if (edgePool[*pCurrIdx].yMax <= y) {
                    *pCurrIdx = edgePool[*pCurrIdx].nextActive;
                } else {
                    pCurrIdx = &(edgePool[*pCurrIdx].nextActive);
                }
            }

            if (activeHead == -1) continue;

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

            int yy = y + yOffset;
            int left = activeHead;
            while (left != -1 && edgePool[left].nextActive != -1) {
                int right = edgePool[left].nextActive;
                int xStart = (edgePool[left].xVal >> 16) + xOffset;
                int xEnd = (edgePool[right].xVal >> 16) + xOffset;
                if (xStart < 0) xStart = 0;
                if (xEnd > MAP_TILE_SIZE) xEnd = MAP_TILE_SIZE;
                if (xEnd > xStart) {
                    map.drawFastHLine(xStart, yy, xEnd - xStart, color);
                }
                left = edgePool[right].nextActive;
            }
            
            for (int a = activeHead; a != -1; a = edgePool[a].nextActive) {
                edgePool[a].xVal += edgePool[a].slope;
            }
        }
    }

    bool renderTile(const char* path, int16_t xOffset, int16_t yOffset, LGFX_Sprite &map) {
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

        uint32_t last_wdt_ms = millis();
        
        uint8_t* p = data + 22;
        for (uint16_t i = 0; i < feature_count; i++) {
            if (millis() - last_wdt_ms > 100) {
                esp_task_wdt_reset();
                yield();
                last_wdt_ms = millis();
            }
            if (p + 12 > data + fileSize) break;

            uint8_t geomType = p[0];
            uint16_t coordCount; 
            memcpy(&coordCount, p + 9, 2);
            
            uint32_t feature_data_size = coordCount * 4;
            uint16_t ringCount = 0;

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

            if (geomType == 3 && coordCount >= 3) {
                uint16_t colorRgb565;
                memcpy(&colorRgb565, p + 1, 2);
                int16_t* coords = (int16_t*)(p + 12);
                uint16_t* ringEnds = (ringCount > 0) ? (uint16_t*)(p + 12 + coordCount * 4 + 2) : nullptr;

                if (proj32X.capacity() < coordCount) proj32X.reserve(coordCount);
                if (proj32Y.capacity() < coordCount) proj32Y.reserve(coordCount);
                proj32X.resize(coordCount);
                proj32Y.resize(coordCount);

                int* px = proj32X.data();
                int* py = proj32Y.data();

                for (uint16_t j = 0; j < coordCount; j++) {
                    px[j] = (coords[j * 2] >> 4) + xOffset;
                    py[j] = (coords[j * 2 + 1] >> 4) + yOffset;
                }
            
                if (fillPolygons) {
                    fillPolygonGeneral(map, px, py, coordCount, colorRgb565, 0, 0, ringCount, ringEnds);
                }

                uint16_t outerRingEnd = (ringCount > 0) ? ringEnds[0] : coordCount;
                if (outerRingEnd >= 2) {
                    uint16_t borderColor = darkenRGB565(colorRgb565, 0.15f);
                    for (int k = 0; k < outerRingEnd; k++) {
                        int next = (k + 1 == outerRingEnd) ? 0 : k + 1;
                        map.drawLine(px[k], py[k], px[next], py[next], borderColor);
                    }
                }
            }
            p += 12 + feature_data_size;
        }

        p = data + 22;
        for (uint16_t i = 0; i < feature_count; i++) {
            if (millis() - last_wdt_ms > 100) { 
                esp_task_wdt_reset(); 
                yield(); 
                last_wdt_ms = millis(); 
            }
            if (p + 12 > data + fileSize) break;

            uint8_t geomType = p[0];
            uint16_t coordCount; 
            memcpy(&coordCount, p + 9, 2);

            uint32_t feature_data_size = coordCount * 4;
            if (geomType == 3) {
                uint16_t ringCount = 0;
                uint8_t* ring_ptr = p + 12 + feature_data_size;
                if (ring_ptr + 2 <= data + fileSize) {
                    memcpy(&ringCount, ring_ptr, 2);
                     if (ring_ptr + 2 + (ringCount * 2) <= data + fileSize) {
                        feature_data_size += 2 + (ringCount * 2);
                    }
                }
            }
            
            if (p + 12 + feature_data_size > data + fileSize) break;

            if (geomType == 2 && coordCount >= 2) {
                uint16_t colorRgb565; 
                memcpy(&colorRgb565, p + 1, 2);
                uint8_t widthPixels = p[4];
                int16_t* coords = (int16_t*)(p + 12);
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
            } else if (geomType == 1 && coordCount > 0) {
                 uint16_t colorRgb565; 
                 memcpy(&colorRgb565, p + 1, 2);
                 int16_t* coords = (int16_t*)(p + 12);
                 int px = (coords[0] >> 4) + xOffset;
                 int py = (coords[1] >> 4) + yOffset;
                 map.fillCircle(px, py, 3, colorRgb565);
            }

            p += 12 + feature_data_size;
        }

        map.clearClipRect();
        free(data);
        return true;
    }
}
#endif
