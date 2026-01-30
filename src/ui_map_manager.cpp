/* Map logic for T-Deck Plus
 * Offline map tile display with stations using LVGL
 */

#ifdef USE_LVGL_UI

#include <Arduino.h>
#include <FS.h>
#include <lvgl.h>
#include <TFT_eSPI.h> // For TFT_eSPI definitions if needed (e.g. for SCREEN_WIDTH/HEIGHT)
#include <TinyGPS++.h>
#include <JPEGDEC.h>
// Undefine macros that conflict between PNGdec and JPEGDEC
#undef INTELSHORT
#undef INTELLONG
#undef MOTOSHORT
#undef MOTOLONG
#include <PNGdec.h>
#include <SD.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <vector>

#include "ui_map_manager.h"
#include "configuration.h"
#include "station_utils.h"
#include "utils.h"
#include "storage_utils.h"
#include "custom_characters.h" // For symbolsAPRS, SYMBOL_WIDTH, SYMBOL_HEIGHT
#include "lvgl_ui.h" // To call LVGL_UI::open_compose_with_callsign

namespace UIMapManager {


    // UI elements - Map screen
    lv_obj_t* screen_map = nullptr;
    lv_obj_t* map_canvas = nullptr;
    lv_color_t* map_canvas_buf = nullptr;
    lv_obj_t* map_title_label = nullptr;
    lv_obj_t* map_container = nullptr;

    // Map state variables
    static const int map_available_zooms[] = {8, 10, 12, 14}; // Available zoom levels (only levels with tiles on SD card)
    const int map_zoom_count = sizeof(map_available_zooms) / sizeof(map_available_zooms[0]);
    int map_zoom_index = 0;  // Index in map_available_zooms (starts at zoom 8)
    int map_current_zoom = map_available_zooms[0]; // Initialize with first available zoom
    float map_center_lat = 0.0f;
    float map_center_lon = 0.0f;
    String map_current_region = "";
    static String cachedMapsPath = "";      // Cached maps path (avoid repeated SD access)
    bool map_follow_gps = true;  // Follow GPS or free panning mode

    // Touch pan state
    static bool touch_dragging = false;
    static lv_coord_t touch_start_x = 0;
    static lv_coord_t touch_start_y = 0;
    static float drag_start_lat = 0.0f;
    static float drag_start_lon = 0.0f;
    static lv_coord_t last_pan_dx = 0;  // Track last pan delta for station sync
    static lv_coord_t last_pan_dy = 0;
    #define PAN_THRESHOLD 5  // Minimum pixels to trigger pan

    // Tile cache in PSRAM
    #define TILE_CACHE_SIZE 15  // Number of tiles to cache
    #define TILE_DATA_SIZE (MAP_TILE_SIZE * MAP_TILE_SIZE * sizeof(uint16_t))  // 128KB per tile for old raster tiles

    // Tile cache system
    static std::vector<CachedTile> tileCache;
    static size_t maxCachedTiles = TILE_CACHE_SIZE;
    static uint32_t cacheAccessCounter = 0;

    // Symbol cache in PSRAM
    #define SYMBOL_CACHE_SIZE 10  // Cache for frequently used symbols
    #define SYMBOL_SIZE 24        // 24x24 pixels
    #define SYMBOL_DATA_SIZE (SYMBOL_SIZE * SYMBOL_SIZE * sizeof(lv_color_t))

    static CachedSymbol symbolCache[SYMBOL_CACHE_SIZE];
    static uint32_t symbolCacheAccessCounter = 0;
    static bool symbolCacheInitialized = false;

    // Forward declarations
    void drawStationOnMap(lv_obj_t* canvas, int x, int y, const String& ssid, const char* aprsSymbol);

    // Station hit zones for click detection (replaces LVGL buttons - no alloc/dealloc)
    struct StationHitZone {
        int16_t x, y;      // Screen position (center)
        int16_t w, h;      // Hit zone size
        int8_t stationIdx; // Index in mapStations array (-1 = unused)
    };
    static StationHitZone stationHitZones[MAP_STATIONS_MAX];
    static int stationHitZoneCount = 0;

    // Station display objects pool (LVGL objects instead of canvas drawing)
    struct StationDisplayObj {
        lv_obj_t* container;  // Parent container for positioning
        lv_obj_t* icon;       // lv_img for APRS symbol
        lv_obj_t* icon_overlay; // Label for alphanumeric overlay (e.g., 'L', '1') or fallback char
        lv_obj_t* label;      // lv_label for callsign
        bool inUse;           // Currently displaying a station
    };
    #define STATION_POOL_SIZE (MAP_STATIONS_MAX + 1)  // +1 for own position at index 0
    static StationDisplayObj stationDisplayPool[STATION_POOL_SIZE];
    static bool stationDisplayPoolInitialized = false;

    // Periodic refresh timer for stations
    static lv_timer_t* map_refresh_timer = nullptr;
    #define MAP_REFRESH_INTERVAL 10000  // 10 seconds

    // Timer callback for periodic map refresh (stations update)
    static void map_refresh_timer_cb(lv_timer_t* timer) {
        if (screen_map && lv_scr_act() == screen_map) {
            Serial.println("[MAP] Periodic refresh (stations)");
            redraw_map_canvas();
        }
    }

    // ============ ASYNC TILE PRELOADING (Core 1) ============
    // Structure for tile preload request
    struct TileRequest {
        int tileX;
        int tileY;
        int zoom;
    };

    static QueueHandle_t tilePreloadQueue = nullptr;
    static TaskHandle_t tilePreloadTask = nullptr;
    static bool preloadTaskRunning = false;
    static volatile bool mainThreadLoading = false;  // Pause preload while main thread loads
    #define TILE_PRELOAD_QUEUE_SIZE 20

    // Forward declaration
    bool preloadTileToCache(int tileX, int tileY, int zoom);

    // Task running on Core 1 to preload tiles in background
    static void tilePreloadTaskFunc(void* param) {
        Serial.println("[MAP] Tile preload task started on Core 1");
        TileRequest req;

        while (preloadTaskRunning) {
            // Wait while main thread is loading tiles (avoid SD contention)
            if (mainThreadLoading) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            // Wait for tile request (100ms timeout to check if task should stop)
            if (xQueueReceive(tilePreloadQueue, &req, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Re-check flag after queue receive
                if (mainThreadLoading) continue;

                // Check if tile already in cache
                int cacheIdx = findCachedTile(req.zoom, req.tileX, req.tileY);
                if (cacheIdx < 0) {
                    // Not in cache - preload it
                    Serial.printf("[MAP-ASYNC] Preloading tile %d/%d/%d\n", req.zoom, req.tileX, req.tileY);
                    preloadTileToCache(req.tileX, req.tileY, req.zoom);
                }
            }
        }

        Serial.println("[MAP] Tile preload task stopped");
        vTaskDelete(NULL);
    }

    // Start the preload task
    void startTilePreloadTask() {
        if (tilePreloadTask != nullptr) return;  // Already running

        // Create queue
        if (tilePreloadQueue == nullptr) {
            tilePreloadQueue = xQueueCreate(TILE_PRELOAD_QUEUE_SIZE, sizeof(TileRequest));
        }

        preloadTaskRunning = true;
        xTaskCreatePinnedToCore(
            tilePreloadTaskFunc,
            "TilePreload",
            4096,
            NULL,
            1,  // Low priority
            &tilePreloadTask,
            1   // Core 1
        );
    }

    // Stop the preload task
    void stopTilePreloadTask() {
        if (tilePreloadTask == nullptr) return;

        preloadTaskRunning = false;
        vTaskDelay(pdMS_TO_TICKS(200));  // Wait for task to finish
        tilePreloadTask = nullptr;
    }

    // Queue tiles from adjacent zoom levels for preloading
    void queueAdjacentZoomTiles(int centerTileX, int centerTileY, int currentZoom) {
        if (tilePreloadQueue == nullptr) return;

        TileRequest req;

        // Get adjacent zoom levels
        int prevZoom = -1, nextZoom = -1;
        for (int i = 0; i < map_zoom_count; i++) {
            if (map_available_zooms[i] == currentZoom) {
                if (i > 0) prevZoom = map_available_zooms[i - 1];
                if (i < map_zoom_count - 1) nextZoom = map_available_zooms[i + 1];
                break;
            }
        }

        // Queue tiles for previous zoom (zoom out = tiles cover larger area)
        if (prevZoom > 0) {
            int scale = 1 << (currentZoom - prevZoom);  // e.g., zoom 12->10 = scale 4
            int prevTileX = centerTileX / scale;
            int prevTileY = centerTileY / scale;

            // Queue 3x3 grid around the corresponding tile
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    req.tileX = prevTileX + dx;
                    req.tileY = prevTileY + dy;
                    req.zoom = prevZoom;
                    xQueueSend(tilePreloadQueue, &req, 0);  // Don't block
                }
            }
        }

        // Queue tiles for next zoom (zoom in = tiles cover smaller area)
        if (nextZoom > 0) {
            int scale = 1 << (nextZoom - currentZoom);  // e.g., zoom 12->14 = scale 4
            int nextTileX = centerTileX * scale;
            int nextTileY = centerTileY * scale;

            // Queue 3x3 grid around the corresponding tile
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    req.tileX = nextTileX + dx;
                    req.tileY = nextTileY + dy;
                    req.zoom = nextZoom;
                    xQueueSend(tilePreloadQueue, &req, 0);  // Don't block
                }
            }
        }
    }
    // ============ END ASYNC TILE PRELOADING ============

    // Clear station hit zones and hide display objects
    void cleanup_station_buttons() {
        stationHitZoneCount = 0;
        // Hide all station display objects (keep pool intact for reuse)
        if (stationDisplayPoolInitialized) {
            for (int i = 0; i < STATION_POOL_SIZE; i++) {
                if (stationDisplayPool[i].container) {
                    lv_obj_add_flag(stationDisplayPool[i].container, LV_OBJ_FLAG_HIDDEN);
                }
                stationDisplayPool[i].inUse = false;
            }
        }
    }

    // Destroy station pool - call only when leaving map screen
    void destroy_station_pool() {
        stationHitZoneCount = 0;
        if (stationDisplayPoolInitialized) {
            for (int i = 0; i < STATION_POOL_SIZE; i++) {
                // Objects will be deleted with parent map_container
                // Just null out pointers to avoid dangling references
                stationDisplayPool[i].container = nullptr;
                stationDisplayPool[i].icon = nullptr;
                stationDisplayPool[i].icon_overlay = nullptr;
                stationDisplayPool[i].label = nullptr;
                stationDisplayPool[i].inUse = false;
            }
            stationDisplayPoolInitialized = false;
            Serial.println("[MAP] Station display pool destroyed");
        }
    }

    // Initialize station display pool (call once when map_container is created)
    void initStationDisplayPool() {
        if (stationDisplayPoolInitialized || !map_container) return;

        for (int i = 0; i < STATION_POOL_SIZE; i++) {
            // Container for positioning (transparent, no layout)
            stationDisplayPool[i].container = lv_obj_create(map_container);
            lv_obj_set_size(stationDisplayPool[i].container, 80, 40);
            lv_obj_set_style_bg_opa(stationDisplayPool[i].container, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(stationDisplayPool[i].container, 0, 0);
            lv_obj_set_style_pad_all(stationDisplayPool[i].container, 0, 0);
            lv_obj_clear_flag(stationDisplayPool[i].container, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(stationDisplayPool[i].container, LV_OBJ_FLAG_IGNORE_LAYOUT);

            // Icon: lv_img for APRS symbol PNG (24x24)
            stationDisplayPool[i].icon = lv_img_create(stationDisplayPool[i].container);
            lv_obj_set_size(stationDisplayPool[i].icon, SYMBOL_SIZE, SYMBOL_SIZE);
            lv_obj_align(stationDisplayPool[i].icon, LV_ALIGN_TOP_MID, 0, 0);
            lv_obj_set_style_border_width(stationDisplayPool[i].icon, 0, 0);
            lv_obj_set_style_outline_width(stationDisplayPool[i].icon, 0, 0);
            lv_obj_set_style_shadow_width(stationDisplayPool[i].icon, 0, 0);
            lv_obj_set_style_pad_all(stationDisplayPool[i].icon, 0, 0);
            lv_obj_set_style_bg_opa(stationDisplayPool[i].icon, LV_OPA_TRANSP, 0);
        
            // Overlay label: displays letters inside the symbol icon (white, centered)
            stationDisplayPool[i].icon_overlay = lv_label_create(stationDisplayPool[i].container);
            lv_label_set_text(stationDisplayPool[i].icon_overlay, "");
            lv_obj_set_style_text_font(stationDisplayPool[i].icon_overlay, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(stationDisplayPool[i].icon_overlay, lv_color_hex(0xffffff), 0);
            lv_obj_align(stationDisplayPool[i].icon_overlay, LV_ALIGN_TOP_MID, 0, 4);

            // Label for callsign (centered under icon)
            stationDisplayPool[i].label = lv_label_create(stationDisplayPool[i].container);
            lv_obj_align(stationDisplayPool[i].label, LV_ALIGN_TOP_MID, 0, SYMBOL_SIZE + 2);
            lv_obj_set_style_text_font(stationDisplayPool[i].label, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_color(stationDisplayPool[i].label, lv_color_hex(0x332221), 0);
            lv_obj_set_style_bg_color(stationDisplayPool[i].label, lv_color_hex(0x759a9e), 0);
            lv_obj_set_style_bg_opa(stationDisplayPool[i].label, LV_OPA_50, 0);
            lv_obj_set_style_pad_left(stationDisplayPool[i].label, 2, 0);
            lv_obj_set_style_pad_right(stationDisplayPool[i].label, 2, 0);
            lv_label_set_text(stationDisplayPool[i].label, "");

            // Hide initially
            lv_obj_add_flag(stationDisplayPool[i].container, LV_OBJ_FLAG_HIDDEN);
            stationDisplayPool[i].inUse = false;
        }

        stationDisplayPoolInitialized = true;
        Serial.println("[MAP] Station display pool initialized (LVGL objects with lv_img)");
    }

    // Move all visible station display objects by delta (for pan sync)
    void moveStationDisplayObjects(lv_coord_t dx, lv_coord_t dy) {
        if (!stationDisplayPoolInitialized) return;
        for (int i = 0; i < STATION_POOL_SIZE; i++) {
            if (stationDisplayPool[i].inUse && stationDisplayPool[i].container) {
                lv_coord_t x = lv_obj_get_x(stationDisplayPool[i].container);
                lv_coord_t y = lv_obj_get_y(stationDisplayPool[i].container);
                lv_obj_set_pos(stationDisplayPool[i].container, x + dx, y + dy);
            }
        }
    }

    // Helper: parse APRS symbol string and get cached symbol image descriptor
    static CachedSymbol* parseAndGetSymbol(const char* aprsSymbol) {
        char table = '/';
        char symbol = ' ';
        if (aprsSymbol && strlen(aprsSymbol) >= 2) {
            if (aprsSymbol[0] == '/' || aprsSymbol[0] == '\\') {
                table = aprsSymbol[0];
                symbol = aprsSymbol[1];
            } else {
                table = '\\';  // Overlay = alternate table
                symbol = aprsSymbol[1];
            }
        } else if (aprsSymbol && strlen(aprsSymbol) >= 1) {
            symbol = aprsSymbol[0];
        }
        return getSymbolCacheEntry(table, symbol);
    }

    // Helper: configure a pool entry with position, symbol image, and label
    static void configurePoolEntry(int poolIdx, int screenX, int screenY,
                                   const char* callsign, const char* aprsSymbol, int8_t stationIdx) {
        if (poolIdx >= STATION_POOL_SIZE || !stationDisplayPool[poolIdx].container) return;

        StationDisplayObj& obj = stationDisplayPool[poolIdx];

        // Position container (center 80px container on symbol position)
        int objX = screenX - MAP_CANVAS_MARGIN - 40;
        int objY = screenY - MAP_CANVAS_MARGIN - SYMBOL_SIZE / 2;
        lv_obj_set_pos(obj.container, objX, objY);

        // Try loading PNG symbol
        CachedSymbol* cache = parseAndGetSymbol(aprsSymbol);

        if (cache && cache->valid) {
            // PNG found — display it
            lv_obj_clear_flag(obj.icon, LV_OBJ_FLAG_HIDDEN);
            lv_img_set_src(obj.icon, &cache->img_dsc);
            lv_obj_set_style_bg_opa(obj.icon, LV_OPA_TRANSP, 0);
            lv_obj_set_style_bg_opa(obj.icon_overlay, LV_OPA_TRANSP, 0);
            lv_label_set_text(obj.icon_overlay, "");  // Clear overlay by default
        } else {
            // Fallback: hide icon (no valid PNG), show symbol char via overlay
            lv_obj_add_flag(obj.icon, LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(obj.icon_overlay, lv_color_hex(0xff0000), 0);
            lv_obj_set_style_bg_opa(obj.icon_overlay, LV_OPA_COVER, 0);
            lv_obj_set_style_radius(obj.icon_overlay, LV_RADIUS_CIRCLE, 0);

            if (aprsSymbol && strlen(aprsSymbol) >= 2) {
                char symChar[2] = { aprsSymbol[1], '\0' };
                lv_label_set_text(obj.icon_overlay, symChar);
            }
        }

        // Alphanumeric overlay (e.g., "L", "1") — displayed on top of icon
        if (aprsSymbol && strlen(aprsSymbol) >= 2) {
            char overlay = aprsSymbol[0];
            if (overlay != '/' && overlay != '\\' && overlay != ' ') {
                char ovStr[2] = { overlay, '\0' };
                lv_label_set_text(obj.icon_overlay, ovStr);
            }
        }

        // Color label background by station type
        lv_color_t bgColor = lv_color_hex(0x759a9e);  // Default GPS gray
        if (aprsSymbol && strlen(aprsSymbol) >= 2) {
            char sym = aprsSymbol[1];
            if (sym == '&')      bgColor = lv_color_hex(0xcc0000);  // Red: iGate
            else if (sym == '#') bgColor = lv_color_hex(0x00cc00);  // Green: Digipeater
            else if (sym == '_') bgColor = lv_color_hex(0x0055cc);  // Blue: Weather
        }
        lv_obj_set_style_bg_color(obj.label, bgColor, 0);

        lv_label_set_text(obj.label, callsign);
        lv_obj_clear_flag(obj.container, LV_OBJ_FLAG_HIDDEN);
        obj.inUse = true;

        // Store hit zone (only for received stations, not own position)
        if (stationIdx >= 0 && stationHitZoneCount < MAP_STATIONS_MAX) {
            stationHitZones[stationHitZoneCount].x = screenX - MAP_CANVAS_MARGIN;
            stationHitZones[stationHitZoneCount].y = screenY - MAP_CANVAS_MARGIN + 12;
            stationHitZones[stationHitZoneCount].w = 80;
            stationHitZones[stationHitZoneCount].h = 50;
            stationHitZones[stationHitZoneCount].stationIdx = stationIdx;
            stationHitZoneCount++;
        }
    }

    // Update all station LVGL objects (own position + received stations)
    void update_station_objects() {
        if (!map_container) return;

        // Init pool if needed
        if (!stationDisplayPoolInitialized) {
            initStationDisplayPool();
        }

        stationHitZoneCount = 0;
        int displayIdx = 0;

        // Index 0: own position
        if (gps.location.isValid()) {
            int myX, myY;
            latLonToPixel(gps.location.lat(), gps.location.lng(),
                          map_center_lat, map_center_lon, map_current_zoom, &myX, &myY);
            if (myX >= 0 && myX < MAP_CANVAS_WIDTH && myY >= 0 && myY < MAP_CANVAS_HEIGHT) {
                Beacon* currentBeacon = &Config.beacons[myBeaconsIndex];
                char fullSymbol[4]; // Overlay (1) + Symbol (1) + Null
                snprintf(fullSymbol, sizeof(fullSymbol), "%s%s", currentBeacon->overlay.c_str(), currentBeacon->symbol.c_str());
                configurePoolEntry(displayIdx, myX, myY,
                                   currentBeacon->callsign.c_str(), fullSymbol, -1);
                displayIdx++;
            }
        }

        // Received stations
        STATION_Utils::cleanOldMapStations();
        int validCount = 0, visibleCount = 0;
        for (int i = 0; i < MAP_STATIONS_MAX && displayIdx < STATION_POOL_SIZE; i++) {
            MapStation* station = STATION_Utils::getMapStation(i);
            if (station && station->valid && station->latitude != 0.0f && station->longitude != 0.0f) {
                validCount++;
                int stX, stY;
                latLonToPixel(station->latitude, station->longitude,
                              map_center_lat, map_center_lon, map_current_zoom, &stX, &stY);
            
                if (stX >= 0 && stX < MAP_CANVAS_WIDTH && stY >= 0 && stY < MAP_CANVAS_HEIGHT) {
                    visibleCount++;
                    // station->symbol already contains full APRS symbol (overlay+symbol, e.g. "/[" or "\>")
                    configurePoolEntry(displayIdx, stX, stY,
                                       station->callsign.c_str(), station->symbol.c_str(), i);
                    displayIdx++;
                }
            }
        }

        // Hide unused objects
        for (int i = displayIdx; i < STATION_POOL_SIZE; i++) {
            if (stationDisplayPool[i].container) {
                lv_obj_add_flag(stationDisplayPool[i].container, LV_OBJ_FLAG_HIDDEN);
            }
            stationDisplayPool[i].inUse = false;
        }
    }

    // Initialize symbol cache
    void initSymbolCache() {
        if (symbolCacheInitialized) return;
        for (int i = 0; i < SYMBOL_CACHE_SIZE; i++) {
            symbolCache[i].data = nullptr;
            symbolCache[i].valid = false;
            symbolCache[i].table = 0;
            symbolCache[i].symbol = 0;
            symbolCache[i].lastAccess = 0;
        }
        symbolCacheInitialized = true;
        Serial.println("[MAP] Symbol cache initialized");
    }

    // Forward declarations for PNG callbacks
    static void* pngOpenFile(const char* filename, int32_t* size);
    static void pngCloseFile(void* handle);
    static int32_t pngReadFile(PNGFILE* pFile, uint8_t* pBuf, int32_t iLen);
    static int32_t pngSeekFile(PNGFILE* pFile, int32_t iPosition);
    static bool pngFileOpened = false;  // Track if PNG file actually opened

    // PNG draw callback for symbols - stores alpha channel info
    static uint8_t* symbolCombinedBuffer = nullptr;  // Target combined buffer
    static PNG symbolPNG;  // PNG decoder instance for symbols

    static int pngSymbolCallback(PNGDRAW* pDraw) {
        if (!symbolCombinedBuffer) return 1;
        if (pDraw->y >= SYMBOL_SIZE) return 1;  // Clamp oversized PNGs

        const int w = (pDraw->iWidth < SYMBOL_SIZE) ? pDraw->iWidth : SYMBOL_SIZE;
        const size_t rgb565Offset = pDraw->y * SYMBOL_SIZE;  // In uint16_t units
        const size_t alphaOffset  = SYMBOL_SIZE * SYMBOL_SIZE * sizeof(uint16_t)
                                  + pDraw->y * SYMBOL_SIZE;   // In bytes

        // Extract alpha BEFORE getLineAsRGB565 (which overwrites pPixels)
        uint8_t* alphaRow = symbolCombinedBuffer + alphaOffset;
        if (pDraw->iHasAlpha) {
            for (int x = 0; x < w; x++) {
                alphaRow[x] = pDraw->pPixels[x * 4 + 3];  // RGBA -> A
            }
            for (int x = w; x < SYMBOL_SIZE; x++) {
                alphaRow[x] = 0;  // Transparent padding
            }
        } else {
            memset(alphaRow, 255, SYMBOL_SIZE);  // Fully opaque
        }

        // Decode line as RGB565 with black background (0x000000)
        uint16_t* rgb565Row = (uint16_t*)symbolCombinedBuffer + rgb565Offset;
        symbolPNG.getLineAsRGB565(pDraw, rgb565Row, PNG_RGB565_LITTLE_ENDIAN, 0x00000000);

        return 1;
    }

    // Load symbol PNG from SD card into a combined RGB565A8 buffer
    uint8_t* loadSymbolFromSD(char table, char symbol) {
        String tableName = (table == '/') ? "primary" : "alternate";
        char hexCode[3];
        snprintf(hexCode, sizeof(hexCode), "%02X", (uint8_t)symbol);
        String path = String("/LoRa_Tracker/Symbols/") + tableName + "/" + hexCode + ".png";

        if (!STORAGE_Utils::isSDAvailable()) {
            Serial.println("[SYMBOL] SD not available");
            return nullptr;
        }

        const size_t rgb565Size = SYMBOL_SIZE * SYMBOL_SIZE * sizeof(uint16_t);
        const size_t alphaSize  = SYMBOL_SIZE * SYMBOL_SIZE;
        const size_t totalSize  = rgb565Size + alphaSize;  // 1728 bytes for 24x24

        // Single allocation: combined RGB565A8 buffer in PSRAM
        uint8_t* combined = (uint8_t*)ps_malloc(totalSize);
        if (!combined) {
            Serial.println("[SYMBOL] PSRAM allocation failed");
            return nullptr;
        }
        memset(combined, 0, totalSize);  // Zero-init (transparent black)

        // Point callback directly at combined buffer
        symbolCombinedBuffer = combined;

        // Decode PNG
        int rc = symbolPNG.open(path.c_str(), pngOpenFile, pngCloseFile, pngReadFile, pngSeekFile, pngSymbolCallback);
        if (rc == PNG_SUCCESS && pngFileOpened) {
            rc = symbolPNG.decode(nullptr, 0);
            symbolPNG.close();

            if (rc == PNG_SUCCESS) {
                symbolCombinedBuffer = nullptr;
                Serial.printf("[SYMBOL] Loaded RGB565A8: %c%c from %s\n", table, symbol, path.c_str());
                return combined;
            }
        }

        // Failed
        symbolPNG.close();
        symbolCombinedBuffer = nullptr;
        free(combined);
        Serial.printf("[SYMBOL] Failed to load: %s\n", path.c_str());
        return nullptr;
    }

    // Get symbol cache entry from cache or load from SD
    CachedSymbol* getSymbolCacheEntry(char table, char symbol) {
        initSymbolCache();

        // Search in cache
        for (int i = 0; i < SYMBOL_CACHE_SIZE; i++) {
            if (symbolCache[i].valid && symbolCache[i].table == table && symbolCache[i].symbol == symbol) {
                symbolCache[i].lastAccess = symbolCacheAccessCounter++;
                return &symbolCache[i];
            }
        }

        // Not in cache - load from SD (returns combined RGB565A8 buffer)
        uint8_t* data = loadSymbolFromSD(table, symbol);
        if (!data) {
            return nullptr;  // Symbol not found
        }

        // Find LRU slot or empty slot
        int slotIdx = -1;
        uint32_t oldestAccess = UINT32_MAX;
        for (int i = 0; i < SYMBOL_CACHE_SIZE; i++) {
            if (!symbolCache[i].valid) {
                slotIdx = i;
                break;
            }
            if (symbolCache[i].lastAccess < oldestAccess) {
                oldestAccess = symbolCache[i].lastAccess;
                slotIdx = i;
            }
        }

        // Evict old entry if needed
        if (symbolCache[slotIdx].valid) {
            if (symbolCache[slotIdx].data) {
                free(symbolCache[slotIdx].data);
            }
        }

        const size_t rgb565Size = SYMBOL_SIZE * SYMBOL_SIZE * sizeof(uint16_t);
        const size_t alphaSize = SYMBOL_SIZE * SYMBOL_SIZE;

        // Store in cache
        symbolCache[slotIdx].table = table;
        symbolCache[slotIdx].symbol = symbol;
        symbolCache[slotIdx].data = data;
        symbolCache[slotIdx].lastAccess = symbolCacheAccessCounter++;
        symbolCache[slotIdx].valid = true;

        // Setup LVGL image descriptor for RGB565A8 format
        symbolCache[slotIdx].img_dsc.header.always_zero = 0;
        symbolCache[slotIdx].img_dsc.header.w = SYMBOL_SIZE;
        symbolCache[slotIdx].img_dsc.header.h = SYMBOL_SIZE;
        symbolCache[slotIdx].img_dsc.data_size = rgb565Size + alphaSize;
        symbolCache[slotIdx].img_dsc.header.cf = LV_IMG_CF_RGB565A8;
        symbolCache[slotIdx].img_dsc.data = data;

        return &symbolCache[slotIdx];
    }

    // Get symbol from cache or load from SD (backward compatibility wrapper)
    lv_img_dsc_t* getSymbol(char table, char symbol) {
        CachedSymbol* cache = getSymbolCacheEntry(table, symbol);
        return cache ? &cache->img_dsc : nullptr;
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

    // Find a tile in cache by its coordinates, returns index or -1
    int findCachedTile(int zoom, int tileX, int tileY) {
        // Create a unique hash from coordinates. Max zoom 14 needs 4 bits. Max tile coord at z14 is 16383 (14 bits).
        // Hash: 4 bits for zoom, 14 for tileX, 14 for tileY.
        uint32_t tileHash = (static_cast<uint32_t>(zoom) << 28) | (static_cast<uint32_t>(tileX) << 14) | static_cast<uint32_t>(tileY);

        for (int i = 0; i < tileCache.size(); ++i) {
            if (tileCache[i].isValid && tileCache[i].tileHash == tileHash) {
                tileCache[i].lastAccess = ++cacheAccessCounter;
                return i;
            }
        }
        return -1; // Not found
    }

    // Find a tile in cache by its path, returns index or -1
    int findCachedTile(const char* filePath) {
        for (int i = 0; i < tileCache.size(); ++i) {
            if (tileCache[i].isValid && strcmp(tileCache[i].filePath, filePath) == 0) {
                tileCache[i].lastAccess = ++cacheAccessCounter;
                return i;
            }
        }
        return -1; // Not found
    }

    // Remove least recently used tile from cache
    void evictLRUTile() {
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
void addToCache(const char* filePath, int zoom, int tileX, int tileY, TFT_eSprite* sourceSprite) {
    if (maxCachedTiles == 0 || !sourceSprite) {
        if(sourceSprite) { // Don't leak memory if cache is disabled
            sourceSprite->deleteSprite();
            delete sourceSprite;
        }
        return;
    }

    // If cache is full, evict the least recently used tile
    if (tileCache.size() >= maxCachedTiles) {
        evictLRUTile();
    }

    // Create new cache entry and take ownership of the sprite pointer
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


    // --- Batch Rendering System ---

    static RenderBatch* activeBatch = nullptr;
    static size_t maxBatchSize = 0;

    static size_t getOptimalBatchSize() {
        return maxBatchSize;
    }

    void initBatchRendering() {
        #ifdef BOARD_HAS_PSRAM
            size_t psramFree = ESP.getFreePsram();
            if (psramFree >= 4 * 1024 * 1024) 
                maxBatchSize = 512;
            else if (psramFree >= 2 * 1024 * 1024)
                maxBatchSize = 256;
            else 
                maxBatchSize = 128;
        #else
            maxBatchSize = 64;
        #endif
        
        activeBatch = nullptr;
    }

    static void createRenderBatch(size_t capacity) {
        if (activeBatch) {
            if (activeBatch->segments) 
                delete[] activeBatch->segments;
            delete activeBatch;
            activeBatch = nullptr;
        }
        
        activeBatch = new RenderBatch();
        activeBatch->segments = new LineSegment[capacity];
        activeBatch->count = 0;
        activeBatch->capacity = capacity;
        activeBatch->color = 0;
    }

    static bool canBatch(uint16_t color) {
        if (!activeBatch) 
            return true;
        return (activeBatch->count == 0) || (activeBatch->color == color);
    }

    static void addToBatch(int x0, int y0, int x1, int y1, uint16_t color) {
        if (!activeBatch)
            createRenderBatch(maxBatchSize);
        
        if (!canBatch(color) || activeBatch->count >= activeBatch->capacity) 
            return;

        activeBatch->segments[activeBatch->count] = {x0, y0, x1, y1, color};
        activeBatch->count++;
        
        if (activeBatch->count == 1) 
            activeBatch->color = color;
    }

    static void flushBatch(TFT_eSprite& map) {
        if (!activeBatch || activeBatch->count == 0)
            return;
        
        for (size_t i = 0; i < activeBatch->count; i++) {
            const LineSegment& segment = activeBatch->segments[i];
            map.drawLine(segment.x0, segment.y0, segment.x1, segment.y1, segment.color);
        }
        
        activeBatch->count = 0;
        activeBatch->color = 0;
    }

    // --- Constants and State Variables for Vector Rendering ---
    static constexpr int VTILE_SIZE = 256;
    static constexpr int MARGIN_PIXELS = 1;
    static uint8_t PALETTE[256] = {0};
    static uint32_t PALETTE_SIZE = 0;
    static bool fillPolygons = true;


    // --- Color Management Utilities ---

    bool loadPalette(const char* palettePath) {
        FILE* f = nullptr;
        if (palettePath) {
            f = fopen(palettePath, "rb");
        }
    
        if (!f) {
            // Default palette fallback
            PALETTE[0] = 0x00; // Black
            PALETTE[1] = 0x25; // Dark Gray (for roads)
            PALETTE[2] = 0x10; // Green (for parks)
            PALETTE[3] = 0x02; // Blue (for water)
            PALETTE_SIZE = 4;
            Serial.println("[MAP] palette.bin not found, using default fallback palette.");
            return true;
        }
        
        uint32_t numColors;
        if (fread(&numColors, 4, 1, f) != 1) {
            fclose(f);
            return false;
        }
        
        uint8_t rgb888[3];
        PALETTE_SIZE = 0;
        for (uint32_t i = 0; i < numColors && i < 256; i++) {
            if (fread(rgb888, 3, 1, f) == 1) {
                uint8_t r332 = rgb888[0] & 0xE0;
                uint8_t g332 = (rgb888[1] & 0xE0) >> 3;
                uint8_t b332 = rgb888[2] >> 6;
                PALETTE[i] = r332 | g332 | b332;
                PALETTE_SIZE++;
            }
        }
        
        fclose(f);
        Serial.printf("[MAP] Loaded palette: %u colors\n", PALETTE_SIZE);
        return PALETTE_SIZE > 0;
    }

    uint8_t paletteToRGB332(const uint32_t idx) {
        if (idx < PALETTE_SIZE) return PALETTE[idx];
        return 0xFF; // White
    }

    uint16_t RGB332ToRGB565(const uint8_t color) {
        uint8_t r = (color & 0xE0);
        uint8_t g = (color & 0x1C) << 3;
        uint8_t b = (color & 0x03) << 6;
        
        uint16_t r565 = (r >> 3) & 0x1F;
        uint16_t g565 = (g >> 2) & 0x3F;
        uint16_t b565 = (b >> 3) & 0x1F;
        
        return (r565 << 11) | (g565 << 5) | b565;
    }

    uint8_t darkenRGB332(const uint8_t color, const float amount = 0.4f) {
        uint8_t r = (color & 0xE0) >> 5;
        uint8_t g = (color & 0x1C) >> 2;
        uint8_t b = (color & 0x03);

        r = static_cast<uint8_t>(r * (1.0f - amount));
        g = static_cast<uint8_t>(g * (1.0f - amount));
        b = static_cast<uint8_t>(b * (1.0f - amount));

        return ((r << 5) | (g << 2) | b);
    }

    // --- Data Decoding Utilities ---

    uint32_t readVarint(const uint8_t* data, size_t& offset, const size_t dataSize) {
        uint32_t value = 0;
        uint8_t shift = 0;
        while (offset < dataSize && shift < 32) {
            uint8_t byte = data[offset++];
            value |= ((uint32_t)(byte & 0x7F)) << shift;
            if ((byte & 0x80) == 0) break;
            shift += 7;
        }
        if (offset > dataSize) {
            offset = dataSize;
            return 0;
        }
        return value;
    }

    int32_t readZigzag(const uint8_t* data, size_t& offset, const size_t dataSize) {
        if (offset >= dataSize) return 0;
        const uint32_t encoded = readVarint(data, offset, dataSize);
        return static_cast<int32_t>((encoded >> 1) ^ (-(int32_t)(encoded & 1)));
    }

    // --- Geometric and Drawing Primitives ---

    int uint16ToPixel(const int32_t val) {
        int p = static_cast<int>((val * (long long)VTILE_SIZE) / 65536);
        if (p < 0) p = 0;
        if (p >= VTILE_SIZE) p = VTILE_SIZE - 1;
        return p;
    }

    bool isPointOnMargin(const int px, const int py) {
        return (px <= MARGIN_PIXELS || px >= VTILE_SIZE - 1 - MARGIN_PIXELS || 
                py <= MARGIN_PIXELS || py >= VTILE_SIZE - 1 - MARGIN_PIXELS);
    }

    bool isNear(int val, int target, int tol = 2) {
        return abs(val - target) <= tol;
    }

    bool shouldDrawLine(const int px1, const int py1, const int px2, const int py2) {
        if ((isNear(px1, 0) && isNear(px2, VTILE_SIZE - 1)) || (isNear(px1, VTILE_SIZE - 1) && isNear(px2, 0))) {
            if ((isNear(py1, 0) && isNear(py2, VTILE_SIZE - 1)) || (isNear(py1, VTILE_SIZE - 1) && isNear(py2, 0))) return false;
            if (isNear(py1, py2)) return false;
        }
        if ((isNear(py1, 0) && isNear(py2, VTILE_SIZE - 1)) || (isNear(py1, VTILE_SIZE - 1) && isNear(py2, 0))) {
            if (isNear(px1, px2)) return false;
        }

        int dx = px2 - px1;
        int dy = py2 - py1;
        long len2 = (long)dx*dx + (long)dy*dy;
        if (len2 > (VTILE_SIZE * VTILE_SIZE * 3)) return false;

        if (isPointOnMargin(px1, py1) && isPointOnMargin(px2, py2)) return false;
        if ((px1 == px2) && (px1 <= MARGIN_PIXELS || px1 >= VTILE_SIZE - 1 - MARGIN_PIXELS)) return false;
        if ((py1 == py2) && (py1 <= MARGIN_PIXELS || py1 >= VTILE_SIZE - 1 - MARGIN_PIXELS)) return false;
        
        return true;
    }

    void fillPolygonGeneral(TFT_eSprite &map, const int *px, const int *py, const int numPoints, const uint16_t color, const int xOffset, const int yOffset) {
        int miny = py[0], maxy = py[0];
        for (int i = 1; i < numPoints; ++i) {
            if (py[i] < miny) miny = py[i];
            if (py[i] > maxy) maxy = py[i];
        }

        int* xints = (int*)ps_malloc(numPoints * sizeof(int));
        if (!xints) {
            Serial.println("[MAP] fillPolygonGeneral: ps_malloc failed");
            return;
        }

        for (int y = miny; y <= maxy; ++y) {
            int nodes = 0;
            for (int i = 0, j = numPoints - 1; i < numPoints; j = i++) {
                if ((py[i] < y && py[j] >= y) || (py[j] < y && py[i] >= y)) {
                    if (py[j] - py[i] != 0) {
                       xints[nodes++] = static_cast<int>(px[i] + (float)(y - py[i]) * (px[j] - px[i]) / (py[j] - py[i]));
                    }
                }
            }
            if (nodes > 1) {
                std::sort(xints, xints + nodes);
                for (int i = 0; i < nodes; i += 2) {
                    if (i + 1 < nodes) {
                        int x0 = xints[i] + xOffset;
                        int x1 = xints[i + 1] + xOffset;
                        int yy = y + yOffset;
                        if (yy >= 0 && yy < VTILE_SIZE + yOffset) {
                            if (x0 < 0) x0 = 0;
                            if (x1 >= VTILE_SIZE + xOffset) x1 = VTILE_SIZE - 1 + xOffset;
                            map.drawLine(x0, yy, x1, yy, color);
                        }
                    }
                }
            }
        }
        free(xints);
    }

    void drawPolygonBorder(TFT_eSprite &map, const int *px, const int *py, const int numPoints, const uint16_t borderColor, const uint16_t fillColor, const int xOffset, const int yOffset) {
        if (numPoints < 2) return;

        for (int i = 0; i < numPoints; ++i) {
            int j = (i + 1) % numPoints;
            const bool marginA = isPointOnMargin(px[i], py[i]);
            const bool marginB = isPointOnMargin(px[j], py[j]);
            const uint16_t color = (marginA && marginB) ? fillColor : borderColor;

            const int x0 = px[i] + xOffset;
            const int y0 = py[i] + yOffset;
            const int x1 = px[j] + xOffset;
            const int y1 = py[j] + yOffset;
            
            if (!(marginA && marginB)) {
                map.drawLine(x0, y0, x1, y1, color);
            }
        }
    }

// --- Main Rendering Function ---

    static void tileToLonLat(int tileX, int tileY, int zoom, double& lon, double& lat) {
        int n = 1 << zoom;
        lon = tileX / (double)n * 360.0 - 180.0;
        double lat_rad = atan(sinh(M_PI * (1.0 - 2.0 * tileY / n)));
        lat = lat_rad * 180.0 / M_PI;
    }

    uint16_t darkenRGB565(const uint16_t color, const float amount = 0.4f) {
        uint8_t r5 = (color >> 11) & 0x1F;
        uint8_t g6 = (color >> 5) & 0x3F;
        uint8_t b5 = color & 0x1F;

        r5 = static_cast<uint8_t>(r5 * (1.0f - amount));
        g6 = static_cast<uint8_t>(g6 * (1.0f - amount));
        b5 = static_cast<uint8_t>(b5 * (1.0f - amount));

        return (r5 << 11) | (g6 << 5) | b5;
    }

bool renderTile(const char* path, int tileX, int tileY, int zoom, int16_t xOffset, int16_t yOffset, TFT_eSprite &map) {
    if (!path || path[0] == '\0') return false;

    // Use Arduino SD library which handles /sd prefix correctly
    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.printf("[MAP] Failed to open tile with SD.open: %s\n", path);
        return false;
    }

    const size_t fileSize = file.size();
    if (fileSize < 22) { // Minimum header size
        file.close();
        return false;
    }

    uint8_t* data = (uint8_t*)ps_malloc(fileSize);
    if (!data) {
        file.close();
        Serial.printf("[MAP] Failed to allocate %ld bytes for tile %s\n", fileSize, path);
        return false;
    }

    const size_t bytesRead = file.read(data, fileSize);
    file.close();

    if (bytesRead != fileSize) {
        free(data);
        return false;
    }

    size_t offset = 0;

    // --- NAV1 Header (22 bytes) ---
    if (memcmp(data, "NAV1", 4) != 0) {
        free(data);
        return false;
    }
    offset += 4;

    uint16_t feature_count;
    memcpy(&feature_count, data + offset, 2);
    offset += 2;

    Serial.printf("[MAP] NAV1: Found %d features in %s\n", feature_count, path);

    // Read bounding box from header, but IGNORE it for scaling. We calculate precise tile boundaries.
    int32_t min_lon_header, min_lat_header, max_lon_header, max_lat_header;
    memcpy(&min_lon_header, data + offset, 4); offset += 4;
    memcpy(&min_lat_header, data + offset, 4); offset += 4;
    memcpy(&max_lon_header, data + offset, 4); offset += 4;
    memcpy(&max_lat_header, data + offset, 4); offset += 4;
    
    // --- Calculate precise tile boundaries for scaling ---
    double tile_min_lon_deg, tile_max_lat_deg;
    double tile_max_lon_deg, tile_min_lat_deg;
    tileToLonLat(tileX, tileY, zoom, tile_min_lon_deg, tile_max_lat_deg);
    tileToLonLat(tileX + 1, tileY + 1, zoom, tile_max_lon_deg, tile_min_lat_deg);

    const int32_t tile_min_lon_e7 = static_cast<int32_t>(tile_min_lon_deg * 10000000.0);
    const int32_t tile_min_lat_e7 = static_cast<int32_t>(tile_min_lat_deg * 10000000.0);
    const int32_t tile_max_lon_e7 = static_cast<int32_t>(tile_max_lon_deg * 10000000.0);
    const int32_t tile_max_lat_e7 = static_cast<int32_t>(tile_max_lat_deg * 10000000.0);

    Serial.printf("[MAP] Tile Bounds E7: Lon(%d to %d) Lat(%d to %d)\n", tile_min_lon_e7, tile_max_lon_e7, tile_min_lat_e7, tile_max_lat_e7);

    int64_t delta_lon = tile_max_lon_e7 - tile_min_lon_e7;
    int64_t delta_lat = tile_max_lat_e7 - tile_min_lat_e7;
    if (delta_lon == 0) delta_lon = 1;
    if (delta_lat == 0) delta_lat = 1;

    int executed = 0;

    // --- Feature Loop ---
    for (uint16_t i = 0; i < feature_count; ++i) {
        if (offset + 6 > fileSize) break; // Check for feature header size

        uint8_t geometry_type = data[offset++];
        uint16_t color;
        memcpy(&color, data + offset, 2); offset += 2;
        uint8_t zoom_priority = data[offset++];
        uint8_t width = data[offset++];
        uint16_t coord_count;
        memcpy(&coord_count, data + offset, 2); offset += 2;

        if (coord_count == 0) continue;
        if (offset + coord_count * 8 > fileSize) break; // Check for coordinates data size

        int* px = (int*)ps_malloc(coord_count * sizeof(int));
        int* py = (int*)ps_malloc(coord_count * sizeof(int));
        if (!px || !py) {
            if (px) free(px);
            if (py) free(py);
            offset += coord_count * 8;
            continue;
        }

        // --- Coordinate Transformation ---
        for (uint16_t j = 0; j < coord_count; ++j) {
            int32_t lon, lat;
            memcpy(&lon, data + offset, 4); offset += 4;
            memcpy(&lat, data + offset, 4); offset += 4;

            px[j] = (int)(((int64_t)(lon - tile_min_lon_e7) * (VTILE_SIZE - 1)) / delta_lon);
            py[j] = VTILE_SIZE - 1 - (int)(((int64_t)(lat - tile_min_lat_e7) * (VTILE_SIZE - 1)) / delta_lat);
        }
        
        // Add debug logs for the first 10 features
        if (i < 10) {
            Serial.printf("[MAP] Feature %d: Type=%s, Color=0x%04X, Points=%d, FirstPx=(%d, %d)\n",
                i,
                (geometry_type == 1) ? "Point" : ((geometry_type == 2) ? "Line" : "Polygon"),
                color,
                coord_count,
                (coord_count > 0) ? px[0] : -1,
                (coord_count > 0) ? py[0] : -1
            );
        }
        
        // --- Rendering ---
        if (geometry_type == 2 && coord_count >= 2) { // Line
            for (uint16_t j = 1; j < coord_count; ++j) {
                map.drawWideLine(px[j-1] + xOffset, py[j-1] + yOffset, px[j] + xOffset, py[j] + yOffset, width, color);
            }
            executed++;
        } else if (geometry_type == 3 && coord_count >= 3) { // Polygon
            // Temporarily disable polygon fill to see if lines appear
            // fillPolygonGeneral(map, px, py, coord_count, color, xOffset, yOffset);
            const uint16_t borderColor = darkenRGB565(color);
            drawPolygonBorder(map, px, py, coord_count, borderColor, color, xOffset, yOffset);
            executed++;
        }
        
        free(px);
        free(py);
    }

    free(data);
    return executed > 0;
}

    // =========================================================================
    // =                  END OF VECTOR TILE RENDERING ENGINE                  =
    // =========================================================================

    // JPEG decoder for map tiles
    static JPEGDEC jpeg;

    // PNG decoder for map tiles
    static PNG png;

    // Context for decoding JPEG/PNG to a TFT_eSprite
    struct SpriteDecodeContext {
        TFT_eSprite* sprite;
    };
    static SpriteDecodeContext spriteDecodeContext;

    // JPEGDEC callback to draw MCU blocks directly to the sprite
    static int jpegSpriteCallback(JPEGDRAW* pDraw) {
        if (!spriteDecodeContext.sprite) return 0; // Stop if sprite is not set
        // pushImage is the fastest way to draw the decoded block to the sprite
        spriteDecodeContext.sprite->pushImage(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->pPixels);
        return 1; // Continue decoding
    }

    // PNGdec callback to draw scanlines directly into the sprite's framebuffer
    static int pngSpriteCallback(PNGDRAW* pDraw) {
        if (!spriteDecodeContext.sprite) return 0; // Stop if sprite is not set
        // Get a pointer to the sprite's framebuffer
        uint16_t* pfb = (uint16_t*)spriteDecodeContext.sprite->frameBuffer(0);
        // Calculate the start of the current line in the framebuffer
        uint16_t* pLine = pfb + (pDraw->y * MAP_TILE_SIZE);
        // Decode the line directly into the sprite's framebuffer
        png.getLineAsRGB565(pDraw, pLine, PNG_RGB565_LITTLE_ENDIAN, 0xffffffff);
        return 1; // Continue decoding
    }

    // PNG file callbacks
    static void* pngOpenFile(const char* filename, int32_t* size) {
        pngFileOpened = false;
        File* file = new File(SD.open(filename, FILE_READ));
        if (!file || !*file) {
            delete file;
            return nullptr;
        }
        *size = file->size();
        pngFileOpened = true;
        return file;
    }

    static void pngCloseFile(void* handle) {
        File* file = (File*)handle;
        if (file) {
            file->close();
            delete file;
        }
    }

    static int32_t pngReadFile(PNGFILE* pFile, uint8_t* pBuf, int32_t iLen) {
        File* file = (File*)pFile->fHandle;
        return file->read(pBuf, iLen);
    }

    static int32_t pngSeekFile(PNGFILE* pFile, int32_t iPosition) {
        File* file = (File*)pFile->fHandle;
        return file->seek(iPosition);
    }

    // JPEG file callbacks
    static void* jpegOpenFile(const char* filename, int32_t* size) {
        File* file = new File(SD.open(filename, FILE_READ));
        if (!file || !*file) {
            delete file;
            return nullptr;
        }
        *size = file->size();
        return file;
    }

    static void jpegCloseFile(void* handle) {
        File* file = (File*)handle;
        if (file) {
            file->close();
            delete file;
        }
    }

    static int32_t jpegReadFile(JPEGFILE* pFile, uint8_t* pBuf, int32_t iLen) {
        File* file = (File*)pFile->fHandle;
        return file->read(pBuf, iLen);
    }

    static int32_t jpegSeekFile(JPEGFILE* pFile, int32_t iPosition) {
        File* file = (File*)pFile->fHandle;
        return file->seek(iPosition);
    }

    // Preload a tile into cache (no canvas drawing) - called from Core 1 task
    bool preloadTileToCache(int tileX, int tileY, int zoom) {
        // TODO: Re-implement for new cache logic.
        // The old implementation decoded raster tiles to a raw buffer, which is incompatible
        // with the new sprite-based cache for vector tiles.
        return false;
    }


    // Convert lat/lon to tile coordinates
    void latLonToTile(float lat, float lon, int zoom, int* tileX, int* tileY) {
        int n = 1 << zoom;
        *tileX = (int)((lon + 180.0f) / 360.0f * n);
        float latRad = lat * PI / 180.0f;
        *tileY = (int)((1.0f - log(tan(latRad) + 1.0f / cos(latRad)) / PI) / 2.0f * n);
    }

    // Convert lat/lon to pixel position on screen (relative to center)
    void latLonToPixel(float lat, float lon, float centerLat, float centerLon, int zoom, int* pixelX, int* pixelY) {
        int centerTileX, centerTileY;
        latLonToTile(centerLat, centerLon, zoom, &centerTileX, &centerTileY);

        int targetTileX, targetTileY;
        latLonToTile(lat, lon, zoom, &targetTileX, &targetTileY);

        // Calculate sub-tile position
        int n = 1 << zoom;
        float subX = ((lon + 180.0f) / 360.0f * n) - targetTileX;
        float subY = ((1.0f - log(tan(lat * PI / 180.0f) + 1.0f / cos(lat * PI / 180.0f)) / PI) / 2.0f * n) - targetTileY;

        float centerSubX = ((centerLon + 180.0f) / 360.0f * n) - centerTileX;
        float centerSubY = ((1.0f - log(tan(centerLat * PI / 180.0f) + 1.0f / cos(centerLat * PI / 180.0f)) / PI) / 2.0f * n) - centerTileY;

        *pixelX = MAP_CANVAS_WIDTH / 2 + (int)(((targetTileX - centerTileX) + (subX - centerSubX)) * MAP_TILE_SIZE);
        *pixelY = MAP_CANVAS_HEIGHT / 2 + (int)(((targetTileY - centerTileY) + (subY - centerSubY)) * MAP_TILE_SIZE);
    }


    // Track map station click - stores callsign to prefill compose screen
    static String map_prefill_callsign = "";

/*    // Station click handler - opens compose screen with prefilled callsign
    void map_station_clicked(lv_event_t* e) {
        int stationIndex = (int)(intptr_t)lv_event_get_user_data(e);
        MapStation* station = STATION_Utils::getMapStation(stationIndex);

        if (station && station->valid && station->callsign.length() > 0) {
            Serial.printf("[MAP] Station clicked : %s\n", station->callsign.c_str());
            map_prefill_callsign = station->callsign;
            LVGL_UI::open_compose_with_callsign(station->callsign); // Call public function
        }
    }
*/
    // Map back button handler
    void btn_map_back_clicked(lv_event_t* e) {
        Serial.println("[LVGL] MAP BACK button pressed");
        destroy_station_pool();
        cleanup_station_buttons();  // Clean up station buttons when leaving map
        map_follow_gps = true;  // Reset to follow GPS when leaving map
        // Stop periodic refresh timer
        if (map_refresh_timer) {
            lv_timer_del(map_refresh_timer);
            map_refresh_timer = nullptr;
        }
        // Stop tile preload task
        stopTilePreloadTask();
        // Return CPU to 80 MHz for power saving
        setCpuFrequencyMhz(80);
        Serial.printf("[MAP] CPU reduced to %d MHz\n", getCpuFrequencyMhz());
        // Return to main dashboard screen
        LVGL_UI::return_to_dashboard();
    }

    // Map recenter button handler - return to GPS position
    void btn_map_recenter_clicked(lv_event_t* e) {
        Serial.println("[MAP] Recentering on GPS");
        map_follow_gps = true;
        if (gps.location.isValid()) {
            map_center_lat = gps.location.lat();
            map_center_lon = gps.location.lng();
            Serial.printf("[MAP] Recentered on GPS : %.4f, %.4f\n", map_center_lat, map_center_lon);
        } else {
            // No GPS - return to default Ariège position
            map_center_lat = 42.9667f;
            map_center_lon = 1.6053f;
            Serial.printf("[MAP] No GPS, recentered on default position : %.4f, %.4f\n", map_center_lat, map_center_lon);
        }
        schedule_map_reload();
    }

    // Redraw only canvas content without recreating screen (for zoom)
    void redraw_map_canvas() {
        if (!map_canvas || !map_canvas_buf || !map_title_label) {
            screen_map = nullptr; // Force recreation
            create_map_screen();
            lv_disp_load_scr(screen_map);
            return;
        }

        // Pause async preloading while we load tiles (avoid SD contention)
        mainThreadLoading = true;

        // Update title with new zoom level
        char title_text[32];
        snprintf(title_text, sizeof(title_text), "MAP (Z%d)", map_current_zoom);
        lv_label_set_text(map_title_label, title_text);

        // Clean up old station buttons before redrawing
        cleanup_station_buttons();

        // Clear canvas with dark slate gray background
        lv_canvas_fill_bg(map_canvas, lv_color_hex(0x2F4F4F), LV_OPA_COVER);

        // Recalculate tile positions
        int centerTileX, centerTileY;
        latLonToTile(map_center_lat, map_center_lon, map_current_zoom, &centerTileX, &centerTileY);

        int n = 1 << map_current_zoom;
        float tileXf = (map_center_lon + 180.0f) / 360.0f * n;
        float latRad = map_center_lat * PI / 180.0f;
        float tileYf = (1.0f - log(tan(latRad) + 1.0f / cos(latRad)) / PI) / 2.0f * n;

        float fracX = tileXf - centerTileX;
        float fracY = tileYf - centerTileY;
        int subTileOffsetX = (int)(fracX * MAP_TILE_SIZE);
        int subTileOffsetY = (int)(fracY * MAP_TILE_SIZE);

        Serial.printf("[MAP] Center tile: %d/%d, sub-tile offset: %d,%d\n", centerTileX, centerTileY, subTileOffsetX, subTileOffsetY);

        // Load tiles
        bool hasTiles = false;
        if (STORAGE_Utils::isSDAvailable()) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int tileX = centerTileX + dx;
                    int tileY = centerTileY + dy;
                    int offsetX = MAP_CANVAS_WIDTH / 2 - subTileOffsetX + dx * MAP_TILE_SIZE;
                    int offsetY = MAP_CANVAS_HEIGHT / 2 - subTileOffsetY + dy * MAP_TILE_SIZE;

                    if (dx == 0 && dy == 0) {
                        Serial.printf("[MAP] Center tile offset: %d,%d\n", offsetX, offsetY);
                    }

                    if (loadTileFromSD(tileX, tileY, map_current_zoom, map_canvas, offsetX, offsetY)) {
                        hasTiles = true;
                    }
                }
            }
        }

        if (!hasTiles) {
            lv_draw_label_dsc_t label_dsc;
            lv_draw_label_dsc_init(&label_dsc);
            label_dsc.color = lv_color_hex(0xaaaaaa);
            label_dsc.font = &lv_font_montserrat_14;
            lv_canvas_draw_text(map_canvas, 40, MAP_CANVAS_HEIGHT / 2 - 30, 240, &label_dsc,
                "No offline tiles available.");
        }

        // Update station LVGL objects (own position + received stations)
        update_station_objects();

        // Recenter canvas after drawing new tiles (avoids visual jump)
        lv_obj_set_pos(map_canvas, -MAP_CANVAS_MARGIN, -MAP_CANVAS_MARGIN);

        // Force container update (needed for touch pan to work)
        lv_obj_invalidate(map_container);

        // Resume async preloading
        mainThreadLoading = false;
    }

    // Timer callback to reload map screen (for panning/recentering)
    void map_reload_timer_cb(lv_timer_t* timer) {
        lv_timer_del(timer);
        redraw_map_canvas(); // Only canvas is redrawn, no need to recreate screen_map
    }

    // Helper function to schedule map reload with delay
    void schedule_map_reload() {
        lv_timer_t* t = lv_timer_create(map_reload_timer_cb, 20, NULL);
        lv_timer_set_repeat_count(t, 1);
    }

    // Map zoom in handler
    void btn_map_zoomin_clicked(lv_event_t* e) {
        if (map_zoom_index < map_zoom_count - 1) {
            map_zoom_index++;
            map_current_zoom = map_available_zooms[map_zoom_index];
            Serial.printf("[MAP] Zoom in: %d\n", map_current_zoom);
            redraw_map_canvas();
        }
    }

    // Map zoom out handler
    void btn_map_zoomout_clicked(lv_event_t* e) {
        if (map_zoom_index > 0) {
            map_zoom_index--;
            map_current_zoom = map_available_zooms[map_zoom_index];
            Serial.printf("[MAP] Zoom out: %d\n", map_current_zoom);
            redraw_map_canvas();
        }
    }

    // Calculate pan step based on zoom level (pixels to degrees)
    float getMapPanStep() {
        int n = 1 << map_current_zoom;
        // Move approximately 50 pixels at current zoom value
        return 50.0f / MAP_TILE_SIZE / n * 360.0f;
    }

    // Map panning handlers
    void btn_map_up_clicked(lv_event_t* e) {
        map_follow_gps = false;
        float step = getMapPanStep();
        map_center_lat += step;
        Serial.printf("[MAP] Pan up: %.4f, %.4f\n", map_center_lat, map_center_lon);
        schedule_map_reload();
    }

    void btn_map_down_clicked(lv_event_t* e) {
        map_follow_gps = false;
        float step = getMapPanStep();
        map_center_lat -= step;
        Serial.printf("[MAP] Pan down: %.4f, %.4f\n", map_center_lat, map_center_lon);
        schedule_map_reload();
    }

    void btn_map_left_clicked(lv_event_t* e) {
        map_follow_gps = false;
        float step = getMapPanStep();
        map_center_lon -= step;
        Serial.printf("[MAP] Pan left: %.4f, %.4f\n", map_center_lat, map_center_lon);
        schedule_map_reload();
    }

    void btn_map_right_clicked(lv_event_t* e) {
        map_follow_gps = false;
        float step = getMapPanStep();
        map_center_lon += step;
        Serial.printf("[MAP] Pan right: %.4f, %.4f\n", map_center_lat, map_center_lon);
        schedule_map_reload();
    }

    // Touch pan handler for finger drag on map
    void map_touch_event_cb(lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        lv_indev_t* indev = lv_indev_get_act();
        if (!indev) return;

        lv_point_t point;
        lv_indev_get_point(indev, &point);

        if (code == LV_EVENT_PRESSED) {
            // Finger down - start tracking
            touch_start_x = point.x;
            touch_start_y = point.y;
            last_pan_dx = 0;
            last_pan_dy = 0;
            drag_start_lat = map_center_lat;
            drag_start_lon = map_center_lon;
            touch_dragging = false;
            Serial.printf("[MAP] Touch PRESSED at %d,%d - start pos: %.4f, %.4f\n",
                          point.x, point.y, drag_start_lat, drag_start_lon);
        }
        else if (code == LV_EVENT_PRESSING) {
            // Finger moving - check if we should pan
            lv_coord_t dx = point.x - touch_start_x;
            lv_coord_t dy = point.y - touch_start_y;

            // Only start dragging if moved beyond threshold
            if (!touch_dragging && (abs(dx) > PAN_THRESHOLD || abs(dy) > PAN_THRESHOLD)) {
                touch_dragging = true;
                map_follow_gps = false;
                Serial.println("[MAP] Touch pan started");

                // Immediate preload: queue tiles in direction of movement
                if (tilePreloadQueue != nullptr) {
                    // Get current center tile
                    int centerTileX, centerTileY;
                    latLonToTile(map_center_lat, map_center_lon, map_current_zoom, &centerTileX, &centerTileY);

                    // Determine direction (drag left = need tiles on right, etc.)
                    int dir_x = (dx < 0) ? 1 : (dx > 0) ? -1 : 0;
                    int dir_y = (dy < 0) ? 1 : (dy > 0) ? -1 : 0;

                    TileRequest req;
                    req.zoom = map_current_zoom;

                    // Queue tiles in movement direction
                    for (int i = -1; i <= 1; i++) {
                        if (dir_x != 0) {
                            req.tileX = centerTileX + dir_x * 2;
                            req.tileY = centerTileY + i;
                            xQueueSend(tilePreloadQueue, &req, 0);
                        }
                        if (dir_y != 0) {
                            req.tileX = centerTileX + i;
                            req.tileY = centerTileY + dir_y * 2;
                            xQueueSend(tilePreloadQueue, &req, 0);
                        }
                    }
                    Serial.printf("[MAP] Preload queued for direction dx=%d dy=%d\n", dir_x, dir_y);
                }
            }

            // Live preview: move canvas within margin bounds
            if (touch_dragging && map_canvas) {
                // Canvas starts at (-MARGIN, -MARGIN), so new position is (-MARGIN + dx, -MARGIN + dy)
                lv_coord_t new_x = -MAP_CANVAS_MARGIN + dx;
                lv_coord_t new_y = -MAP_CANVAS_MARGIN + dy;

                if (abs(dx) > MAP_CANVAS_MARGIN - 10 || abs(dy) > MAP_CANVAS_MARGIN - 10) {
                    if (dx > MAP_CANVAS_MARGIN - 10) dx = MAP_CANVAS_MARGIN - 10;
                    if (dx < -(MAP_CANVAS_MARGIN - 10)) dx = -(MAP_CANVAS_MARGIN - 10);
                    if (dy > MAP_CANVAS_MARGIN - 10) dy = MAP_CANVAS_MARGIN - 10;
                    if (dy < -(MAP_CANVAS_MARGIN - 10)) dy = -(MAP_CANVAS_MARGIN - 10);

                    new_x = -MAP_CANVAS_MARGIN + dx;
                    new_y = -MAP_CANVAS_MARGIN + dy;
                }

                {
                    // Normal panning within margin
                    lv_obj_set_pos(map_canvas, new_x, new_y);
                    // Move station objects by incremental delta
                    lv_coord_t delta_dx = dx - last_pan_dx;
                    lv_coord_t delta_dy = dy - last_pan_dy;
                    moveStationDisplayObjects(delta_dx, delta_dy);
                    last_pan_dx = dx;
                    last_pan_dy = dy;
                }
            }
        }
        else if (code == LV_EVENT_RELEASED) {
            if (touch_dragging) {
                // Finger up after pan - finish pan
                touch_dragging = false;

                // Calculate final displacement from last drag_start position
                lv_coord_t dx = point.x - touch_start_x;
                lv_coord_t dy = point.y - touch_start_y;

                // Convert pixel movement to lat/lon change
                int n = 1 << map_current_zoom;
                float degrees_per_pixel = 360.0f / n / MAP_TILE_SIZE;

                // Update map center (drag right = view moves right = lon increases)
                map_center_lon = drag_start_lon - (dx * degrees_per_pixel);
                map_center_lat = drag_start_lat + (dy * degrees_per_pixel);

                Serial.printf("[MAP] Touch pan end: %.4f,%.4f -> %.4f,%.4f\n",
                              drag_start_lat, drag_start_lon, map_center_lat, map_center_lon);

                // Schedule redraw (canvas will be recentered after new tiles are drawn)
                schedule_map_reload();

                // Preload tiles at adjacent zoom levels for fast zoom switch
                int cX, cY;
                latLonToTile(map_center_lat, map_center_lon, map_current_zoom, &cX, &cY);
                queueAdjacentZoomTiles(cX, cY, map_current_zoom);
            } else {
                // Tap (no drag) - check if a station was tapped
                for (int i = 0; i < stationHitZoneCount; i++) {
                    int16_t hx = stationHitZones[i].x;
                    int16_t hy = stationHitZones[i].y;
                    int16_t hw = stationHitZones[i].w;
                    int16_t hh = stationHitZones[i].h;

                    // Check if tap is within hit zone (centered on station)
                    if (point.x >= hx - hw/2 && point.x <= hx + hw/2 &&
                        point.y >= hy - hh/2 && point.y <= hy + hh/2) {

                        int stationIdx = stationHitZones[i].stationIdx;
                        MapStation* station = STATION_Utils::getMapStation(stationIdx);
                        if (station && station->valid && station->callsign.length() > 0) {
                            Serial.printf("[MAP] Station tapped: %s\n", station->callsign.c_str());
                            LVGL_UI::open_compose_with_callsign(station->callsign);
                        }
                        break;
                    }
                }
            }
        }
    }

    // Discover the first available map region from the SD card
    static void discoverAndSetMapRegion() {
        if (!map_current_region.isEmpty()) {
            return; // Region is already set
        }

        Serial.println("[MAP] Map region not set, attempting to discover...");
        if (spiMutex != NULL && xSemaphoreTake(spiMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            if (STORAGE_Utils::isSDAvailable()) {
                File mapsDir = SD.open("/LoRa_Tracker/Maps");
                if (mapsDir && mapsDir.isDirectory()) {
                    File entry = mapsDir.openNextFile();
                    while(entry) {
                        if (entry.isDirectory()) {
                            String dirName = String(entry.name());
                            // Extract just the directory name from the full path
                            map_current_region = dirName.substring(dirName.lastIndexOf('/') + 1);
                            Serial.printf("[MAP] Discovered and set map region: %s\n", map_current_region.c_str());
                            entry.close();
                            break; // Use the first one we find
                        }
                        entry.close();
                        entry = mapsDir.openNextFile();
                    }
                } else {
                    Serial.println("[MAP] ERROR: Could not open /LoRa_Tracker/Maps directory.");
                }
                mapsDir.close();
            }
            xSemaphoreGive(spiMutex);
        } else {
            Serial.println("[MAP] ERROR: Could not get SPI Mutex for region discovery.");
        }

        if (map_current_region.isEmpty()) {
            Serial.println("[MAP] WARNING: No map region found on SD card.");
        }
    }

// Helper function to safely copy a sprite to the canvas with clipping
static void copySpriteToCanvasWithClip(lv_obj_t* canvas, TFT_eSprite* sprite, int offsetX, int offsetY) {
    if (!canvas || !sprite || sprite->frameBuffer(0) == nullptr) return;

    // Calculate source and destination rectangles for clipping
    int src_x = 0;
    int src_y = 0;
    int dest_x = offsetX;
    int dest_y = offsetY;
    int copy_w = MAP_TILE_SIZE;
    int copy_h = MAP_TILE_SIZE;

    // Clip left edge
    if (dest_x < 0) {
        src_x = -dest_x;
        copy_w += dest_x; // Decrease width
        dest_x = 0;
    }

    // Clip top edge
    if (dest_y < 0) {
        src_y = -dest_y;
        copy_h += dest_y; // Decrease height
        dest_y = 0;
    }

    // Clip right edge
    if (dest_x + copy_w > MAP_CANVAS_WIDTH) {
        copy_w = MAP_CANVAS_WIDTH - dest_x;
    }

    // Clip bottom edge
    if (dest_y + copy_h > MAP_CANVAS_HEIGHT) {
        copy_h = MAP_CANVAS_HEIGHT - dest_y;
    }

    // If there is anything to draw
    if (copy_w > 0 && copy_h > 0) {
        uint16_t* fb = (uint16_t*)sprite->frameBuffer(0);
        // Copy scanline by scanline to handle correct source buffer stride
        for (int y = 0; y < copy_h; y++) {
            uint16_t* src_row_ptr = fb + ((src_y + y) * MAP_TILE_SIZE) + src_x;
            lv_canvas_copy_buf(canvas, src_row_ptr, dest_x, dest_y + y, copy_w, 1);
        }
    }
}

bool loadTileFromSD(int tileX, int tileY, int zoom, lv_obj_t* canvas, int offsetX, int offsetY) {
    if (spiMutex == NULL) {
        Serial.println("[MAP] ERROR: spiMutex is NULL. Skipping SD access.");
        return false;
    }
    char path[128];
    char found_path[128] = {0};
    enum { TILE_NONE, TILE_NAV, TILE_PNG, TILE_JPG } found_type = TILE_NONE;
    bool tileRendered = false;
    TFT_eSprite* newSprite = nullptr;

    // --- 1. Check cache first ---
    int cacheIdx = findCachedTile(zoom, tileX, tileY);
    if (cacheIdx >= 0) {
        TFT_eSprite* cachedSprite = tileCache[cacheIdx].sprite;
        copySpriteToCanvasWithClip(canvas, cachedSprite, offsetX, offsetY);
        return true;
    }

    // --- 2. Check for a valid region before proceeding ---
    if (map_current_region.isEmpty()) {
        return false;
    }

    // --- 3. Find a valid tile file on SD card (with SPI Mutex) ---
    if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (STORAGE_Utils::isSDAvailable()) {
            const char* region = map_current_region.c_str();

            snprintf(path, sizeof(path), "/LoRa_Tracker/VectMaps/%s/%d/%d/%d.nav", region, zoom, tileX, tileY);
            File f = SD.open(path);
            if (f) { f.close(); strcpy(found_path, path); found_type = TILE_NAV; }
            else {
                snprintf(path, sizeof(path), "/LoRa_Tracker/Maps/%s/%d/%d/%d.png", region, zoom, tileX, tileY);
                f = SD.open(path);
                if (f) { f.close(); strcpy(found_path, path); found_type = TILE_PNG; }
                else {
                    snprintf(path, sizeof(path), "/LoRa_Tracker/Maps/%s/%d/%d/%d.jpg", region, zoom, tileX, tileY);
                    f = SD.open(path);
                    if (f) { f.close(); strcpy(found_path, path); found_type = TILE_JPG; }
                }
            }
        }
        xSemaphoreGive(spiMutex);
    } else {
        Serial.println("[MAP] ERROR: Could not get SPI Mutex for SD access");
    }

    // --- 4. If a file was found, create sprite and render it ---
    if (found_type != TILE_NONE) {
        Serial.printf("[MAP] Found file: %s\n", found_path);
        newSprite = new TFT_eSprite(&tft);
        newSprite->setAttribute(PSRAM_ENABLE, true);
        if (newSprite->createSprite(MAP_TILE_SIZE, MAP_TILE_SIZE) != nullptr && newSprite->frameBuffer(0) != nullptr) {
            if (spiMutex != NULL && xSemaphoreTake(spiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                switch (found_type) {
                    case TILE_NAV:
                        tileRendered = renderTile(found_path, tileX, tileY, zoom, 0, 0, *newSprite);
                        break;
                    case TILE_PNG:
                        spriteDecodeContext.sprite = newSprite;
                        if (png.open(found_path, pngOpenFile, pngCloseFile, pngReadFile, pngSeekFile, pngSpriteCallback) == PNG_SUCCESS && pngFileOpened) {
                            if (png.decode(NULL, 0) == PNG_SUCCESS) tileRendered = true;
                            png.close();
                        }
                        break;
                    case TILE_JPG:
                        spriteDecodeContext.sprite = newSprite;
                        if (jpeg.open(found_path, jpegOpenFile, jpegCloseFile, jpegReadFile, jpegSeekFile, jpegSpriteCallback) == 1) {
                            if (jpeg.decode(0, 0, 0) == 1) tileRendered = true;
                            jpeg.close();
                        }
                        break;
                    default: break;
                }
                xSemaphoreGive(spiMutex);
            } else {
                Serial.println("[MAP] ERROR: Could not get SPI Mutex for rendering");
            }
        } else {
            Serial.println("[MAP] ERROR: Sprite creation failed (Out of PSRAM?)");
            delete newSprite;
            return false;
        }
    }

    // --- 5. If rendering successful, update cache and draw on canvas ---
    if (tileRendered && newSprite) {
        copySpriteToCanvasWithClip(canvas, newSprite, offsetX, offsetY);
        addToCache(found_path, zoom, tileX, tileY, newSprite); // Cache takes ownership
    } else if (newSprite) {
        // Cleanup if rendering failed or sprite wasn't added to cache
        newSprite->deleteSprite();
        delete newSprite;
    }
    
    return tileRendered;
}

    // Create map screen
    void create_map_screen() {
        // Boost CPU to 240 MHz for smooth map rendering
        setCpuFrequencyMhz(240);
        Serial.printf("[MAP] CPU boosted to %d MHz\n", getCpuFrequencyMhz());

        // Discover and set the map region if it's not already defined
        discoverAndSetMapRegion();

        initBatchRendering();
        if (!map_current_region.isEmpty()) {
            String palettePath = "/LoRa_Tracker/VectMaps/" + map_current_region + "/palette.bin";
            loadPalette(palettePath.c_str());
        } else {
            loadPalette(nullptr); // Will trigger fallback
        }

        // Clean up old station buttons if screen is being recreated
        cleanup_station_buttons();

        screen_map = lv_obj_create(NULL);
        lv_obj_set_style_bg_color(screen_map, lv_color_hex(0x1a1a2e), 0);

        // Use current GPS position as center if follow mode is active
        if (map_follow_gps && gps.location.isValid()) {
            map_center_lat = gps.location.lat();
            map_center_lon = gps.location.lng();
            Serial.printf("[MAP] Using GPS position: %.4f, %.4f\n", map_center_lat, map_center_lon);
        } else if (map_center_lat == 0.0f && map_center_lon == 0.0f) {
            // Default to Ariège (Foix) if no GPS - matches OCC tiles
            map_center_lat = 42.9667f;
            map_center_lon = 1.6053f;
            Serial.printf("[MAP] No GPS, using default Ariège position: %.4f, %.4f\n", map_center_lat, map_center_lon);
        } else {
            Serial.printf("[MAP] Using pan position: %.4f, %.4f\n", map_center_lat, map_center_lon);
        }

        // Title bar (green for map)
        lv_obj_t* title_bar = lv_obj_create(screen_map);
        lv_obj_set_size(title_bar, SCREEN_WIDTH, 35);
        lv_obj_set_pos(title_bar, 0, 0);
        lv_obj_set_style_bg_color(title_bar, lv_color_hex(0x009933), 0);
        lv_obj_set_style_border_width(title_bar, 0, 0);
        lv_obj_set_style_radius(title_bar, 0, 0);
        lv_obj_set_style_pad_all(title_bar, 5, 0);

        // Back button
        lv_obj_t* btn_back = lv_btn_create(title_bar);
        lv_obj_set_size(btn_back, 60, 25);
        lv_obj_set_style_bg_color(btn_back, lv_color_hex(0x16213e), 0);
        lv_obj_add_event_cb(btn_back, btn_map_back_clicked, LV_EVENT_CLICKED, NULL);
        lv_obj_t* lbl_back = lv_label_create(btn_back);
        lv_label_set_text(lbl_back, "< BACK");
        lv_obj_center(lbl_back);

        // Title with zoom level (keep reference for updates)
        map_title_label = lv_label_create(title_bar);
        char title_text[32];
        snprintf(title_text, sizeof(title_text), "MAP (Z%d)", map_current_zoom);
        lv_label_set_text(map_title_label, title_text);
        lv_obj_set_style_text_color(map_title_label, lv_color_hex(0xffffff), 0);
        lv_obj_set_style_text_font(map_title_label, &lv_font_montserrat_18, 0);
        lv_obj_align(map_title_label, LV_ALIGN_CENTER, -30, 0);

        // Zoom buttons on right
        lv_obj_t* btn_zoomin = lv_btn_create(title_bar);
        lv_obj_set_size(btn_zoomin, 30, 25);
        lv_obj_set_style_bg_color(btn_zoomin, lv_color_hex(0x16213e), 0);
        lv_obj_align(btn_zoomin, LV_ALIGN_RIGHT_MID, -70, 0);
        lv_obj_add_event_cb(btn_zoomin, btn_map_zoomin_clicked, LV_EVENT_RELEASED, NULL);
        lv_obj_t* lbl_zoomin = lv_label_create(btn_zoomin);
        lv_label_set_text(lbl_zoomin, "+");
        lv_obj_center(lbl_zoomin);

        lv_obj_t* btn_zoomout = lv_btn_create(title_bar);
        lv_obj_set_size(btn_zoomout, 30, 25);
        lv_obj_set_style_bg_color(btn_zoomout, lv_color_hex(0x16213e), 0);
        lv_obj_align(btn_zoomout, LV_ALIGN_RIGHT_MID, -35, 0);
        lv_obj_add_event_cb(btn_zoomout, btn_map_zoomout_clicked, LV_EVENT_RELEASED, NULL);
        lv_obj_t* lbl_zoomout = lv_label_create(btn_zoomout);
        lv_label_set_text(lbl_zoomout, "-");
        lv_obj_center(lbl_zoomout);

        // Recenter button (GPS icon) - shows different color when GPS not followed
        lv_obj_t* btn_recenter = lv_btn_create(title_bar);
        lv_obj_set_size(btn_recenter, 30, 25);
        lv_obj_set_style_bg_color(btn_recenter, map_follow_gps ? lv_color_hex(0x16213e) : lv_color_hex(0xff6600), 0);
        lv_obj_align(btn_recenter, LV_ALIGN_RIGHT_MID, 0, 0);
        lv_obj_add_event_cb(btn_recenter, btn_map_recenter_clicked, LV_EVENT_CLICKED, NULL);
        lv_obj_t* lbl_recenter = lv_label_create(btn_recenter);
        lv_label_set_text(lbl_recenter, LV_SYMBOL_GPS);
        lv_obj_center(lbl_recenter);

        // Map canvas area (container clips the larger canvas to visible area)
        map_container = lv_obj_create(screen_map);
        lv_obj_set_size(map_container, SCREEN_WIDTH, MAP_VISIBLE_HEIGHT);
        lv_obj_set_pos(map_container, 0, 35);
        lv_obj_set_style_bg_color(map_container, lv_color_hex(0x2F4F4F), 0);  // Dark slate gray
        lv_obj_set_style_border_width(map_container, 0, 0);
        lv_obj_set_style_radius(map_container, 0, 0);
        lv_obj_set_style_pad_all(map_container, 0, 0);
        lv_obj_clear_flag(map_container, LV_OBJ_FLAG_SCROLLABLE);  // Force clipping of children

        // Enable touch pan on map container
        lv_obj_add_flag(map_container, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(map_container, map_touch_event_cb, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(map_container, map_touch_event_cb, LV_EVENT_PRESSING, NULL);
        lv_obj_add_event_cb(map_container, map_touch_event_cb, LV_EVENT_RELEASED, NULL);

        // Create canvas for map drawing
        // Free old buffer if it exists (memory leak prevention)
        if (map_canvas_buf) {
            heap_caps_free(map_canvas_buf);
            map_canvas_buf = nullptr;
        }
        map_canvas_buf = (lv_color_t*)heap_caps_malloc(MAP_CANVAS_WIDTH * MAP_CANVAS_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
        if (map_canvas_buf) {
            map_canvas = lv_canvas_create(map_container);
            lv_canvas_set_buffer(map_canvas, map_canvas_buf, MAP_CANVAS_WIDTH, MAP_CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
            // Position canvas with negative margin so visible area is centered
            lv_obj_set_pos(map_canvas, -MAP_CANVAS_MARGIN, -MAP_CANVAS_MARGIN);

            // Fill with background color
            lv_canvas_fill_bg(map_canvas, lv_color_hex(0x2F4F4F), LV_OPA_COVER);

            // Calculate center tile and fractional position within tile
            int centerTileX, centerTileY;
            latLonToTile(map_center_lat, map_center_lon, map_current_zoom, &centerTileX, &centerTileY);

            // Calculate sub-tile offset (where our center point is within the tile)
            int n = 1 << map_current_zoom;
            float tileXf = (map_center_lon + 180.0f) / 360.0f * n;
            float latRad = map_center_lat * PI / 180.0f;
            float tileYf = (1.0f - log(tan(latRad) + 1.0f / cos(latRad)) / PI) / 2.0f * n;

            // Fractional part (0.0 to 1.0) represents position within tile
            float fracX = tileXf - centerTileX;
            float fracY = tileYf - centerTileY;

            // Convert to pixel offset (how many pixels to shift tiles)
            int subTileOffsetX = (int)(fracX * MAP_TILE_SIZE);
            int subTileOffsetY = (int)(fracY * MAP_TILE_SIZE);

            Serial.printf("[MAP] Center tile: %d/%d, sub-tile offset: %d,%d\n", centerTileX, centerTileY, subTileOffsetX, subTileOffsetY);

            // Pause async preloading while we load tiles (avoid SD contention)
            mainThreadLoading = true;

            // Try to load tiles from SD card
            bool hasTiles = false;
            if (STORAGE_Utils::isSDAvailable()) {
                // Load center tile and surrounding tiles (3x3 grid, or more if needed)
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int tileX = centerTileX + dx;
                        int tileY = centerTileY + dy;
                        // Apply sub-tile offset so center point is at screen center
                        int offsetX = MAP_CANVAS_WIDTH / 2 - subTileOffsetX + dx * MAP_TILE_SIZE;
                        int offsetY = MAP_CANVAS_HEIGHT / 2 - subTileOffsetY + dy * MAP_TILE_SIZE;

                        if (dx == 0 && dy == 0) {
                            Serial.printf("[MAP] Center tile offset: %d,%d\n", offsetX, offsetY);
                        }

                        if (loadTileFromSD(tileX, tileY, map_current_zoom, map_canvas, offsetX, offsetY)) {
                            hasTiles = true;
                        }
                    }
                }
            }

            if (!hasTiles) {
                // No tiles - display message
                lv_draw_label_dsc_t label_dsc;
                lv_draw_label_dsc_init(&label_dsc);
                label_dsc.color = lv_color_hex(0xaaaaaa);
                label_dsc.font = &lv_font_montserrat_14;
                lv_canvas_draw_text(map_canvas, 40, MAP_CANVAS_HEIGHT / 2 - 30, 240, &label_dsc,
                    "No offline tiles available.\nDownload OSM tiles and copy to:\nSD:/LoRa_Tracker/Maps/REGION/z/x/y.png");
            }

            // Update station LVGL objects (own position + received stations)
            update_station_objects();

            // Force canvas redraw after direct buffer writes
            lv_canvas_set_buffer(map_canvas, map_canvas_buf, MAP_CANVAS_WIDTH, MAP_CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
            lv_obj_invalidate(map_canvas);

            // Resume async preloading
            mainThreadLoading = false;
        }

        // Arrow buttons for panning (touch pan still unstable)
#if 1
        int arrow_size = 28;
        int arrow_x = 5;
        int arrow_y = MAP_VISIBLE_HEIGHT - 105;  // Above info bar
        lv_color_t arrow_color = lv_color_hex(0x444444);

        // Up button
        lv_obj_t* btn_up = lv_btn_create(map_container);
        lv_obj_set_size(btn_up, arrow_size, arrow_size);
        lv_obj_set_pos(btn_up, arrow_x + arrow_size, arrow_y);
        lv_obj_set_style_bg_color(btn_up, arrow_color, 0);
        lv_obj_set_style_bg_opa(btn_up, 90, 0);  // 45% opacity
        lv_obj_add_event_cb(btn_up, btn_map_up_clicked, LV_EVENT_CLICKED, NULL);
        lv_obj_t* lbl_up = lv_label_create(btn_up);
        lv_label_set_text(lbl_up, LV_SYMBOL_UP);
        lv_obj_center(lbl_up);

        // Down button
        lv_obj_t* btn_down = lv_btn_create(map_container);
        lv_obj_set_size(btn_down, arrow_size, arrow_size);
        lv_obj_set_pos(btn_down, arrow_x + arrow_size, arrow_y + arrow_size * 2);
        lv_obj_set_style_bg_color(btn_down, arrow_color, 0);
        lv_obj_set_style_bg_opa(btn_down, 90, 0);  // 45% opacity
        lv_obj_add_event_cb(btn_down, btn_map_down_clicked, LV_EVENT_CLICKED, NULL);
        lv_obj_t* lbl_down = lv_label_create(btn_down);
        lv_label_set_text(lbl_down, LV_SYMBOL_DOWN);
        lv_obj_center(lbl_down);

        // Left button
        lv_obj_t* btn_left = lv_btn_create(map_container);
        lv_obj_set_size(btn_left, arrow_size, arrow_size);
        lv_obj_set_pos(btn_left, arrow_x, arrow_y + arrow_size);
        lv_obj_set_style_bg_color(btn_left, arrow_color, 0);
        lv_obj_set_style_bg_opa(btn_left, 90, 0);  // 45% opacity
        lv_obj_add_event_cb(btn_left, btn_map_left_clicked, LV_EVENT_CLICKED, NULL);
        lv_obj_t* lbl_left = lv_label_create(btn_left);
        lv_label_set_text(lbl_left, LV_SYMBOL_LEFT);
        lv_obj_center(lbl_left);

        // Right button
        lv_obj_t* btn_right = lv_btn_create(map_container);
        lv_obj_set_size(btn_right, arrow_size, arrow_size);
        lv_obj_set_pos(btn_right, arrow_x + arrow_size * 2, arrow_y + arrow_size);
        lv_obj_set_style_bg_color(btn_right, arrow_color, 0);
        lv_obj_set_style_bg_opa(btn_right, 90, 0);  // 45% opacity
        lv_obj_add_event_cb(btn_right, btn_map_right_clicked, LV_EVENT_CLICKED, NULL);
        lv_obj_t* lbl_right = lv_label_create(btn_right);
        lv_label_set_text(lbl_right, LV_SYMBOL_RIGHT);
        lv_obj_center(lbl_right);
#endif

        // Info bar at bottom
        lv_obj_t* info_bar = lv_obj_create(screen_map);
        lv_obj_set_size(info_bar, SCREEN_WIDTH, 25);
        lv_obj_set_pos(info_bar, 0, SCREEN_HEIGHT - 25);
        lv_obj_set_style_bg_color(info_bar, lv_color_hex(0x16213e), 0);
        lv_obj_set_style_border_width(info_bar, 0, 0);
        lv_obj_set_style_radius(info_bar, 0, 0);
        lv_obj_set_style_pad_all(info_bar, 2, 0);

        // Display coordinates
        lv_obj_t* lbl_coords = lv_label_create(info_bar);
        char coords_text[64];
        snprintf(coords_text, sizeof(coords_text), "Center: %.4f, %.4f  Stations: %d",
                 map_center_lat, map_center_lon, mapStationsCount);
        lv_label_set_text(lbl_coords, coords_text);
        lv_obj_set_style_text_color(lbl_coords, lv_color_hex(0xaaaaaa), 0);
        lv_obj_set_style_text_font(lbl_coords, &lv_font_montserrat_14, 0);
        lv_obj_center(lbl_coords);

        // Create periodic refresh timer for stations (10 seconds)
        if (map_refresh_timer) {
            lv_timer_del(map_refresh_timer);
        }
        map_refresh_timer = lv_timer_create(map_refresh_timer_cb, MAP_REFRESH_INTERVAL, NULL);

        // Start tile preload task on Core 1 for directional preloading during touch pan
        startTilePreloadTask();

        Serial.println("[LVGL] Map screen created");
    }


} // namespace UIMapManager

#endif // USE_LVGL_UI
