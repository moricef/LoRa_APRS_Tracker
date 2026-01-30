/* Map logic for T-Deck Plus
 * Offline map tiles display with stations using LVGL
 */

#ifndef UI_MAP_MANAGER_H
#define UI_MAP_MANAGER_H

#ifdef USE_LVGL_UI

#include <lvgl.h>
#include <Arduino.h> // Pour String, millis, etc.
#include <TinyGPS++.h> // Pour les données GPS
#include <TFT_eSPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>
#include "esp_heap_caps.h"

// Forward declarations
class Configuration;

// External data sources from lvgl_ui.cpp and other global variables
extern TinyGPSPlus gps;
extern Configuration Config;
extern TFT_eSPI tft;
extern uint8_t myBeaconsIndex;
extern int mapStationsCount;
extern SemaphoreHandle_t spiMutex; // Declared extern for SPI bus mutex access

// Dimensions de l'affichage
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240

namespace UIMapManager {

    /**
     * @brief Draw command types used for rendering vector graphics.
     */
    enum DrawCommand : uint8_t 
    {
        DRAW_LINE = 1, DRAW_POLYLINE = 2, DRAW_STROKE_POLYGON = 3, DRAW_STROKE_POLYGONS = 4,
        DRAW_HORIZONTAL_LINE = 5, DRAW_VERTICAL_LINE = 6, SET_COLOR = 0x80, SET_COLOR_INDEX = 0x81,
        RECTANGLE = 0x82, STRAIGHT_LINE = 0x83, HIGHWAY_SEGMENT = 0x84, GRID_PATTERN = 0x85,
        BLOCK_PATTERN = 0x86, CIRCLE = 0x87, SET_LAYER = 0x88, RELATIVE_MOVE = 0x89,
        PREDICTED_LINE = 0x8A, COMPRESSED_POLYLINE = 0x8B, OPTIMIZED_POLYGON = 0x8C,
        HOLLOW_POLYGON = 0x8D, OPTIMIZED_TRIANGLE = 0x8E, OPTIMIZED_RECTANGLE = 0x8F,
        OPTIMIZED_CIRCLE = 0x90, SIMPLE_RECTANGLE = 0x96, SIMPLE_CIRCLE = 0x97,
        SIMPLE_TRIANGLE = 0x98, DASHED_LINE = 0x99, DOTTED_LINE = 0x9A,
    };
    
    struct MapTile
    {
        char file[255];
        uint32_t tilex;
        uint32_t tiley;
        uint8_t zoom;
        float lat;
        float lon;
    };
    
    struct CachedTile
    {
        TFT_eSprite* sprite;
        uint32_t tileHash;
        uint32_t lastAccess;
        bool isValid;
        char filePath[255];
    };
    
    struct CachedSymbol {
        char table;              // '/' for primary, '\' for alternate
        char symbol;             // ASCII character
        lv_img_dsc_t img_dsc;   // LVGL image descriptor (RGB565A8 format)
        uint8_t* data;           // Combined RGB565+Alpha buffer in PSRAM
        uint32_t lastAccess;     // For LRU eviction
        bool valid;
    };
    
    struct LineSegment { int x0, y0, x1, y1; uint16_t color; };
    
    struct RenderBatch
    {
        LineSegment* segments;
        size_t count;
        size_t capacity;
        uint16_t color;
    };

    struct tileBounds
    {
        float lat_min; float lat_max;
        float lon_min; float lon_max;
    };

    // APRS symbol arrays
    extern const char* const* symbolArray;
    extern const int& symbolArraySize;
    extern const uint8_t* const* symbolsAPRS;

    // Map constants
    #define MAP_TILE_SIZE 256
    #define MAP_CANVAS_WIDTH   600
    #define MAP_CANVAS_HEIGHT  520
    #define MAP_VISIBLE_WIDTH 320  // Visible area on screen
    #define MAP_VISIBLE_HEIGHT 200
    #define MAP_CANVAS_MARGIN 128  // Margin on each side (half tile)

    // UI elements - Map screen
    extern lv_obj_t* screen_map;
    extern lv_obj_t* map_canvas;
    extern lv_color_t* map_canvas_buf;
    extern lv_obj_t* map_title_label;  // Title label to update zoom level
    extern lv_obj_t* map_container;    // Container for canvas and station buttons

    // Map state variables
    extern int map_zoom_index;  // Index in map_available_zooms (starts at zoom 8)
    extern int map_current_zoom;
    extern float map_center_lat;
    extern float map_center_lon;
    extern String map_current_region;
    extern bool map_follow_gps;  // Follow GPS or free panning mode

    // Unified memory pool methods
    void initUnifiedPool();
    void* unifiedAlloc(size_t size, uint8_t type = 0);
    void unifiedDealloc(void* ptr);

    // RAII Memory Guard for automatic memory management
    template<typename T>
    class MemoryGuard
    {
        private:
            T* ptr;
            size_t size;
            uint8_t type;
            bool fromPool;
            
        public:
            MemoryGuard(size_t numElements, uint8_t poolType = 0) 
                : ptr(nullptr), size(numElements * sizeof(T)), type(poolType), fromPool(false)
            {
                ptr = static_cast<T*>(UIMapManager::unifiedAlloc(size, type));
                if (ptr) 
                    fromPool = true;
                else 
                {
                    #ifdef BOARD_HAS_PSRAM
                        ptr = static_cast<T*>(heap_caps_malloc(size, MALLOC_CAP_SPIRAM));
                    #else
                        ptr = static_cast<T*>(heap_caps_malloc(size, MALLOC_CAP_8BIT));
                    #endif
                    fromPool = false;
                }
            }
            
            ~MemoryGuard()
            {
                if (ptr) 
                {
                    if (fromPool)
                        UIMapManager::unifiedDealloc(ptr);
                    else 
                        heap_caps_free(ptr);
                }
            }
            
            T* get() const { return ptr; }
            T& operator*() const { return *ptr; }
            T* operator->() const { return ptr; }
            operator bool() const { return ptr != nullptr; }
            
            MemoryGuard(const MemoryGuard&) = delete;
            MemoryGuard& operator=(const MemoryGuard&) = delete;
    };

    // Function declarations
    void initTileCache();
    void clearTileCache();
    void initBatchRendering();
    bool renderTile(const char* path, int16_t xOffset, int16_t yOffset, TFT_eSprite &map);
    bool loadPalette(const char* palettePath);
    int findCachedTile(int zoom, int tileX, int tileY);
    int findCacheSlot();
    // void copyTileToCanvas(uint16_t* tileData, lv_color_t* canvasBuffer,
    //                              int offsetX, int offsetY, int canvasWidth, int canvasHeight);
    void latLonToTile(float lat, float lon, int zoom, int* tileX, int* tileY);
    void latLonToPixel(float lat, float lon, float centerLat, float centerLon, int zoom, int* pixelX, int* pixelY);
    CachedSymbol* getSymbolCacheEntry(char table, char symbol);
    void map_station_clicked(lv_event_t* e);
    void btn_map_back_clicked(lv_event_t* e);
    void btn_map_recenter_clicked(lv_event_t* e);
    bool loadTileFromSD(int tileX, int tileY, int zoom, lv_obj_t* canvas, int offsetX, int offsetY);
    void redraw_map_canvas();
    void map_reload_timer_cb(lv_timer_t* timer);
    void schedule_map_reload();
    void btn_map_zoomin_clicked(lv_event_t* e);
    void btn_map_zoomout_clicked(lv_event_t* e);
    float getMapPanStep();
    void btn_map_up_clicked(lv_event_t* e);
    void btn_map_down_clicked(lv_event_t* e);
    void btn_map_left_clicked(lv_event_t* e);
    void btn_map_right_clicked(lv_event_t* e);
    void create_map_screen();

} // namespace UIMapManager

#endif // USE_LVGL_UI
#endif // UI_MAP_MANAGER_H
