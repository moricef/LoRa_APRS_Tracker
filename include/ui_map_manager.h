/* Map logic for T-Deck Plus
 * Offline map tiles display with stations using LVGL
 */

#ifndef UI_MAP_MANAGER_H
#define UI_MAP_MANAGER_H

#ifdef USE_LVGL_UI

#include <lvgl.h>
#include <Arduino.h>
#include <NMEAGPS.h>
#if defined(CROWPANEL_ADVANCE_35)
#include "LGFX_CrowPanel_35.h"
#elif defined(WAVESHARE_S3_TOUCH_LCD_7)
#include "LGFX_Waveshare7.h"
#else
#include "LGFX_TDeck.h"
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "map_state.h"
#include "map_engine.h"

// Forward declarations
class Configuration;

// External data sources from lvgl_ui.cpp and other global variables
extern gps_fix gpsFix;
extern Configuration Config;
// extern LGFX_TDeck tft;
extern uint8_t myBeaconsIndex;
extern int mapStationsCount;
extern SemaphoreHandle_t spiMutex; // Declared extern for SPI bus mutex access

// Map constants
#define MAP_TILE_SIZE 256

// Screen dimensions + tiles grid (per board)
#if defined(WAVESHARE_S3_TOUCH_LCD_7)
#define SCREEN_WIDTH  800
#define SCREEN_HEIGHT 480
#define MAP_VISIBLE_WIDTH  768             // sprite width = 3×256, 16px margin each side of screen
#define MAP_VISIBLE_HEIGHT (480 - 35 - 25)
#define MAP_TILES_GRID     3
#elif defined(CROWPANEL_ADVANCE_35)
#define SCREEN_WIDTH  480
#define SCREEN_HEIGHT 320
#define MAP_VISIBLE_WIDTH  320
#define MAP_VISIBLE_HEIGHT 200
#define MAP_TILES_GRID     3
#else
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240
#define MAP_VISIBLE_WIDTH  320
#define MAP_VISIBLE_HEIGHT 200
#define MAP_TILES_GRID     3
#endif

#define MAP_SPRITE_SIZE    (MAP_TILES_GRID * MAP_TILE_SIZE)
#define MAP_MARGIN_X  ((MAP_SPRITE_SIZE - MAP_VISIBLE_WIDTH) / 2)
#define MAP_MARGIN_Y  ((MAP_SPRITE_SIZE - MAP_VISIBLE_HEIGHT) / 2)

namespace UIMapManager {

    // Functions remaining in ui_map_manager.cpp (glue)
    // State variables now in MapState namespace (map_state.h)
    void redraw_map_canvas();
    void create_map_screen();
    void addOwnTracePoint();

} // namespace UIMapManager

#endif // USE_LVGL_UI
#endif // UI_MAP_MANAGER_H
