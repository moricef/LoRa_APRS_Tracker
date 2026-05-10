#pragma once

#ifdef USE_LVGL_UI

#include <stdint.h>

// Display Hardware Abstraction Layer.
//
// One implementation per target in src/display/, selected by build_src_filter
// in the variant's platformio.ini. Encapsulates panel hardware init, LVGL
// display driver registration, and touch read. The rest of the codebase
// stays platform-agnostic and goes through this interface.
//
// Phase 1 status: only Waveshare 7" RGB is implemented (direct framebuffer
// mode). T-Deck Plus and Crowpanel still use the legacy paths in lvgl_ui.cpp.

namespace DisplayHAL {

    // Init panel hardware + LVGL display driver. Returns true on success.
    // Must be called once after lv_init().
    bool init();

    // Read touch coordinates. Returns true if a touch is currently active.
    // Coordinates in screen pixels.
    bool readTouch(uint16_t* x, uint16_t* y);

}

#endif // USE_LVGL_UI
