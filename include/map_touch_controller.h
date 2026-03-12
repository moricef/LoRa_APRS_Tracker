#ifndef MAP_TOUCH_CONTROLLER_H
#define MAP_TOUCH_CONTROLLER_H

#include <cstdint>
#include <functional>
#include <lvgl.h>
#include "ui_map_manager.h"  // Pour les vrais #define constants

class MapTouchController {
public:
    // Callback types
    using RedrawCallback = std::function<void()>;
    using ScrollCallback = std::function<void(int16_t dx, int16_t dy, bool breakFollowGps)>;
    using ShiftTileCallback = std::function<void(int dX, int dY)>;
    using DoubleTapCallback = std::function<void()>;
    using GetCenterTileCallback = std::function<void(int& x, int& y)>;

    MapTouchController(
        RedrawCallback redraw_cb = nullptr,
        ScrollCallback scroll_cb = nullptr,
        ShiftTileCallback shift_tile_cb = nullptr,
        DoubleTapCallback double_tap_cb = nullptr,
        GetCenterTileCallback get_center_tile_cb = nullptr
    );

    void reset();
    void handleTouchEvent(lv_event_t* e);
    bool updateInertia(bool redraw_in_progress);  // Retourne true si mouvement
    
    // Getters/setters
    void getOffsets(int16_t& x, int16_t& y) const;
    void getNavSubTileOffsets(int16_t& x, int16_t& y) const;
    void setNavSubTileOffsets(int16_t x, int16_t y);
    void getVelocity(float& vx, float& vy) const;
    void setVelocity(float vx, float vy);
    
    bool isScrolling() const { return isScrollingMap; }
    bool isDragStarted() const { return dragStarted; }
    bool hasPendingResetPan() const { return pendingResetPan; }
    void setPendingResetPan(bool pending);
    void resetPanOffsets();

private:
    // État original
    bool isScrollingMap = false;
    bool dragStarted = false;
    int16_t offsetX = 0;
    int16_t offsetY = 0;
    int16_t navSubTileX = 0;
    int16_t navSubTileY = 0;
    float velocityX = 0.0f;
    float velocityY = 0.0f;
    int last_x = 0;
    int last_y = 0;
    uint32_t last_time = 0;
    bool pendingResetPan = false;

    // Double-tap
    uint32_t firstTapTime = 0;
    uint8_t tapCount = 0;

    // Callbacks
    RedrawCallback redrawCallback;
    ScrollCallback scrollCallback;
    ShiftTileCallback shiftTileCallback;
    DoubleTapCallback doubleTapCallback;
    GetCenterTileCallback getCenterTileCallback;

    // Méthodes internes
    bool scroll(int16_t dx, int16_t dy);  // Interne, utilise getCenterTileCallback
    uint32_t getCurrentTimeMs() const;
    
    template<typename T>
    static T clampValue(T value, T min, T max) {
        return (value < min) ? min : ((value > max) ? max : value);
    }
};

#endif // MAP_TOUCH_CONTROLLER_H