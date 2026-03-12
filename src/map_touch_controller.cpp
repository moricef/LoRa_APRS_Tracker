#include "map_touch_controller.h"
#include <esp_timer.h>
#include <esp_log.h>
#include <cmath>
#include <algorithm>

static const char* TAG = "TouchController";

MapTouchController::MapTouchController(
    RedrawCallback redraw_cb,
    ScrollCallback scroll_cb,
    ShiftTileCallback shift_tile_cb,
    DoubleTapCallback double_tap_cb,
    GetCenterTileCallback get_center_tile_cb
) : redrawCallback(redraw_cb),
    scrollCallback(scroll_cb),
    shiftTileCallback(shift_tile_cb),
    doubleTapCallback(double_tap_cb),
    getCenterTileCallback(get_center_tile_cb) {
    ESP_LOGD(TAG, "Touch controller initialized");
}

void MapTouchController::reset() {
    isScrollingMap = false;
    dragStarted = false;
    offsetX = 0;
    offsetY = 0;
    navSubTileX = 0;
    navSubTileY = 0;
    velocityX = 0.0f;
    velocityY = 0.0f;
    last_x = 0;
    last_y = 0;
    last_time = 0;
    pendingResetPan = false;
    firstTapTime = 0;
    tapCount = 0;
    ESP_LOGD(TAG, "Touch controller reset");
}

void MapTouchController::handleTouchEvent(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t* indev = lv_indev_get_act();
    if (!indev) return;

    lv_point_t p;
    lv_indev_get_point(indev, &p);

    switch (code) {
    case LV_EVENT_PRESSED:
        last_x = p.x;
        last_y = p.y;
        last_time = getCurrentTimeMs();
        dragStarted = false;
        isScrollingMap = true;
        velocityX = 0.0f;
        velocityY = 0.0f;
        ESP_LOGV(TAG, "Touch pressed at (%d,%d)", p.x, p.y);
        break;

    case LV_EVENT_PRESSING: {
        uint32_t current_time = getCurrentTimeMs();
        int dx = p.x - last_x;
        int dy = p.y - last_y;
        uint32_t dt = current_time - last_time;

        if (!dragStarted) {
            if (abs(dx) > START_THRESHOLD || abs(dy) > START_THRESHOLD) {
                dragStarted = true;
                pendingResetPan = false;
                ESP_LOGV(TAG, "Drag started (dx=%d, dy=%d)", dx, dy);
            }
        }

        if (dragStarted && dt > 0) {
            // ORIGINAL: scrollMap(-dx, -dy)
            scroll(-dx, -dy);

            // ORIGINAL: velocity = -dx/dt avec filtre exponentiel
            float weight = 0.7f;
            velocityX = velocityX * (1.0f - weight) + (-(float)dx / (float)dt) * weight;
            velocityY = velocityY * (1.0f - weight) + (-(float)dy / (float)dt) * weight;

            last_x = p.x;
            last_y = p.y;
            last_time = current_time;
        }
        break;
    }

    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST: {
        bool wasDragging = dragStarted;
        isScrollingMap = false;
        dragStarted = false;

        // ORIGINAL: kill very low velocity
        if (fabsf(velocityX) < 0.1f) velocityX = 0.0f;
        if (fabsf(velocityY) < 0.1f) velocityY = 0.0f;

        // ORIGINAL: double-tap 300ms window
        if (!wasDragging) {
            uint32_t now = getCurrentTimeMs();
            if (now - firstTapTime > 300) {
                tapCount = 1;
                firstTapTime = now;
            } else {
                tapCount++;
                if (tapCount >= 2) {
                    if (doubleTapCallback) doubleTapCallback();
                    tapCount = 0;
                    firstTapTime = 0;
                }
            }
        }
        ESP_LOGV(TAG, "Touch released, velocity=(%.2f,%.2f)", velocityX, velocityY);
        break;
    }

    default:
        break;
    }
}

bool MapTouchController::updateInertia(bool redraw_in_progress) {
    if (!isScrollingMap && (velocityX != 0.0f || velocityY != 0.0f)) {
        uint32_t now = getCurrentTimeMs();
        uint32_t dt = (last_time > 0) ? (now - last_time) : 0;
        last_time = now;
        
        if (dt > 0 && dt < 100) {
            int16_t dx = (int16_t)(velocityX * dt);
            int16_t dy = (int16_t)(velocityY * dt);
            
            if (dx != 0 || dy != 0) {
                scroll(dx, dy);
                
                float friction = redraw_in_progress ? PAN_FRICTION_BUSY : PAN_FRICTION;
                velocityX *= friction;
                velocityY *= friction;

                if (fabsf(velocityX) < 0.01f) velocityX = 0.0f;
                if (fabsf(velocityY) < 0.01f) velocityY = 0.0f;
                
                return true;  // Inertia caused movement
            }
        }
    }
    return false;
}

bool MapTouchController::scroll(int16_t dx, int16_t dy) {
    if (dx == 0 && dy == 0) return false;
    
    bool tileShifted = false;
    
    if (pendingResetPan) {
        offsetX = 0;
        offsetY = 0;
        velocityX = 0.0f;
        velocityY = 0.0f;
        pendingResetPan = false;
    }
    
    offsetX += dx;
    offsetY += dy;
    
    bool breakFollowGps = !pendingResetPan;
    
    // ORIGINAL: clamp exact MAP_MARGIN_X/Y (no -10)
    offsetX = clampValue<int16_t>(offsetX, -MAP_MARGIN_X, MAP_MARGIN_X);
    offsetY = clampValue<int16_t>(offsetY, -MAP_MARGIN_Y, MAP_MARGIN_Y);
    
    // Get current center tile via callback
    int centerTileX = 0, centerTileY = 0;
    if (getCenterTileCallback) {
        getCenterTileCallback(centerTileX, centerTileY);
    }
    
    // ORIGINAL: shift tile if |offset| > PAN_TILE_THRESHOLD
    int targetX = centerTileX;
    int targetY = centerTileY;
    int16_t tempX = offsetX, tempY = offsetY;
    
    if (tempX >= PAN_TILE_THRESHOLD) { 
        targetX++; 
        tempX -= MAP_TILE_SIZE; 
        tileShifted = true;
    }
    else if (tempX <= -PAN_TILE_THRESHOLD) { 
        targetX--; 
        tempX += MAP_TILE_SIZE; 
        tileShifted = true;
    }
    
    if (tempY >= PAN_TILE_THRESHOLD) { 
        targetY++; 
        tempY -= MAP_TILE_SIZE; 
        tileShifted = true;
    }
    else if (tempY <= -PAN_TILE_THRESHOLD) { 
        targetY--; 
        tempY += MAP_TILE_SIZE; 
        tileShifted = true;
    }
    
    // Callbacks
    if (scrollCallback) {
        scrollCallback(dx, dy, breakFollowGps);
    }
    
    if (tileShifted && shiftTileCallback) {
        int dX = targetX - centerTileX;
        int dY = targetY - centerTileY;
        shiftTileCallback(dX, dY);
    }
    
    if (redrawCallback) {
        redrawCallback();
    }
    
    ESP_LOGV(TAG, "Scroll(%d,%d), offsets=(%d,%d), tileShifted=%d", 
             dx, dy, offsetX, offsetY, tileShifted);
    
    return tileShifted;
}

// Getters/setters
void MapTouchController::getOffsets(int16_t& x, int16_t& y) const { x = offsetX; y = offsetY; }
void MapTouchController::getNavSubTileOffsets(int16_t& x, int16_t& y) const { x = navSubTileX; y = navSubTileY; }
void MapTouchController::setNavSubTileOffsets(int16_t x, int16_t y) { navSubTileX = x; navSubTileY = y; }
void MapTouchController::getVelocity(float& vx, float& vy) const { vx = velocityX; vy = velocityY; }
void MapTouchController::setVelocity(float vx, float vy) { velocityX = vx; velocityY = vy; }
void MapTouchController::setPendingResetPan(bool pending) { pendingResetPan = pending; }
void MapTouchController::resetPanOffsets() { offsetX = 0; offsetY = 0; velocityX = 0.0f; velocityY = 0.0f; pendingResetPan = false; }

uint32_t MapTouchController::getCurrentTimeMs() const {
    return (uint32_t)(esp_timer_get_time() / 1000);
}