#pragma once
#include <Arduino.h>  // temporaire, sera remplacé par esp_timer.h, driver/gpio.h, etc.

// ============================================================================
// Time functions
// ============================================================================
inline uint32_t compat_millis() { return millis(); }
inline uint32_t compat_micros() { return micros(); }
inline void compat_delay(uint32_t ms) { delay(ms); }
inline void compat_delayMicroseconds(uint32_t us) { delayMicroseconds(us); }

// ============================================================================
// GPIO functions
// ============================================================================
inline void compat_pinMode(uint8_t pin, uint8_t mode) { pinMode(pin, mode); }
inline void compat_digitalWrite(uint8_t pin, uint8_t val) { digitalWrite(pin, val); }
inline int compat_digitalRead(uint8_t pin) { return digitalRead(pin); }
inline int compat_analogRead(uint8_t pin) { return analogRead(pin); }

// ============================================================================
// Interrupts (optional, not used everywhere)
// ============================================================================
inline void compat_attachInterrupt(uint8_t pin, void (*isr)(), int mode) { attachInterrupt(pin, isr, mode); }
inline void compat_detachInterrupt(uint8_t pin) { detachInterrupt(pin); }

// ============================================================================
// Yield (allows background tasks)
// ============================================================================
inline void compat_yield() { yield(); }

// ============================================================================
// Constants (re-exported for clarity)
// ============================================================================
#define COMPAT_INPUT          INPUT
#define COMPAT_OUTPUT         OUTPUT
#define COMPAT_INPUT_PULLUP   INPUT_PULLUP
#define COMPAT_INPUT_PULLDOWN INPUT_PULLDOWN
#define COMPAT_HIGH           HIGH
#define COMPAT_LOW            LOW
#define COMPAT_RISING         RISING
#define COMPAT_FALLING        FALLING
#define COMPAT_CHANGE         CHANGE

// ============================================================================
// SPI / I2C / Serial wrappers (if needed later)
// ============================================================================
// Not yet implemented; will be added when migrating those buses.
// For now, keep direct Arduino calls.