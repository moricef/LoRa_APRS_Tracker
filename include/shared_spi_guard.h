#ifndef SHARED_SPI_GUARD_H
#define SHARED_SPI_GUARD_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Display, SD and LoRa share one physical SPI bus on the T-Deck Plus.
// The mutex is created by the LVGL display initialization before storage setup.
extern SemaphoreHandle_t spiMutex;

class SharedSpiGuard {
public:
    explicit SharedSpiGuard(TickType_t timeout = portMAX_DELAY)
        : mutex_(spiMutex), locked_(mutex_ == nullptr) {
        if (mutex_ != nullptr) {
            locked_ = xSemaphoreTakeRecursive(mutex_, timeout) == pdTRUE;
        }
    }

    ~SharedSpiGuard() {
        if (mutex_ != nullptr && locked_) {
            xSemaphoreGiveRecursive(mutex_);
        }
    }

    SharedSpiGuard(const SharedSpiGuard&) = delete;
    SharedSpiGuard& operator=(const SharedSpiGuard&) = delete;

    bool acquired() const { return locked_; }

private:
    SemaphoreHandle_t mutex_;
    bool locked_;
};

#endif
