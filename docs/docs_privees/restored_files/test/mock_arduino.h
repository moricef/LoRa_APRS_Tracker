// test/mock_arduino.h
#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <cstdint>

// Mock des fonctions Arduino utilisées dans map_gps_filter.cpp
namespace MockArduino {
    static unsigned long current_millis = 0;

    // Fonction mockée pour millis()
    unsigned long millis() {
        return current_millis++;
    }

    // Reset pour les tests (appelé dans SetUp() si besoin)
    void reset_millis() {
        current_millis = 0;
    }

    // No-op pour d'autres fonctions potentielles (ex. delay)
    void delay(unsigned long) {}
}

#endif // MOCK_ARDUINO_H