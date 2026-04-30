// shared_symbols.cpp — Symbols shared between LVGL and non-LVGL builds
// Extracted from lvgl_ui.cpp so T-Beam (non-LVGL) can link without LVGL deps.

#include <Arduino.h>
#include "custom_characters.h"

SemaphoreHandle_t spiMutex = NULL;

const char *symbolArray[] = {"[", ">", "j", "b", "<", "s", "u", "R",
                             "v", "(", ";", "-", "k", "C", "a", "Y",
                             "O", "'", "=", "y", "U", "p", "_", ")"};
const int symbolArraySize = sizeof(symbolArray) / sizeof(symbolArray[0]);
const uint8_t *symbolsAPRS[] = {runnerSymbol,     carSymbol,
                                jeepSymbol,       bikeSymbol,
                                motorcycleSymbol, shipSymbol,
                                truck18Symbol,    recreationalVehicleSymbol,
                                vanSymbol,        carsateliteSymbol,
                                tentSymbol,       houseSymbol,
                                truckSymbol,      canoeSymbol,
                                ambulanceSymbol,  yatchSymbol,
                                baloonSymbol,     aircraftSymbol,
                                trainSymbol,      yagiSymbol,
                                busSymbol,        dogSymbol,
                                wxSymbol,         wheelchairSymbol};
