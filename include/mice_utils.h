/* Copyright (C) 2026 Fabrice - F4MLV
 *
 * This file is part of LoRa APRS Tracker.
 *
 * LoRa APRS Tracker is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef MICE_UTILS_H_
#define MICE_UTILS_H_

namespace MicEUtils {

// APRSPacketLib 1.0.4 applies the Mic-E longitude offset but omits the two
// normalization steps from APRS101 chapter 10. Consequently, decoded values
// in 180..189 represent 100..109 degrees and values in 190..199 represent
// 0..9 degrees. Preserve the hemisphere while restoring the real longitude.
inline float normalizeDecodedLongitude(float longitude) {
    const bool west = longitude < 0.0f;
    float degrees = west ? -longitude : longitude;

    if (degrees >= 180.0f && degrees < 190.0f) {
        degrees -= 80.0f;
    } else if (degrees >= 190.0f && degrees < 200.0f) {
        degrees -= 190.0f;
    }

    return west ? -degrees : degrees;
}

} // namespace MicEUtils

#endif // MICE_UTILS_H_
