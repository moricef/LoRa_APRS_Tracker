/*
 * Stubs for ESP32-S3 SIMD functions missing when esp-dsp is not available
 * to the assembler in the Arduino Core 3.x build environment.
 * Provides C fallback implementations matching the .S file signatures.
 */
#ifdef ARDUINO_ESP32S3_DEV

#include <cstdint>

extern "C" {

void s3_rgb565(uint8_t *pSrc, uint8_t *pDest, int iCount, bool bBigEndian) {
    for (int x = 0; x < iCount; x++) {
        uint16_t usPixel = (pSrc[2] >> 3);
        usPixel |= ((pSrc[1] >> 2) << 5);
        usPixel |= ((pSrc[0] >> 3) << 11);
        if (bBigEndian) usPixel = __builtin_bswap16(usPixel);
        *pDest++ = usPixel;
        pSrc += 4;
    }
}

}  // extern "C"

#endif  // ARDUINO_ESP32S3_DEV
