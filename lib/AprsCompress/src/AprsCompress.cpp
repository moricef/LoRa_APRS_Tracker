#include "AprsCompress.h"
#include "AprsCompressModel.h"

namespace {

    constexpr uint32_t PROB_TOTAL = 1u << 16;
    constexpr uint32_t RANGE_TOP  = 1u << 24;

    // Binary search for a context key; returns the context index or -1.
    int findContext(const AprsCompressModel::OrderTable& t, uint32_t key) {
        int lo = 0, hi = (int)t.numContexts - 1;
        while (lo <= hi) {
            int mid = (lo + hi) >> 1;
            const uint8_t* kp = t.keys + 3 * mid;
            uint32_t v = ((uint32_t)kp[0] << 16) | ((uint32_t)kp[1] << 8) | kp[2];
            if (v == key) return mid;
            if (v < key) lo = mid + 1; else hi = mid - 1;
        }
        return -1;
    }

    // Probability model for the next byte, given the bytes before it in this packet.
    // Blends orders 3..0 (PPM method C escape weights) in 32.32 fixed point, then adds a
    // uniform floor of 1 so every byte value stays encodable. Fills freq[256]; returns the total.
    // If ref is not null the model is primed: for each order, every position of the reference text
    // whose preceding k bytes equal the current context adds PRIME_WEIGHT to the count of the byte
    // that followed it, on top of the stored table's count. With ref == null it is the plain model.
    // The work arrays are static (single-threaded use) to keep the stack small on the ESP32.
    uint32_t buildFreq(const uint8_t* hist, size_t pos, uint16_t* freq, const uint8_t* ref, size_t refLen) {
        static uint32_t pr[256];
        static uint32_t cnt[256];
        for (int s = 0; s < 256; s++) pr[s] = 0;
        uint64_t rem = 0xFFFFFFFFull;
        int maxK = pos < AprsCompressModel::ORDER ? (int)pos : (int)AprsCompressModel::ORDER;
        for (int k = maxK; k >= 0 && rem > 0; k--) {
            uint32_t key = 0;
            for (int j = k; j >= 1; j--) key = (key << 8) | hist[pos - j];
            const AprsCompressModel::OrderTable& t = AprsCompressModel::ORDERS[k];
            for (int s = 0; s < 256; s++) cnt[s] = 0;
            int c = findContext(t, key);
            if (c >= 0) {
                uint32_t from = t.offsets[c], to = t.offsets[c + 1];
                for (uint32_t i = from; i < to; i++) cnt[AprsCompressModel::SYMBOLS[i]] += AprsCompressModel::COUNTS[i];
            }
            if (ref != nullptr) {
                for (size_t r = (size_t)k; r < refLen; r++) {
                    bool match = true;
                    for (int j = 0; j < k; j++)
                        if (ref[r - (size_t)k + (size_t)j] != hist[pos - (size_t)k + (size_t)j]) { match = false; break; }
                    if (match) cnt[ref[r]] += AprsCompress::PRIME_WEIGHT;
                }
            }
            uint32_t u = 0, n = 0;
            for (int s = 0; s < 256; s++) if (cnt[s]) { u++; n += cnt[s]; }
            if (u == 0) continue;
            uint32_t denom = n + u;
            for (int s = 0; s < 256; s++)
                if (cnt[s]) pr[s] += (uint32_t)((rem * cnt[s]) / denom);
            rem = (rem * u) / denom;
        }
        uint32_t floorShare = (uint32_t)(rem >> 8);
        uint32_t total = 0;
        for (int s = 0; s < 256; s++) {
            uint64_t p = (uint64_t)pr[s] + floorShare;
            uint32_t f = 1 + (uint32_t)((p * (PROB_TOTAL - 256)) >> 32);
            freq[s] = (uint16_t)f;
            total += f;
        }
        return total;
    }

    struct Encoder {
        uint8_t* out;  size_t cap;  size_t n;  bool overflow;
        uint64_t low;  uint32_t range;
        void init(uint8_t* o, size_t c) { out = o; cap = c; n = 0; overflow = false; low = 0; range = 0xFFFFFFFFu; }
        void putByte(uint8_t b) { if (n < cap) out[n++] = b; else overflow = true; }
        void carry() {
            size_t i = n;
            while (i > 0) { if (++out[i - 1] != 0) return; i--; }
        }
        void shiftLow() {
            if (low >> 32) { carry(); low &= 0xFFFFFFFFull; }
            putByte((uint8_t)(low >> 24));
            low = (low << 8) & 0xFFFFFFFFull;
            range <<= 8;
        }
        void encode(uint32_t cum, uint32_t freq, uint32_t total) {
            uint32_t r = range / total;
            low += (uint64_t)r * cum;
            range = r * freq;
            while (range < RANGE_TOP) shiftLow();
        }
        // Emit the fewest bytes such that a decoder padding with zeros lands inside [low, low+range).
        void finish() {
            if (low >> 32) { carry(); low &= 0xFFFFFFFFull; }
            uint64_t hiEdge = low + range - 1;
            for (int nb = 0; nb <= 4; nb++) {
                uint64_t mask = nb == 4 ? 0 : ((1ull << (8 * (4 - nb))) - 1);
                uint64_t v = (low + mask) & ~mask;
                if (v <= hiEdge) {
                    if (v >> 32) { carry(); v &= 0xFFFFFFFFull; }
                    for (int b = 0; b < nb; b++) putByte((uint8_t)(v >> (24 - 8 * b)));
                    break;
                }
            }
            while (n > 0 && out[n - 1] == 0) n--;      // decoder pads with zeros
        }
    };

    struct Decoder {
        const uint8_t* in;  size_t len;  size_t pos;
        uint32_t code;  uint32_t range;
        uint8_t next() { return pos < len ? in[pos++] : (pos++, 0); }
        void init(const uint8_t* i, size_t l) {
            in = i; len = l; pos = 0; range = 0xFFFFFFFFu; code = 0;
            for (int b = 0; b < 4; b++) code = (code << 8) | next();
        }
        uint32_t target(uint32_t total) { range_r = range / total; uint32_t v = code / range_r; return v >= total ? total - 1 : v; }
        void consume(uint32_t cum, uint32_t freq) {
            code -= range_r * cum;
            range = range_r * freq;
            while (range < RANGE_TOP) { code = (code << 8) | next(); range <<= 8; }
        }
        uint32_t range_r;
    };
}

namespace AprsCompress {

    uint8_t modelVersion() { return AprsCompressModel::VERSION; }

    bool isCompressedHeader(const uint8_t* f, size_t len) {
        return len >= 4 && f[0] == HEADER_0 && f[1] == HEADER_1 && f[2] == (VERSION_FLAG | AprsCompressModel::VERSION);
    }

    size_t encodeFrame(const uint8_t* text, size_t textLen, uint8_t* frame, size_t frameCap) {
        if (textLen == 0 || textLen > MAX_TEXT || frameCap < 5) return 0;
        Encoder e;
        e.init(frame + 4, frameCap - 4);
        uint16_t freq[256];
        for (size_t i = 0; i < textLen; i++) {
            uint32_t total = buildFreq(text, i, freq, nullptr, 0);
            uint32_t cum = 0;
            for (int s = 0; s < text[i]; s++) cum += freq[s];
            e.encode(cum, freq[text[i]], total);
        }
        e.finish();
        if (e.overflow) return 0;
        size_t frameLen = 4 + e.n;
        if (frameLen >= 3 + textLen) return 0;
        frame[0] = HEADER_0; frame[1] = HEADER_1;
        frame[2] = VERSION_FLAG | AprsCompressModel::VERSION;
        frame[3] = (uint8_t)textLen;
        return frameLen;
    }

    size_t decodeFrame(const uint8_t* frame, size_t frameLen, uint8_t* text, size_t textCap) {
        if (!isCompressedHeader(frame, frameLen)) return 0;
        size_t textLen = frame[3];
        if (textLen == 0 || textLen > textCap) return 0;
        Decoder d;
        d.init(frame + 4, frameLen - 4);
        uint16_t freq[256];
        for (size_t i = 0; i < textLen; i++) {
            uint32_t total = buildFreq(text, i, freq, nullptr, 0);
            uint32_t v = d.target(total);
            uint32_t cum = 0; int s = 0;
            while (s < 255 && cum + freq[s] <= v) { cum += freq[s]; s++; }
            d.consume(cum, freq[s]);
            text[i] = (uint8_t)s;
        }
        return textLen;
    }

    // ---- primed frames ----

    uint8_t crc8(const uint8_t* p, size_t n) {
        uint8_t c = 0;
        for (size_t i = 0; i < n; i++) {
            c ^= p[i];
            for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
        }
        return c;
    }

    uint8_t stationId(const uint8_t* text, size_t len) {
        size_t n = 0;
        while (n < len && text[n] != '>') n++;
        return crc8(text, n);
    }

    bool isPrimedHeader(const uint8_t* f, size_t len) {
        return len >= 6 && f[0] == HEADER_0 && f[1] == HEADER_1 &&
               f[2] == (uint8_t)(VERSION_FLAG | PRIMED_FLAG | AprsCompressModel::VERSION);
    }

    bool primedHeader(const uint8_t* f, size_t len, uint8_t& id, uint8_t& refCheck) {
        if (!isPrimedHeader(f, len)) return false;
        id = f[3]; refCheck = f[4];
        return true;
    }

    size_t encodePrimed(const uint8_t* ref, size_t refLen, const uint8_t* text, size_t textLen,
                        uint8_t* frame, size_t frameCap) {
        if (ref == nullptr || refLen == 0 || refLen > MAX_TEXT) return 0;
        if (textLen == 0 || textLen > MAX_TEXT || frameCap < 7) return 0;
        Encoder e;
        e.init(frame + 6, frameCap - 6);
        uint16_t freq[256];
        for (size_t i = 0; i < textLen; i++) {
            uint32_t total = buildFreq(text, i, freq, ref, refLen);
            uint32_t cum = 0;
            for (int s = 0; s < text[i]; s++) cum += freq[s];
            e.encode(cum, freq[text[i]], total);
        }
        e.finish();
        if (e.overflow) return 0;
        size_t frameLen = 6 + e.n;
        if (frameLen >= 3 + textLen) return 0;
        frame[0] = HEADER_0; frame[1] = HEADER_1;
        frame[2] = VERSION_FLAG | PRIMED_FLAG | AprsCompressModel::VERSION;
        frame[3] = stationId(text, textLen);
        frame[4] = crc8(ref, refLen);
        frame[5] = (uint8_t)textLen;
        return frameLen;
    }

    size_t decodePrimed(const uint8_t* ref, size_t refLen, const uint8_t* frame, size_t frameLen,
                        uint8_t* text, size_t textCap) {
        if (ref == nullptr || refLen == 0 || refLen > MAX_TEXT) return 0;
        if (!isPrimedHeader(frame, frameLen)) return 0;
        if (frame[4] != crc8(ref, refLen)) return 0;                // wrong reference
        size_t textLen = frame[5];
        if (textLen == 0 || textLen > textCap) return 0;
        Decoder d;
        d.init(frame + 6, frameLen - 6);
        uint16_t freq[256];
        for (size_t i = 0; i < textLen; i++) {
            uint32_t total = buildFreq(text, i, freq, ref, refLen);
            uint32_t v = d.target(total);
            uint32_t cum = 0; int s = 0;
            while (s < 255 && cum + freq[s] <= v) { cum += freq[s]; s++; }
            d.consume(cum, freq[s]);
            text[i] = (uint8_t)s;
        }
        if (stationId(text, textLen) != frame[3]) return 0;         // decoded to the wrong station
        return textLen;
    }
}
