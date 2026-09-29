// Copied from moricef/LoRa_APRS_iGate, branch rxt, commit b433e56 (src/native_aprs.cpp).
// Keep identical: the host tests and test vectors live in that repository.
#include "native_aprs.h"
#include "native_aprs_huffman_table.h"
#include "rxt_protocol.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace {

    const char B40[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    const char DTIS[] = "!=/@";
    const uint8_t PREFIX[3] = {'<', 0xFF, 0x02};
    constexpr int HUFFMAN_MAX_BITS = 15;

    bool isDigit(char c) { return c >= '0' && c <= '9'; }
    bool isUpperAlnum(char c) { return (c >= 'A' && c <= 'Z') || isDigit(c); }

    bool allDigits(const std::string& s, size_t pos, size_t n) {
        if (pos + n > s.size()) return false;
        for (size_t i = pos; i < pos + n; i++) {
            if (!isDigit(s[i])) return false;
        }
        return true;
    }

    int toInt(const std::string& s, size_t pos, size_t n) {
        int v = 0;
        for (size_t i = pos; i < pos + n; i++) v = v * 10 + (s[i] - '0');
        return v;
    }

    void putBE(std::vector<uint8_t>& out, uint64_t v, int bytes) {
        for (int i = bytes - 1; i >= 0; i--) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
    }

    bool getBE(const uint8_t* b, size_t size, size_t& i, int bytes, uint64_t& v) {
        if (i + bytes > size) return false;
        v = 0;
        for (int k = 0; k < bytes; k++) v = v << 8 | b[i + k];
        i += bytes;
        return true;
    }

    std::string pad(unsigned v, int width) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%0*u", width, v);
        return buf;
    }

    // ---- address elements ---------------------------------------------------

    // WIDEn[-N][*], n and N in 1..7.
    bool matchWide(const std::string& e, int& n, int& N, bool& star) {
        if (e.size() < 5 || e.compare(0, 4, "WIDE") != 0 || e[4] < '1' || e[4] > '7') return false;
        n = e[4] - '0';
        N = 0;
        size_t i = 5;
        if (i < e.size() && e[i] == '-') {
            if (i + 1 >= e.size() || e[i + 1] < '1' || e[i + 1] > '7') return false;
            N = e[i + 1] - '0';
            i += 2;
        }
        star = i < e.size() && e[i] == '*';
        if (star) i++;
        return i == e.size();
    }

    // 1-6 of A-Z0-9, optional -SSID in 1..255 without leading zero, optional '*'.
    bool matchCall(const std::string& e, std::string& call, int& ssid, bool& star) {
        size_t i = 0;
        while (i < e.size() && i < 6 && isUpperAlnum(e[i])) i++;
        if (i == 0) return false;
        call = e.substr(0, i);
        ssid = 0;
        if (i < e.size() && e[i] == '-') {
            size_t j = i + 1;
            while (j < e.size() && j < i + 4 && isDigit(e[j])) j++;
            size_t len = j - (i + 1);
            if (len == 0 || e[i + 1] == '0') return false;
            ssid = toInt(e, i + 1, len);
            if (ssid > 255) return false;
            i = j;
        }
        star = i < e.size() && e[i] == '*';
        if (star) i++;
        return i == e.size();
    }

    bool encElement(const std::string& e, std::vector<uint8_t>& out) {
        int n, N, ssid;
        bool star;
        std::string call;
        if (matchWide(e, n, N, star)) {                 // 0b0 s nnn NNN
            out.push_back((star ? 0x40 : 0) | n << 3 | N);
            return true;
        }
        if (matchCall(e, call, ssid, star)) {           // 0b10 s xxxxx + base 40 [+ SSID]
            uint32_t v = 0;
            for (size_t k = 0; k < 6; k++) {
                char c = k < call.size() ? call[k] : ' ';
                v = v * 40 + static_cast<uint32_t>(std::strchr(B40, c) - B40);
            }
            uint8_t star_bit = star ? 0x20 : 0;
            out.push_back(0x80 | star_bit | (ssid <= 30 ? ssid : 31));
            putBE(out, v, 4);
            if (ssid > 30) out.push_back(static_cast<uint8_t>(ssid));
            return true;
        }
        if (e.empty() || e.size() > 31) return false;   // 0b11 lllll + raw text
        out.push_back(0xC0 | static_cast<uint8_t>(e.size()));
        out.insert(out.end(), e.begin(), e.end());
        return true;
    }

    bool decElement(const uint8_t* b, size_t size, size_t& i, std::string& e) {
        if (i >= size) return false;
        uint8_t t = b[i];
        if (t < 0x80) {
            e = "WIDE" + std::to_string(t >> 3 & 7);
            if (t & 7) e += "-" + std::to_string(t & 7);
            if (t & 0x40) e += "*";
            i += 1;
            return true;
        }
        if (t < 0xC0) {
            size_t j = i + 1;
            uint64_t v;
            if (!getBE(b, size, j, 4, v)) return false;
            char s[7];
            for (int k = 5; k >= 0; k--) {
                s[k] = B40[v % 40];
                v /= 40;
            }
            s[6] = 0;
            e = s;
            e.erase(e.find_last_not_of(' ') + 1);
            unsigned ssid = t & 0x1F;
            if (ssid == 31) {
                if (j >= size) return false;
                ssid = b[j++];
            }
            if (ssid) e += "-" + std::to_string(ssid);
            if (t & 0x20) e += "*";
            i = j;
            return true;
        }
        size_t n = t & 0x1F;
        if (i + 1 + n > size) return false;
        e.assign(reinterpret_cast<const char*>(b + i + 1), n);
        i += 1 + n;
        return true;
    }

    // ---- position -----------------------------------------------------------

    uint32_t base91(const std::string& s, size_t pos) {
        uint32_t v = 0;
        for (size_t k = 0; k < 4; k++) v = v * 91 + static_cast<uint8_t>(s[pos + k]) - 33;
        return v;
    }

    std::string toBase91(uint32_t v) {
        std::string s(4, ' ');
        for (int k = 3; k >= 0; k--) {
            s[k] = static_cast<char>(v % 91 + 33);
            v /= 91;
        }
        return s;
    }

    bool isB91(char c) { return c >= '!' && c <= '{'; }
    bool isSymbolTable(char c) { return c == '/' || c == '\\' || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'j'); }

    // First /A=dddddd or /A=-ddddd, as Python re.search would find it.
    bool findAltitude(const std::string& s, size_t& start, size_t& len, long& value) {
        for (size_t p = 0; p + 3 <= s.size(); p++) {
            if (s.compare(p, 3, "/A=") != 0) continue;
            if (allDigits(s, p + 3, 6)) {
                start = p; len = 9; value = toInt(s, p + 3, 6);
                return true;
            }
            if (p + 3 < s.size() && s[p + 3] == '-' && allDigits(s, p + 4, 5)) {
                start = p; len = 9; value = -toInt(s, p + 4, 5);
                return true;
            }
        }
        return false;
    }

    // Fixed position fields and the remaining comment text, or false.
    bool encPosition(const std::string& info, std::vector<uint8_t>& fields, std::string& rest) {
        const char* d = std::strchr(DTIS, info[0]);
        if (info.empty() || d == nullptr) return false;
        uint8_t flags = static_cast<uint8_t>(d - DTIS);
        std::vector<uint8_t> out;
        std::string body = info.substr(1);
        if (info[0] == '/' || info[0] == '@') {
            if (!allDigits(body, 0, 6) || body.size() < 7) return false;
            const char* f = std::strchr("zh/", body[6]);
            if (f == nullptr || body[6] == 0) return false;
            flags |= 0x04;
            putBE(out, static_cast<uint32_t>(toInt(body, 0, 6)) << 2 | static_cast<uint32_t>(f - "zh/"), 3);
            body = body.substr(7);
        }
        if (body.size() >= 13 && isSymbolTable(body[0]) && !isDigit(body[0]) &&
            std::all_of(body.begin() + 1, body.begin() + 9, isB91)) {
            flags |= 0x08;
            uint64_t v = static_cast<uint64_t>(base91(body, 1)) << 27 | base91(body, 5);
            putBE(out, v, 7);
            out.push_back(static_cast<uint8_t>(body[0]));
            out.push_back(static_cast<uint8_t>(body[9]));
            if (body[10] == ' ' && body[11] == ' ') {
                flags |= 0x40;
                out.push_back(static_cast<uint8_t>(body[12]));
            } else {
                out.push_back(static_cast<uint8_t>(body[10]));
                out.push_back(static_cast<uint8_t>(body[11]));
                out.push_back(static_cast<uint8_t>(body[12]));
            }
            rest = body.substr(13);
        } else {
            // DDMM.mmN T DDDMM.mmE C
            if (body.size() < 19 || !allDigits(body, 0, 4) || body[4] != '.' || !allDigits(body, 5, 2) ||
                (body[7] != 'N' && body[7] != 'S') || body[8] == '\n' ||
                !allDigits(body, 9, 5) || body[14] != '.' || !allDigits(body, 15, 2) ||
                (body[17] != 'E' && body[17] != 'W') || body[18] == '\n') return false;
            uint64_t lat = toInt(body, 0, 2) * 6000 + toInt(body, 2, 2) * 100 + toInt(body, 5, 2);
            uint64_t lon = toInt(body, 9, 3) * 6000 + toInt(body, 12, 2) * 100 + toInt(body, 15, 2);
            if (lat >= (1u << 20) || lon >= (1u << 21)) return false;
            uint64_t v = (lat << 1 | (body[7] == 'S')) << 22 | (lon << 1 | (body[17] == 'W'));
            putBE(out, v, 6);
            out.push_back(static_cast<uint8_t>(body[8]));
            out.push_back(static_cast<uint8_t>(body[18]));
            rest = body.substr(19);
            if (allDigits(rest, 0, 3) && rest.size() >= 7 && rest[3] == '/' && allDigits(rest, 4, 3)) {
                flags |= 0x10;
                putBE(out, static_cast<uint32_t>(toInt(rest, 0, 3)) << 10 | toInt(rest, 4, 3), 3);
                rest = rest.substr(7);
            }
        }
        size_t start, len;
        long alt;
        if (findAltitude(rest, start, len, alt) && start < 256) {
            flags |= 0x20;
            out.push_back(static_cast<uint8_t>(start));
            putBE(out, static_cast<uint32_t>(alt) & 0xFFFFFF, 3);
            rest.erase(start, len);
        }
        fields.clear();
        fields.push_back(flags);
        fields.insert(fields.end(), out.begin(), out.end());
        return true;
    }

    struct Altitude {
        bool present = false;
        size_t offset = 0;
        std::string text;
    };

    bool decPosition(const uint8_t* b, size_t size, size_t& i, std::string& s, Altitude& alt) {
        if (i >= size) return false;
        uint8_t flags = b[i++];
        s.assign(1, DTIS[flags & 3]);
        uint64_t v;
        if (flags & 0x04) {
            if (!getBE(b, size, i, 3, v)) return false;
            s += pad(static_cast<unsigned>(v >> 2), 6);
            s += "zh/"[v & 3];
        }
        if (flags & 0x08) {
            if (!getBE(b, size, i, 7, v) || i + 2 > size) return false;
            char table = b[i], code = b[i + 1];
            i += 2;
            std::string cst;
            if (flags & 0x40) {
                if (i >= size) return false;
                cst = std::string("  ") + static_cast<char>(b[i++]);
            } else {
                if (i + 3 > size) return false;
                cst.assign(reinterpret_cast<const char*>(b + i), 3);
                i += 3;
            }
            s += table + toBase91(static_cast<uint32_t>(v >> 27)) +
                 toBase91(static_cast<uint32_t>(v & ((1u << 27) - 1))) + code + cst;
        } else {
            if (!getBE(b, size, i, 6, v) || i + 2 > size) return false;
            unsigned la = static_cast<unsigned>(v >> 22), lo = static_cast<unsigned>(v & ((1u << 22) - 1));
            unsigned lat = la >> 1, lon = lo >> 1;
            char table = b[i], code = b[i + 1];
            i += 2;
            s += pad(lat / 6000, 2) + pad(lat / 100 % 60, 2) + "." + pad(lat % 100, 2) + (la & 1 ? 'S' : 'N') + table;
            s += pad(lon / 6000, 3) + pad(lon / 100 % 60, 2) + "." + pad(lon % 100, 2) + (lo & 1 ? 'W' : 'E') + code;
            if (flags & 0x10) {
                if (!getBE(b, size, i, 3, v)) return false;
                s += pad(static_cast<unsigned>(v >> 10), 3) + "/" + pad(static_cast<unsigned>(v & 1023), 3);
            }
        }
        if (flags & 0x20) {
            if (i + 4 > size) return false;
            alt.present = true;
            alt.offset = b[i++];
            getBE(b, size, i, 3, v);
            long a = static_cast<long>(v);
            if (a & 0x800000) a -= 1L << 24;
            alt.text = a >= 0 ? "/A=" + pad(static_cast<unsigned>(a), 6) : "/A=-" + pad(static_cast<unsigned>(-a), 5);
        }
        return true;
    }

    // ---- Huffman --------------------------------------------------------------

    struct Canonical {
        uint16_t code[256];
        uint8_t  symbols[256];                   // sorted by (length, symbol)
        uint16_t firstCode[HUFFMAN_MAX_BITS + 2];
        uint16_t firstIndex[HUFFMAN_MAX_BITS + 2];
        uint16_t count[HUFFMAN_MAX_BITS + 2];
    };

    const Canonical& canonical() {
        static Canonical c;
        static bool built = false;
        if (built) return c;
        std::fill(std::begin(c.count), std::end(c.count), 0);
        for (int s = 0; s < 256; s++) c.count[NATIVE_APRS::HUFFMAN_LENGTHS[s]]++;
        uint16_t code = 0, index = 0;
        for (int len = 1; len <= HUFFMAN_MAX_BITS; len++) {
            code = (code + (len > 1 ? c.count[len - 1] : 0)) << 1;
            if (len == 1) code = 0;
            c.firstCode[len] = code;
            c.firstIndex[len] = index;
            index += c.count[len];
        }
        uint16_t next[HUFFMAN_MAX_BITS + 2];
        std::copy(std::begin(c.firstCode), std::end(c.firstCode), next);
        uint16_t pos[HUFFMAN_MAX_BITS + 2];
        std::copy(std::begin(c.firstIndex), std::end(c.firstIndex), pos);
        for (int s = 0; s < 256; s++) {
            int len = NATIVE_APRS::HUFFMAN_LENGTHS[s];
            c.code[s] = next[len]++;
            c.symbols[pos[len]++] = static_cast<uint8_t>(s);
        }
        built = true;
        return c;
    }

    bool encText(const std::string& text, uint8_t& flag, std::vector<uint8_t>& out) {
        flag = 0;
        if (!text.empty() && text.size() <= 255) {
            std::vector<uint8_t> coded = NATIVE_APRS::huffmanEncode(text);
            if (coded.size() + 1 < text.size()) {
                flag = NATIVE_APRS::TEXT_HUFFMAN;
                out.push_back(static_cast<uint8_t>(text.size()));
                out.insert(out.end(), coded.begin(), coded.end());
                return true;
            }
        }
        out.insert(out.end(), text.begin(), text.end());
        return true;
    }

    bool splitPacket(const std::string& tnc2, std::string& src, std::vector<std::string>& addrs,
                     std::string& info) {
        size_t colon = tnc2.find(':');
        if (colon == std::string::npos) return false;
        std::string head = tnc2.substr(0, colon);
        info = tnc2.substr(colon + 1);
        size_t gt = head.find('>');
        if (gt == std::string::npos) return false;
        src = head.substr(0, gt);
        addrs.clear();
        size_t start = gt + 1;
        while (true) {
            size_t comma = head.find(',', start);
            addrs.push_back(head.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
            if (comma == std::string::npos) break;
            start = comma + 1;
        }
        return true;
    }

    bool build(const std::string& src, const std::vector<std::string>& addrs,
               const std::vector<NATIVE_APRS::RxtTuple>& rxt, uint8_t kind,
               const std::vector<uint8_t>& fields, const std::string& text, std::vector<uint8_t>& out) {
        size_t npath = addrs.size() - 1;
        std::vector<uint8_t> tail;
        uint8_t tflag;
        encText(text, tflag, tail);
        out.assign(PREFIX, PREFIX + 3);
        out.push_back(static_cast<uint8_t>(npath << 4 | (rxt.empty() ? 0 : NATIVE_APRS::RXT_FLAG) | tflag | kind));
        if (!encElement(src, out)) return false;
        for (const std::string& e : addrs) {
            if (!encElement(e, out)) return false;
        }
        if (!rxt.empty()) {
            std::vector<NATIVE_APRS::RxtTuple> sorted(rxt);
            std::sort(sorted.begin(), sorted.end(),
                      [](const NATIVE_APRS::RxtTuple& a, const NATIVE_APRS::RxtTuple& b) { return a.index < b.index; });
            uint8_t mask = 0;
            std::vector<uint8_t> tuples;
            for (const NATIVE_APRS::RxtTuple& t : sorted) {
                if (t.index >= std::min(npath, NATIVE_APRS::MAX_RXT) || (mask >> t.index & 1)) return false;
                mask |= 1 << t.index;
                for (char c : t.tuple) {
                    if (c < 33 || c > 122) return false;
                    tuples.push_back(static_cast<uint8_t>(c - 33));
                }
            }
            out.push_back(mask);
            out.insert(out.end(), tuples.begin(), tuples.end());
        }
        out.insert(out.end(), fields.begin(), fields.end());
        out.insert(out.end(), tail.begin(), tail.end());
        return true;
    }

    bool sameRxt(const std::vector<NATIVE_APRS::RxtTuple>& a, const std::vector<NATIVE_APRS::RxtTuple>& b) {
        std::vector<NATIVE_APRS::RxtTuple> x(a);
        std::sort(x.begin(), x.end(),
                  [](const NATIVE_APRS::RxtTuple& p, const NATIVE_APRS::RxtTuple& q) { return p.index < q.index; });
        if (x.size() != b.size()) return false;
        for (size_t k = 0; k < x.size(); k++) {
            if (x[k].index != b[k].index || !std::equal(x[k].tuple, x[k].tuple + 4, b[k].tuple)) return false;
        }
        return true;
    }

}

namespace NATIVE_APRS {

    std::vector<uint8_t> huffmanEncode(const std::string& text) {
        const Canonical& c = canonical();
        std::vector<uint8_t> out;
        uint32_t acc = 0;
        int nbits = 0;
        for (unsigned char ch : text) {
            int len = HUFFMAN_LENGTHS[ch];
            acc = acc << len | c.code[ch];
            nbits += len;
            while (nbits >= 8) {
                out.push_back(static_cast<uint8_t>(acc >> (nbits - 8)));
                nbits -= 8;
                acc &= (1u << nbits) - 1;
            }
        }
        if (nbits > 0) out.push_back(static_cast<uint8_t>(acc << (8 - nbits)));
        return out;
    }

    bool huffmanDecode(const uint8_t* data, size_t size, size_t count, std::string& text) {
        const Canonical& c = canonical();
        text.clear();
        if (count == 0) return true;
        uint32_t code = 0;
        int len = 0;
        for (size_t i = 0; i < size; i++) {
            for (int k = 7; k >= 0; k--) {
                code = code << 1 | (data[i] >> k & 1);
                len++;
                if (code - c.firstCode[len] < c.count[len] && code >= c.firstCode[len]) {
                    text += static_cast<char>(c.symbols[c.firstIndex[len] + code - c.firstCode[len]]);
                    if (text.size() == count) return true;
                    code = 0;
                    len = 0;
                } else if (len >= HUFFMAN_MAX_BITS) {
                    return false;
                }
            }
        }
        return false;
    }

    bool isNativeFrame(const uint8_t* data, size_t size) {
        return size > PREFIX_BYTES && std::equal(PREFIX, PREFIX + 3, data);
    }

    std::string toText(const std::string& tnc2, const std::vector<RxtTuple>& rxt,
                       const std::string& regionalAliases) {
        if (rxt.empty() || rxt.size() > RXT_Protocol::MAX_TUPLES) return tnc2;
        std::string src, info;
        std::vector<std::string> addrs;
        if (!splitPacket(tnc2, src, addrs, info)) return tnc2;
        std::string content;
        for (const RxtTuple& t : rxt) {
            if (static_cast<size_t>(t.index) + 1 >= addrs.size()) return tnc2;
            content += RXT_Protocol::fingerprint(addrs[t.index + 1]);
            content.append(t.tuple, 4);
        }
        if (!RXT_Protocol::trailerMatchesPath(tnc2, content, regionalAliases)) return tnc2;
        return tnc2 + "{" + content + "}";
    }

    bool fromText(const std::string& text, const std::string& regionalAliases,
                  std::string& tnc2, std::vector<RxtTuple>& rxt) {
        rxt.clear();
        std::string content;
        tnc2 = RXT_Protocol::stripTrailer(text, &content, regionalAliases);
        if (content.empty()) return true;
        std::string src, info;
        std::vector<std::string> addrs;
        if (!splitPacket(tnc2, src, addrs, info)) return false;
        size_t next = 1;                                    // addrs[0] is the destination
        for (size_t i = 0; i + RXT_Protocol::TUPLE_BYTES <= content.size(); i += RXT_Protocol::TUPLE_BYTES) {
            while (next < addrs.size() && RXT_Protocol::fingerprint(addrs[next]) != content[i]) next++;
            if (next >= addrs.size() || next - 1 >= MAX_RXT) return false;
            RxtTuple t;
            t.index = static_cast<uint8_t>(next - 1);
            std::copy(content.begin() + i + 1, content.begin() + i + RXT_Protocol::TUPLE_BYTES, t.tuple);
            rxt.push_back(t);
            next++;
        }
        return true;
    }

    std::string freeText(const std::string& tnc2) {
        size_t colon = tnc2.find(':');
        if (colon == std::string::npos) return std::string();
        std::string info = tnc2.substr(colon + 1);
        std::vector<uint8_t> fields;
        std::string rest;
        if (!info.empty() && std::strchr(DTIS, info[0]) && info[0] && encPosition(info, fields, rest)) return rest;
        return info;
    }

    bool encode(const std::string& tnc2, const std::vector<RxtTuple>& rxt, std::vector<uint8_t>& out) {
        std::string src, info;
        std::vector<std::string> addrs;
        if (!splitPacket(tnc2, src, addrs, info) || addrs.size() - 1 > MAX_PATH) return false;
        std::vector<uint8_t> fields;
        std::string text = info;
        uint8_t kind = KIND_RAW;
        if (!info.empty() && info[0] && std::strchr(DTIS, info[0])) {
            std::string rest;
            if (encPosition(info, fields, rest)) {
                kind = KIND_POSITION;
                text = rest;
            } else {
                fields.clear();
            }
        }
        if (!build(src, addrs, rxt, kind, fields, text, out)) return false;
        std::string back;
        std::vector<RxtTuple> backRxt;
        if (!decode(out.data(), out.size(), back, backRxt) || back != tnc2 || !sameRxt(rxt, backRxt)) {
            // Never emit a frame that does not round-trip.
            if (!build(src, addrs, rxt, KIND_RAW, std::vector<uint8_t>(), info, out)) return false;
        }
        return true;
    }

    bool decode(const uint8_t* b, size_t size, std::string& tnc2, std::vector<RxtTuple>& rxt) {
        if (!isNativeFrame(b, size)) return false;
        uint8_t header = b[3];
        size_t npath = header >> 4;
        uint8_t kind = header & KIND_MASK;
        if (kind > KIND_POSITION) return false;
        size_t i = 4;
        std::string e;
        if (!decElement(b, size, i, e)) return false;
        tnc2 = e + ">";
        for (size_t k = 0; k <= npath; k++) {
            if (!decElement(b, size, i, e)) return false;
            if (k) tnc2 += ",";
            tnc2 += e;
        }
        tnc2 += ":";
        rxt.clear();
        if (header & RXT_FLAG) {
            if (i >= size) return false;
            uint8_t mask = b[i++];
            for (uint8_t k = 0; k < MAX_RXT; k++) {
                if (!(mask >> k & 1)) continue;
                if (i + 4 > size) return false;
                RxtTuple t;
                t.index = k;
                for (int j = 0; j < 4; j++) t.tuple[j] = static_cast<char>(b[i + j] + 33);
                rxt.push_back(t);
                i += 4;
            }
        }
        std::string prefix;
        Altitude alt;
        if (kind == KIND_POSITION && !decPosition(b, size, i, prefix, alt)) return false;
        std::string text;
        if (header & TEXT_HUFFMAN) {
            if (i >= size) return false;
            size_t count = b[i];
            if (!huffmanDecode(b + i + 1, size - i - 1, count, text)) return false;
        } else {
            text.assign(reinterpret_cast<const char*>(b + i), size - i);
        }
        if (alt.present) {
            if (alt.offset > text.size()) return false;
            text.insert(alt.offset, alt.text);
        }
        tnc2 += prefix + text;
        return true;
    }

}
