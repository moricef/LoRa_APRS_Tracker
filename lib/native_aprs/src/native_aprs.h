// Copied from moricef/LoRa_APRS_iGate, branch rxt, commit b433e56 (include/native_aprs.h).
// Keep identical: the host tests and test vectors live in that repository.
#ifndef NATIVE_APRS_H_
#define NATIVE_APRS_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Native LoRa APRS frame, draft 0 (docs/LORA_APRS_NATIVE_FORMAT.md).
// encode() and decode() round-trip a TNC2 packet byte for byte; anything
// the draft cannot represent exactly is carried as raw text. Strings hold
// raw bytes (one char per byte).
namespace NATIVE_APRS {

    // RXT measurement attached to path element `index`: RSSI SNR FO TTH,
    // each one character in '!'..'z' as in the text trailer.
    struct RxtTuple {
        uint8_t index;
        char    tuple[4];
    };

    // Header byte: path count (bits 7-4), RXT block (bit 3),
    // Huffman text (bit 2), information kind (bits 1-0).
    constexpr uint8_t RXT_FLAG     = 0x08;
    constexpr uint8_t TEXT_HUFFMAN = 0x04;
    constexpr uint8_t KIND_MASK    = 0x03;
    constexpr uint8_t KIND_RAW      = 0;
    constexpr uint8_t KIND_POSITION = 1;

    constexpr size_t PREFIX_BYTES = 3;          // '<' 0xFF 0x02
    constexpr size_t MAX_PATH     = 15;
    constexpr size_t MAX_RXT      = 8;          // path elements a tuple can refer to

    // False when the packet cannot be encoded (no ':' or '>', path longer
    // than MAX_PATH, raw address element longer than 31 bytes, invalid tuple).
    bool encode(const std::string& tnc2, const std::vector<RxtTuple>& rxt,
                std::vector<uint8_t>& out);

    // False on a malformed frame. rxt is returned in path order.
    bool decode(const uint8_t* data, size_t size, std::string& tnc2,
                std::vector<RxtTuple>& rxt);

    bool isNativeFrame(const uint8_t* data, size_t size);

    // Text packet for the existing RXT v2 pipeline: tnc2 plus a v2 trailer
    // built from rxt (fingerprint of the path element, then the 4 characters).
    // The trailer is left out when there are more than RXT_Protocol::MAX_TUPLES
    // tuples or when it would not pass RXT_Protocol::trailerMatchesPath().
    std::string toText(const std::string& tnc2, const std::vector<RxtTuple>& rxt,
                       const std::string& regionalAliases = "");

    // Inverse of toText(): split a text packet into tnc2 and its RXT tuples,
    // each placed on the path element whose fingerprint it carries (in path
    // order). A trailer that fails RXT_Protocol::trailerMatchesPath() is not
    // RXT and stays in tnc2. Returns false if a tuple matches no element.
    bool fromText(const std::string& text, const std::string& regionalAliases,
                  std::string& tnc2, std::vector<RxtTuple>& rxt);

    // Free text the encoder carries (position comment without /A=, or the
    // whole information field). Used to build the Huffman table.
    std::string freeText(const std::string& tnc2);

    // Canonical Huffman over bytes with the fixed table.
    std::vector<uint8_t> huffmanEncode(const std::string& text);
    bool huffmanDecode(const uint8_t* data, size_t size, size_t count, std::string& text);

}

#endif
