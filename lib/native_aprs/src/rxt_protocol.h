// Copied from moricef/LoRa_APRS_iGate, branch rxt, commit b433e56 (include/rxt_protocol.h).
// Keep identical: the host tests and test vectors live in that repository.
#ifndef RXT_PROTOCOL_H_
#define RXT_PROTOCOL_H_

#include <cstddef>
#include <string>
#include <vector>

// RXT v2 (docs/RXT_V2.md): each tuple is ID RSSI SNR FO TTH, where ID is the
// fingerprint of the relaying digi's callsign as written in the path.
namespace RXT_Protocol {

    constexpr size_t TUPLE_BYTES = 5;
    constexpr size_t MAX_TUPLES = 3;
    constexpr size_t LORA_MAX_PAYLOAD_BYTES = 255;
    constexpr size_t LORA_APRS_PREFIX_BYTES = 3;
    bool fitsLoRaPayload(size_t packetBytes);

    // 32-bit FNV-1a of the upper-case callsign without '*', mod 89, plus 33.
    char fingerprint(const std::string& callsign);

    // True when content is v2-shaped and every tuple ID designates a digi
    // actually used in the packet path.
    bool trailerMatchesPath(const std::string& packet, const std::string& content,
                            const std::string& regionalAliases = "");

    // Leaves the original packet (including existing tuples) intact if the
    // new tuple does not fit. Oversize originals must be rejected by caller.
    std::string attachTrailerWithinLimit(const std::string& packet, const std::string& newTuple,
                                         const std::string& regionalAliases = "");
    std::string attachTrailer(const std::string& packet, const std::string& newTuple,
                              const std::string& regionalAliases = "");
    // Removes the final {...} only if it passes trailerMatchesPath().
    std::string stripTrailer(const std::string& packet, std::string* outTuples = nullptr,
                             const std::string& regionalAliases = "");
    // Shape-only removal for an information field without path (duplicate
    // keys only; never used for data sent to APRS-IS).
    std::string stripTrailerShape(const std::string& information);
    // For each tuple of content, the index in usedNodes of the relay that
    // produced it, or -1 when its ID matches no remaining node. Tuples are
    // matched in path order, since digis append them in relay order.
    std::vector<int> attributeTuples(const std::string& content,
                                     const std::vector<std::string>& usedNodes);
    std::vector<std::string> usedPathNodes(
        const std::string& packet,
        const std::string& regionalAliases = "");
    bool isAprsMessage(const std::string& packet);

}

#endif
