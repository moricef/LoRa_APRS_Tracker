// Copied from moricef/LoRa_APRS_iGate, branch rxt, commit b433e56 (src/rxt_protocol.cpp).
// Keep identical: the host tests and test vectors live in that repository.
#include "rxt_protocol.h"

#include <algorithm>
#include <cctype>
#include <cstdint>

namespace {

// RXT v2 trailer content: 1 to MAX_TUPLES tuples of TUPLE_BYTES printable
// characters (ID RSSI SNR FO TTH), each in the '!'..'z' range.
bool isRxtContent(const std::string& content) {
    if (content.empty() || content.size() > RXT_Protocol::TUPLE_BYTES * RXT_Protocol::MAX_TUPLES ||
        content.size() % RXT_Protocol::TUPLE_BYTES != 0) return false;
    for (char c : content) {
        if (c < 33 || c > 122) return false;
    }
    return true;
}

size_t trailerStart(const std::string& packet) {
    if (packet.size() <= 6 || packet.back() != '}') return std::string::npos;
    size_t lowerBound = packet.size() > 20 ? packet.size() - 20 : 0;
    size_t start = packet.rfind('{', packet.size() - 2);
    return start != std::string::npos && start > lowerBound ? start : std::string::npos;
}

bool isAliasSeparator(char character) {
    return character == ' ' || character == ',' || character == '\t' ||
           character == '\r' || character == '\n';
}

bool matchesNumberedAlias(const std::string& upper, const std::string& alias) {
    if (alias.empty() || upper.compare(0, alias.size(), alias) != 0) return false;
    if (upper.size() == alias.size()) return true;

    size_t dash = upper.find('-', alias.size() + 1);
    if (dash == std::string::npos || dash + 1 >= upper.size()) return false;
    for (size_t i = alias.size(); i < dash; ++i) {
        if (!std::isdigit(static_cast<unsigned char>(upper[i]))) return false;
    }
    for (size_t i = dash + 1; i < upper.size(); ++i) {
        if (!std::isdigit(static_cast<unsigned char>(upper[i]))) return false;
    }
    return dash > alias.size();
}

bool isRoutingAlias(const std::string& node, const std::string& regionalAliases) {
    std::string upper = node;
    std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });

    if (upper == "RELAY") return true;

    if (matchesNumberedAlias(upper, "WIDE") || matchesNumberedAlias(upper, "TRACE")) {
        return true;
    }

    size_t start = 0;
    while (start < regionalAliases.size()) {
        while (start < regionalAliases.size() && isAliasSeparator(regionalAliases[start])) ++start;
        if (start >= regionalAliases.size()) break;
        size_t end = start;
        while (end < regionalAliases.size() && !isAliasSeparator(regionalAliases[end])) ++end;
        std::string alias = regionalAliases.substr(start, end - start);
        std::transform(alias.begin(), alias.end(), alias.begin(), [](unsigned char c) {
            return static_cast<char>(std::toupper(c));
        });
        if (matchesNumberedAlias(upper, alias)) return true;
        start = end + 1;
    }
    return false;
}

}

namespace RXT_Protocol {

bool fitsLoRaPayload(size_t packetBytes) {
    return packetBytes <= LORA_MAX_PAYLOAD_BYTES - LORA_APRS_PREFIX_BYTES;
}

std::string attachTrailerWithinLimit(const std::string& packet, const std::string& newTuple,
                                     const std::string& regionalAliases) {
    if (!fitsLoRaPayload(packet.size())) return packet;
    const std::string candidate = attachTrailer(packet, newTuple, regionalAliases);
    return fitsLoRaPayload(candidate.size()) ? candidate : packet;
}

char fingerprint(const std::string& callsign) {
    uint32_t hash = 0x811C9DC5u;
    for (unsigned char c : callsign) {
        if (c == '*') continue;
        hash ^= static_cast<uint8_t>(std::toupper(c));
        hash *= 0x01000193u;
    }
    return static_cast<char>(33 + hash % 89);
}

bool trailerMatchesPath(const std::string& packet, const std::string& content,
                        const std::string& regionalAliases) {
    if (!isRxtContent(content)) return false;
    std::string ids;
    for (const std::string& node : usedPathNodes(packet, regionalAliases)) {
        ids += fingerprint(node);
    }
    for (size_t i = 0; i < content.size(); i += TUPLE_BYTES) {
        if (ids.find(content[i]) == std::string::npos) return false;
    }
    return true;
}

std::string attachTrailer(const std::string& packet, const std::string& newTuple,
                          const std::string& regionalAliases) {
    size_t start = trailerStart(packet);
    if (start == std::string::npos) return packet + "{" + newTuple + "}";

    std::string content = packet.substr(start + 1, packet.size() - start - 2);
    // A final {...} that is not RXT data from digis of this path is user
    // text: leave it intact and open a new trailer after it.
    if (!trailerMatchesPath(packet, content, regionalAliases)) {
        return packet + "{" + newTuple + "}";
    }
    if (content.size() >= TUPLE_BYTES * MAX_TUPLES) return packet;
    return packet.substr(0, start) + "{" + content + newTuple + "}";
}

std::string stripTrailer(const std::string& packet, std::string* outTuples,
                         const std::string& regionalAliases) {
    size_t start = trailerStart(packet);
    if (start != std::string::npos) {
        std::string content = packet.substr(start + 1, packet.size() - start - 2);
        if (trailerMatchesPath(packet, content, regionalAliases)) {
            if (outTuples != nullptr) *outTuples = content;
            return packet.substr(0, start);
        }
    }
    if (outTuples != nullptr) outTuples->clear();
    return packet;
}

std::vector<int> attributeTuples(const std::string& content,
                                 const std::vector<std::string>& usedNodes) {
    std::vector<int> owners;
    size_t nextNode = 0;
    for (size_t i = 0; i + TUPLE_BYTES <= content.size(); i += TUPLE_BYTES) {
        size_t match = nextNode;
        while (match < usedNodes.size() && fingerprint(usedNodes[match]) != content[i]) match++;
        if (match < usedNodes.size()) {
            owners.push_back(static_cast<int>(match));
            nextNode = match + 1;
        } else {
            owners.push_back(-1);
        }
    }
    return owners;
}

std::string stripTrailerShape(const std::string& information) {
    size_t start = trailerStart(information);
    if (start == std::string::npos) return information;
    std::string content = information.substr(start + 1, information.size() - start - 2);
    return isRxtContent(content) ? information.substr(0, start) : information;
}

std::vector<std::string> usedPathNodes(const std::string& packet,
                                       const std::string& regionalAliases) {
    std::vector<std::string> pathElements;
    size_t gt = packet.find('>');
    size_t colon = packet.find(':', gt == std::string::npos ? 0 : gt + 1);
    size_t comma = packet.find(',', gt == std::string::npos ? 0 : gt + 1);
    if (gt == std::string::npos || colon == std::string::npos ||
        comma == std::string::npos || comma > colon) return {};

    std::string path = packet.substr(comma + 1, colon - comma - 1);
    int lastStarredIndex = -1;
    size_t start = 0;
    while (start <= path.size()) {
        size_t end = path.find(',', start);
        std::string node = path.substr(start, end == std::string::npos ? std::string::npos : end - start);
        bool starred = node.find('*') != std::string::npos;
        node.erase(std::remove(node.begin(), node.end(), '*'), node.end());
        if (!node.empty()) {
            pathElements.push_back(node);
            if (starred) lastStarredIndex = static_cast<int>(pathElements.size()) - 1;
        }
        if (end == std::string::npos) break;
        start = end + 1;
    }

    if (lastStarredIndex < 0) return {};
    pathElements.resize(static_cast<size_t>(lastStarredIndex) + 1);

    // WIDEn-N/TRACE aliases describe routing work, not physical transmitters.
    // Some LoRa digis leave a consumed alias starred alongside their own
    // callsign; retaining it would invent links such as WIDE2-2 -> F4MLV-10.
    std::vector<std::string> pathNodes;
    for (const std::string& node : pathElements) {
        if (!isRoutingAlias(node, regionalAliases)) pathNodes.push_back(node);
    }
    return pathNodes;
}

bool isAprsMessage(const std::string& packet) {
    std::string current = packet;

    // A third-party packet starts its information field with '}' followed by
    // another complete TNC2 frame. Unwrap each level before deciding which
    // APRS data type is actually being carried.
    while (true) {
        size_t gt = current.find('>');
        size_t colon = current.find(':', gt == std::string::npos ? 0 : gt + 1);
        if (gt == std::string::npos || colon == std::string::npos || colon + 1 >= current.size()) {
            return false;
        }

        char dataType = current[colon + 1];
        if (dataType == ':') return true;
        if (dataType != '}') return false;

        current = current.substr(colon + 2);
    }
}

}
