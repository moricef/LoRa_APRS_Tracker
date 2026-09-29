#ifndef REPLY_ACK_UTILS_H_
#define REPLY_ACK_UTILS_H_

#include <Arduino.h>

// APRS reply-ack message numbers (aprs.org/aprs11/replyacks.txt).
// Outgoing messages carry "{MM}AA": MM is our two-character number, AA the
// last MM received from the peer in the reply-ack form (empty if none).
namespace REPLY_ACK_Utils {

    // Number of two-character base-36 message numbers, "01".."ZZ".
    const int MESSAGE_NUMBER_SLOTS = 36 * 36 - 1;

    // Renders n in [1, MESSAGE_NUMBER_SLOTS] as two base-36 characters.
    inline String formatMessageNumber(int n) {
        const char* digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
        String out;
        out += digits[(n / 36) % 36];
        out += digits[n % 36];
        return out;
    }

    // Message number carried by an ack or rej: "MM" from "MM}AA" (step 7).
    inline String ackedNumber(const String& idAfterAck) {
        int close = idAfterAck.indexOf('}');
        return close >= 0 ? idAfterAck.substring(0, close) : idAfterAck;
    }

    // Splits a received message payload "text{MM}AA". Returns true only for
    // the reply-ack form, with mm and aa set (aa may be empty).
    inline bool parseReplyAck(const String& payload, String& mm, String& aa) {
        int open = payload.indexOf('{');
        if (open < 0) return false;
        String number = payload.substring(open + 1);
        int close = number.indexOf('}');
        if (close <= 0) return false;
        mm = number.substring(0, close);
        aa = number.substring(close + 1);
        return true;
    }

}

#endif
