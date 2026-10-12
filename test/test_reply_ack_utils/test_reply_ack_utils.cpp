#include <gtest/gtest.h>
#include "reply_ack_utils.h"

using namespace REPLY_ACK_Utils;

TEST(ReplyAck, FormatsTwoCharacterBase36Numbers) {
    EXPECT_EQ(formatMessageNumber(1), String("01"));
    EXPECT_EQ(formatMessageNumber(35), String("0Z"));
    EXPECT_EQ(formatMessageNumber(36), String("10"));
    EXPECT_EQ(formatMessageNumber(MESSAGE_NUMBER_SLOTS), String("ZZ"));
}

TEST(ReplyAck, AckedNumberKeepsMessageNumberOnly) {
    EXPECT_EQ(ackedNumber(String("12}AB")), String("12"));
    EXPECT_EQ(ackedNumber(String("12}")), String("12"));
    EXPECT_EQ(ackedNumber(String("123")), String("123"));
}

TEST(ReplyAck, ParsesReplyAckFormOnly) {
    String mm, aa;
    ASSERT_TRUE(parseReplyAck(String("hello{12}AB"), mm, aa));
    EXPECT_EQ(mm, String("12"));
    EXPECT_EQ(aa, String("AB"));
    ASSERT_TRUE(parseReplyAck(String("hello{12}"), mm, aa));
    EXPECT_EQ(mm, String("12"));
    EXPECT_EQ(aa, String(""));
    EXPECT_FALSE(parseReplyAck(String("hello{123"), mm, aa));
    EXPECT_FALSE(parseReplyAck(String("hello"), mm, aa));
    EXPECT_FALSE(parseReplyAck(String("hello{}AB"), mm, aa));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
