#include "kiss_utils.h"
#include <gtest/gtest.h>

TEST(KissEncoding, EncodesClassicAddresses) {
    String encoded = KISS_Utils::encodeKISS("F4MLV-7>APLRG1,WIDE1-1,WIDE2-1:Test");
    ASSERT_FALSE(encoded.isEmpty());
    bool dataFrame = false;
    EXPECT_EQ(KISS_Utils::decodeKISS(encoded, dataFrame).s,
              "F4MLV-7>APLRG1,WIDE1-1,WIDE2-1:Test");
    EXPECT_TRUE(dataFrame);
}

TEST(KissEncoding, RejectsExtendedSourceWithoutChangingItToZero) {
    EXPECT_TRUE(KISS_Utils::encodeKISS("F4MLV-GS>APLRG1:Test").isEmpty());
}

TEST(KissEncoding, RejectsExtendedDestinationAndRelay) {
    EXPECT_TRUE(KISS_Utils::encodeKISS("F4MLV-7>APLR-GS:Test").isEmpty());
    EXPECT_TRUE(KISS_Utils::encodeKISS("F4MLV-7>APLRG1,WIDE-GS:Test").isEmpty());
}

TEST(KissEncoding, RejectsSsidAboveFifteen) {
    EXPECT_TRUE(KISS_Utils::encodeKISS("F4MLV-16>APLRG1:Test").isEmpty());
    EXPECT_TRUE(KISS_Utils::encodeKISS("F4MLV-7>APLRG1,WIDE1-16:Test").isEmpty());
}

TEST(KissEncoding, AcceptsMaximumClassicSsid) {
    String encoded = KISS_Utils::encodeKISS("F4MLV-15>APLRG1:Test");
    ASSERT_FALSE(encoded.isEmpty());
    bool dataFrame = false;
    EXPECT_EQ(KISS_Utils::decodeKISS(encoded, dataFrame).s, "F4MLV-15>APLRG1:Test");
    EXPECT_TRUE(dataFrame);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
