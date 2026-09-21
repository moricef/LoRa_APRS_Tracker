#include "mice_utils.h"
#include <gtest/gtest.h>

TEST(MicELongitude, NormalizesZeroToNineDegreesEastAndWest) {
    EXPECT_NEAR(MicEUtils::normalizeDecodedLongitude(191.370833f), 1.370833f, 0.00001f);
    EXPECT_NEAR(MicEUtils::normalizeDecodedLongitude(-191.370833f), -1.370833f, 0.00001f);
}

TEST(MicELongitude, NormalizesOneHundredToOneHundredNineDegreesEastAndWest) {
    EXPECT_NEAR(MicEUtils::normalizeDecodedLongitude(180.250000f), 100.250000f, 0.000001f);
    EXPECT_NEAR(MicEUtils::normalizeDecodedLongitude(-189.750000f), -109.750000f, 0.000001f);
}

TEST(MicELongitude, LeavesOtherValidLongitudesUnchanged) {
    EXPECT_FLOAT_EQ(MicEUtils::normalizeDecodedLongitude(71.625f), 71.625f);
    EXPECT_FLOAT_EQ(MicEUtils::normalizeDecodedLongitude(-71.625f), -71.625f);
    EXPECT_FLOAT_EQ(MicEUtils::normalizeDecodedLongitude(110.0f), 110.0f);
    EXPECT_FLOAT_EQ(MicEUtils::normalizeDecodedLongitude(179.999f), 179.999f);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
