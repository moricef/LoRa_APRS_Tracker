#include <gtest/gtest.h>
#include "map_gps_filter.h"
#include "TinyGPS++.h"
#include <string>
#include <cmath>
#include <cstdio>
#include "mock_arduino.h"

// Helper pour générer NMEA GGA sentence
std::string generateGGA(double lat, double lon, unsigned int sats, double hdop, bool valid_location = true, bool valid_hdop = true) {
    char buf[128];
    if (!valid_location) {
        snprintf(buf, sizeof(buf), "$GPGGA,123456.00,,,,1,%02u,%.1f,0.0,M,0.0,M,,", sats, hdop);
    } else {
        double lat_abs = std::fabs(lat);
        int lat_deg = static_cast<int>(lat_abs);
        double lat_min = (lat_abs - lat_deg) * 60.0;
        char lat_str[16];
        snprintf(lat_str, sizeof(lat_str), "%02d%07.4f", lat_deg, lat_min);
        char ns = (lat >= 0) ? 'N' : 'S';

        double lon_abs = std::fabs(lon);
        int lon_deg = static_cast<int>(lon_abs);
        double lon_min = (lon_abs - lon_deg) * 60.0;
        char lon_str[16];
        snprintf(lon_str, sizeof(lon_str), "%03d%07.4f", lon_deg, lon_min);
        char ew = (lon >= 0) ? 'E' : 'W';

        char hdop_str[8] = "";
        if (valid_hdop) {
            snprintf(hdop_str, sizeof(hdop_str), "%.1f", hdop);
        }

        snprintf(buf, sizeof(buf), "$GPGGA,123456.00,%s,%c,%s,%c,1,%02u,%s,0.0,M,0.0,M,,", lat_str, ns, lon_str, ew, sats, hdop_str);
    }

    // Calcul checksum
    unsigned char checksum = 0;
    for (char* p = buf + 1; *p != '\0'; ++p) {
        checksum ^= static_cast<unsigned char>(*p);
    }
    char checksum_str[3];
    snprintf(checksum_str, sizeof(checksum_str), "%02X", checksum);

    // Ajout *checksum\r\n
    std::string nmea = std::string(buf) + "*" + checksum_str + "\r\n";
    return nmea;
}

// Helper pour feed NMEA to gps
void feedGPS(TinyGPSPlus& gps, double lat, double lon, unsigned int sats, double hdop, bool valid_location = true, bool valid_hdop = true) {
    std::string nmea = generateGGA(lat, lon, sats, hdop, valid_location, valid_hdop);
    for (char c : nmea) {
        gps.encode(c);
    }
}

// Fixture pour initialiser le filtre avant chaque test
class MapGPSFilterTest : public ::testing::Test {
protected:
    MapGPSFilter filter;

    void SetUp() override {
        filter.reset();
        MockArduino::reset_millis();  // Reset millis() pour chaque test
    }
};

TEST_F(MapGPSFilterTest, InitialState) {
    EXPECT_FALSE(filter.isFilteredValid());
    EXPECT_FALSE(filter.isIconGpsValid());
    EXPECT_EQ(0.0f, filter.getIconGpsLat());
    EXPECT_EQ(0.0f, filter.getIconGpsLon());
    EXPECT_EQ(0.0f, filter.getFilteredOwnLat());
    EXPECT_EQ(0.0f, filter.getFilteredOwnLon());
    EXPECT_EQ(0, filter.getOwnTraceCount());
    EXPECT_EQ(0, filter.getOwnTraceHead());

    float lat, lon;
    EXPECT_FALSE(filter.getUiPosition(&lat, &lon));
}

TEST_F(MapGPSFilterTest, UpdateWithInvalidLocation) {
    TinyGPSPlus gps;
    feedGPS(gps, 0.0, 0.0, 0, 99.9, false);
    filter.updateFilteredOwnPosition(gps);

    EXPECT_FALSE(filter.isFilteredValid());
    EXPECT_FALSE(filter.isIconGpsValid());
}

TEST_F(MapGPSFilterTest, UpdateWithLowSatellites) {
    TinyGPSPlus gps;
    feedGPS(gps, 37.7749, -122.4194, 2, 1.0);
    filter.updateFilteredOwnPosition(gps);

    EXPECT_FALSE(filter.isIconGpsValid());
    EXPECT_FALSE(filter.isFilteredValid());
}

TEST_F(MapGPSFilterTest, UpdateWithIconSatellites) {
    TinyGPSPlus gps;
    feedGPS(gps, 37.7749, -122.4194, 4, 1.0);
    filter.updateFilteredOwnPosition(gps);

    EXPECT_TRUE(filter.isIconGpsValid());
    EXPECT_NEAR(37.7749f, filter.getIconGpsLat(), 0.0001f);
    EXPECT_NEAR(-122.4194f, filter.getIconGpsLon(), 0.0001f);
    EXPECT_FALSE(filter.isFilteredValid());
}

TEST_F(MapGPSFilterTest, FirstFilteredUpdate) {
    TinyGPSPlus gps;
    feedGPS(gps, 37.7750, -122.4195, 7, 1.0);
    filter.updateFilteredOwnPosition(gps);

    EXPECT_TRUE(filter.isFilteredValid());
    EXPECT_NEAR(37.7750f, filter.getFilteredOwnLat(), 0.0001f);
    EXPECT_NEAR(-122.4195f, filter.getFilteredOwnLon(), 0.0001f);
    EXPECT_TRUE(filter.isIconGpsValid());
}

TEST_F(MapGPSFilterTest, SmallMovementNoUpdate) {
    TinyGPSPlus gps_first;
    feedGPS(gps_first, 37.7750, -122.4195, 7, 1.0);
    filter.updateFilteredOwnPosition(gps_first);

    TinyGPSPlus gps_small;
    feedGPS(gps_small, 37.7750 + 0.00005, -122.4195 + 0.00005, 7, 1.0);
    filter.updateFilteredOwnPosition(gps_small);

    EXPECT_NEAR(37.7750f, filter.getFilteredOwnLat(), 0.0001f);
    EXPECT_NEAR(-122.4195f, filter.getFilteredOwnLon(), 0.0001f);
}

TEST_F(MapGPSFilterTest, LargeMovementUpdate) {
    TinyGPSPlus gps_first;
    feedGPS(gps_first, 37.7750, -122.4195, 7, 1.0);
    filter.updateFilteredOwnPosition(gps_first);

    TinyGPSPlus gps_large;
    feedGPS(gps_large, 37.7750 + 0.001, -122.4195 + 0.001, 7, 1.0);
    filter.updateFilteredOwnPosition(gps_large);

    EXPECT_NEAR(37.7750f + 0.001f, filter.getFilteredOwnLat(), 0.0001f);
    EXPECT_NEAR(-122.4195f + 0.001f, filter.getFilteredOwnLon(), 0.0001f);
}

TEST_F(MapGPSFilterTest, AddTracePoint) {
    TinyGPSPlus gps;
    feedGPS(gps, 37.7750, -122.4195, 7, 1.0);
    filter.updateFilteredOwnPosition(gps);

    filter.addOwnTracePoint();
    EXPECT_EQ(1, filter.getOwnTraceCount());
    const TracePoint& point = filter.getOwnTracePoint(0);
    EXPECT_NEAR(37.7750f, point.lat, 0.0001f);
    EXPECT_NEAR(-122.4195f, point.lon, 0.0001f);
}

TEST_F(MapGPSFilterTest, AddCloseTracePointNoAdd) {
    TinyGPSPlus gps_first;
    feedGPS(gps_first, 37.7750, -122.4195, 7, 1.0);
    filter.updateFilteredOwnPosition(gps_first);
    filter.addOwnTracePoint();

    TinyGPSPlus gps_close;
    feedGPS(gps_close, 37.7750 + 0.00005, -122.4195 + 0.00005, 7, 1.0);
    filter.updateFilteredOwnPosition(gps_close);
    filter.addOwnTracePoint();

    EXPECT_EQ(1, filter.getOwnTraceCount());
}

TEST_F(MapGPSFilterTest, AddFarTracePointAdd) {
    TinyGPSPlus gps_first;
    feedGPS(gps_first, 37.7750, -122.4195, 7, 1.0);
    filter.updateFilteredOwnPosition(gps_first);
    filter.addOwnTracePoint();

    TinyGPSPlus gps_far;
    feedGPS(gps_far, 37.7750 + 0.0002, -122.4195 + 0.0002, 7, 1.0);
    filter.updateFilteredOwnPosition(gps_far);
    filter.addOwnTracePoint();

    EXPECT_EQ(2, filter.getOwnTraceCount());
    const TracePoint& point2 = filter.getOwnTracePoint(1);
    EXPECT_NEAR(37.7750f + 0.0002f, point2.lat, 0.0001f);
    EXPECT_NEAR(-122.4195f + 0.0002f, point2.lon, 0.0001f);
}

TEST_F(MapGPSFilterTest, CircularBufferWrapAround) {
    for (int i = 0; i < TRACE_MAX_POINTS; ++i) {
        TinyGPSPlus gps;
        feedGPS(gps, 37.7750 + i * 0.001, -122.4195, 7, 1.0);
        filter.updateFilteredOwnPosition(gps);
        filter.addOwnTracePoint();
    }
    EXPECT_EQ(TRACE_MAX_POINTS, filter.getOwnTraceCount());
    EXPECT_EQ(0, filter.getOwnTraceHead());

    TinyGPSPlus gps_extra;
    feedGPS(gps_extra, 40.0, -120.0, 7, 1.0);
    filter.updateFilteredOwnPosition(gps_extra);
    filter.addOwnTracePoint();
    EXPECT_EQ(TRACE_MAX_POINTS, filter.getOwnTraceCount());
    EXPECT_EQ(1, filter.getOwnTraceHead());

    const TracePoint& oldest = filter.getOwnTracePoint(0);
    EXPECT_NEAR(37.7750f + 1 * 0.001f, oldest.lat, 0.0001f);
}

TEST_F(MapGPSFilterTest, GetUiPositionPriority) {
    TinyGPSPlus gps_icon;
    feedGPS(gps_icon, 37.7749, -122.4194, 4, 1.0);
    filter.updateFilteredOwnPosition(gps_icon);

    float lat, lon;
    EXPECT_TRUE(filter.getUiPosition(&lat, &lon));
    EXPECT_NEAR(37.7749f, lat, 0.0001f);
    EXPECT_NEAR(-122.4194f, lon, 0.0001f);

    TinyGPSPlus gps_filtered;
    feedGPS(gps_filtered, 37.7750, -122.4195, 7, 1.0);
    filter.updateFilteredOwnPosition(gps_filtered);
    EXPECT_TRUE(filter.getUiPosition(&lat, &lon));
    EXPECT_NEAR(37.7750f, lat, 0.0001f);
    EXPECT_NEAR(-122.4195f, lon, 0.0001f);
}

TEST_F(MapGPSFilterTest, ResetClearsAll) {
    TinyGPSPlus gps;
    feedGPS(gps, 37.7750, -122.4195, 7, 1.0);
    filter.updateFilteredOwnPosition(gps);
    filter.addOwnTracePoint();

    filter.reset();

    EXPECT_FALSE(filter.isFilteredValid());
    EXPECT_FALSE(filter.isIconGpsValid());
    EXPECT_EQ(0, filter.getOwnTraceCount());
    float lat, lon;
    EXPECT_FALSE(filter.getUiPosition(&lat, &lon));
}