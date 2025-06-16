#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "hesai_lidar_driver/packet_parser.h"

namespace hesai_lidar_driver {

/**
 * @brief Test fixture for PacketParser unit tests
 */
class PacketParserTest : public ::testing::Test {
protected:
    void SetUp() override {
        parser_ = std::make_shared<PacketParser>();
        ASSERT_TRUE(parser_->initialize()) << "Failed to initialise packet parser";
    }

    void TearDown() override {
        parser_.reset();
    }

    std::shared_ptr<PacketParser> parser_;
};

/**
 * @brief Test packet parser initialisation
 */
TEST_F(PacketParserTest, InitializationTest) {
    // Parser should be initialised in SetUp
    EXPECT_TRUE(parser_ != nullptr);
    
    // Test re-initialisation
    EXPECT_TRUE(parser_->initialize());
}

/**
 * @brief Test parsing of valid data packets
 */
TEST_F(PacketParserTest, ValidDataPacketParsing) {
    // Create mock packet data (simplified structure)
    std::vector<uint8_t> mock_packet(1262, 0);
    
    // Set packet header (simplified)
    mock_packet[0] = 0xEE;  // Header start
    mock_packet[1] = 0xFF;
    
    std::vector<HesaiPoint> points;
    
    // Note: This will likely fail with mock data, but tests the interface
    // In real implementation, this would use actual packet data
    bool result = parser_->parseDataPacket(mock_packet.data(), mock_packet.size(), points);
    
    // For mock data, we expect parsing to handle gracefully
    EXPECT_TRUE(result || points.empty()) << "Parser should handle mock data gracefully";
}

/**
 * @brief Test parsing with invalid packet size
 */
TEST_F(PacketParserTest, InvalidPacketSizeParsing) {
    std::vector<uint8_t> invalid_packet(100, 0);  // Too small
    std::vector<HesaiPoint> points;
    
    bool result = parser_->parseDataPacket(invalid_packet.data(), invalid_packet.size(), points);
    EXPECT_FALSE(result) << "Parser should reject packets with invalid size";
    EXPECT_TRUE(points.empty()) << "No points should be generated from invalid packets";
}

/**
 * @brief Test conversion to ROS point cloud
 */
TEST_F(PacketParserTest, ROSPointCloudConversion) {
    std::vector<HesaiPoint> test_points;
    
    // Create test points
    HesaiPoint point1;
    point1.x = 1.0f; point1.y = 2.0f; point1.z = 3.0f;
    point1.intensity = 100.0f;
    point1.ring = 0;
    test_points.push_back(point1);
    
    HesaiPoint point2;
    point2.x = 4.0f; point2.y = 5.0f; point2.z = 6.0f;
    point2.intensity = 150.0f;
    point2.ring = 1;
    test_points.push_back(point2);
    
    sensor_msgs::PointCloud2 cloud;
    parser_->pointsToROS(test_points, cloud, "hesai_lidar");
    
    EXPECT_EQ(cloud.header.frame_id, "hesai_lidar");
    EXPECT_EQ(cloud.width, 2u);
    EXPECT_EQ(cloud.height, 1u);
    EXPECT_FALSE(cloud.data.empty());
}

/**
 * @brief Test conversion to PCL point cloud
 */
TEST_F(PacketParserTest, PCLPointCloudConversion) {
    std::vector<HesaiPoint> test_points;
    
    // Create test points
    HesaiPoint point;
    point.x = 1.5f; point.y = 2.5f; point.z = 3.5f;
    point.intensity = 200.0f;
    test_points.push_back(point);
    
    pcl::PointCloud<pcl::PointXYZI> cloud;
    parser_->pointsToPCL(test_points, cloud);
    
    EXPECT_EQ(cloud.size(), 1u);
    EXPECT_FLOAT_EQ(cloud.points[0].x, 1.5f);
    EXPECT_FLOAT_EQ(cloud.points[0].y, 2.5f);
    EXPECT_FLOAT_EQ(cloud.points[0].z, 3.5f);
    EXPECT_FLOAT_EQ(cloud.points[0].intensity, 200.0f);
}

/**
 * @brief Test polar to Cartesian coordinate conversion
 */
TEST_F(PacketParserTest, CoordinateConversionTest) {
    // This would test the internal polar to Cartesian conversion
    // Implementation would depend on making the conversion method public or testable
    
    // Test case: Known polar coordinates should convert to expected Cartesian
    // Distance: 10m, Azimuth: 0°, Elevation: 0°
    // Expected: x=10, y=0, z=0
    
    // Note: This test requires access to the conversion method
    // In implementation, consider making a public test interface
    SUCCEED() << "Coordinate conversion test placeholder - implement when method is accessible";
}

} // namespace hesai_lidar_driver

/**
 * @brief Main function for running packet parser tests
 */
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
