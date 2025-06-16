#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "hesai_lidar_driver/cone_detector.h"

namespace hesai_lidar_driver {

/**
 * @brief Test fixture for ConeDetector unit tests
 */
class ConeDetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create dummy node handles for testing
        nh_ = std::make_shared<ros::NodeHandle>();
        private_nh_ = std::make_shared<ros::NodeHandle>("~");
        
        detector_ = std::make_shared<ConeDetector>(*nh_, *private_nh_);
        ASSERT_TRUE(detector_->initialize()) << "Failed to initialise cone detector";
        
        // Create test scenarios
        createEmptyPointCloud();
        createSingleConePointCloud();
        createMultipleConePointCloud();
        createNoisePointCloud();
    }

    void TearDown() override {
        detector_.reset();
        private_nh_.reset();
        nh_.reset();
    }

    void createEmptyPointCloud() {
        empty_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        empty_cloud_->width = 0;
        empty_cloud_->height = 1;
        empty_cloud_->is_dense = true;
    }

    void createSingleConePointCloud() {
        single_cone_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        
        // Create cone-like cluster at (5, 0, 0) with typical cone dimensions
        float cone_x = 5.0f, cone_y = 0.0f;
        float cone_height = 0.4f;  // 40cm typical cone height
        float cone_radius = 0.15f;  // 15cm radius at base
        
        // Generate points in cone shape
        for (float z = 0.0f; z <= cone_height; z += 0.02f) {
            float radius_at_height = cone_radius * (1.0f - z / cone_height);  // Tapered cone
            int points_at_height = static_cast<int>(radius_at_height * 100);  // Density
            
            for (int i = 0; i < points_at_height; ++i) {
                float angle = 2.0f * M_PI * i / points_at_height;
                float r = radius_at_height * (0.8f + 0.4f * static_cast<float>(rand()) / RAND_MAX);  // Some variation
                
                pcl::PointXYZI point;
                point.x = cone_x + r * cos(angle);
                point.y = cone_y + r * sin(angle);
                point.z = z;
                point.intensity = 150.0f + 50.0f * static_cast<float>(rand()) / RAND_MAX;
                
                single_cone_cloud_->points.push_back(point);
            }
        }
        
        single_cone_cloud_->width = single_cone_cloud_->points.size();
        single_cone_cloud_->height = 1;
        single_cone_cloud_->is_dense = true;
    }

    void createMultipleConePointCloud() {
        multiple_cone_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        
        // Create two cone-like clusters
        std::vector<std::pair<float, float>> cone_positions = {{3.0f, -2.0f}, {7.0f, 3.0f}};
        
        for (const auto& pos : cone_positions) {
            float cone_x = pos.first, cone_y = pos.second;
            float cone_height = 0.35f;
            float cone_radius = 0.12f;
            
            // Generate points in cone shape
            for (float z = 0.0f; z <= cone_height; z += 0.03f) {
                float radius_at_height = cone_radius * (1.0f - z / cone_height);
                int points_at_height = static_cast<int>(radius_at_height * 80);
                
                for (int i = 0; i < points_at_height; ++i) {
                    float angle = 2.0f * M_PI * i / points_at_height;
                    float r = radius_at_height * (0.7f + 0.6f * static_cast<float>(rand()) / RAND_MAX);
                    
                    pcl::PointXYZI point;
                    point.x = cone_x + r * cos(angle);
                    point.y = cone_y + r * sin(angle);
                    point.z = z;
                    point.intensity = 140.0f + 30.0f * static_cast<float>(rand()) / RAND_MAX;
                    
                    multiple_cone_cloud_->points.push_back(point);
                }
            }
        }
        
        multiple_cone_cloud_->width = multiple_cone_cloud_->points.size();
        multiple_cone_cloud_->height = 1;
        multiple_cone_cloud_->is_dense = true;
    }

    void createNoisePointCloud() {
        noise_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        
        // Create scattered noise points (should not be detected as cones)
        for (int i = 0; i < 100; ++i) {
            pcl::PointXYZI point;
            point.x = 20.0f * (static_cast<float>(rand()) / RAND_MAX - 0.5f);  // -10 to 10
            point.y = 20.0f * (static_cast<float>(rand()) / RAND_MAX - 0.5f);
            point.z = 2.0f * static_cast<float>(rand()) / RAND_MAX;  // 0 to 2
            point.intensity = 100.0f * static_cast<float>(rand()) / RAND_MAX;
            
            noise_cloud_->points.push_back(point);
        }
        
        noise_cloud_->width = noise_cloud_->points.size();
        noise_cloud_->height = 1;
        noise_cloud_->is_dense = true;
    }

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<ros::NodeHandle> private_nh_;
    std::shared_ptr<ConeDetector> detector_;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr empty_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr single_cone_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr multiple_cone_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr noise_cloud_;
};

/**
 * @brief Test detector initialisation
 */
TEST_F(ConeDetectorTest, InitializationTest) {
    EXPECT_TRUE(detector_ != nullptr);
    
    // Test re-initialisation
    EXPECT_TRUE(detector_->initialize());
}

/**
 * @brief Test detection with empty point cloud
 */
TEST_F(ConeDetectorTest, EmptyCloudDetectionTest) {
    std::vector<DetectedCone> detected_cones;
    
    bool result = detector_->detectCones(empty_cloud_, detected_cones);
    
    // Should handle empty cloud gracefully
    EXPECT_TRUE(result) << "Empty cloud detection should succeed";
    EXPECT_TRUE(detected_cones.empty()) << "No cones should be detected in empty cloud";
}

/**
 * @brief Test detection with single cone
 */
TEST_F(ConeDetectorTest, SingleConeDetectionTest) {
    std::vector<DetectedCone> detected_cones;
    
    bool result = detector_->detectCones(single_cone_cloud_, detected_cones);
    
    EXPECT_TRUE(result) << "Single cone detection should succeed";
    EXPECT_GE(detected_cones.size(), 1u) << "Should detect at least one cone";
    
    if (!detected_cones.empty()) {
        const auto& cone = detected_cones[0];
        
        // Check cone position is reasonable (should be near 5, 0, ~0.2)
        EXPECT_NEAR(cone.position.x, 5.0f, 1.0f) << "Cone X position should be near expected location";
        EXPECT_NEAR(cone.position.y, 0.0f, 1.0f) << "Cone Y position should be near expected location";
        EXPECT_GT(cone.position.z, 0.0f) << "Cone Z position should be above ground";
        EXPECT_LT(cone.position.z, 1.0f) << "Cone Z position should be reasonable height";
        
        // Check cone properties
        EXPECT_EQ(cone.colour, "unknown") << "Cone colour should be 'unknown'";
        EXPECT_GT(cone.confidence, 0.0f) << "Cone confidence should be positive";
        EXPECT_LE(cone.confidence, 1.0f) << "Cone confidence should not exceed 1.0";
        EXPECT_GT(cone.point_count, 0) << "Cone should have associated points";
    }
}

/**
 * @brief Test detection with multiple cones
 */
TEST_F(ConeDetectorTest, MultipleConeDetectionTest) {
    std::vector<DetectedCone> detected_cones;
    
    bool result = detector_->detectCones(multiple_cone_cloud_, detected_cones);
    
    EXPECT_TRUE(result) << "Multiple cone detection should succeed";
    EXPECT_GE(detected_cones.size(), 1u) << "Should detect at least one cone";
    EXPECT_LE(detected_cones.size(), 3u) << "Should not detect too many false positives";
    
    // Ideally should detect 2 cones, but allow some tolerance
    if (detected_cones.size() >= 2) {
        // Check that detected cones are in different locations
        float distance = sqrt(pow(detected_cones[0].position.x - detected_cones[1].position.x, 2) +
                             pow(detected_cones[0].position.y - detected_cones[1].position.y, 2));
        EXPECT_GT(distance, 2.0f) << "Detected cones should be spatially separated";
    }
}

/**
 * @brief Test detection with noise (should not detect cones)
 */
TEST_F(ConeDetectorTest, NoiseRejectionTest) {
    std::vector<DetectedCone> detected_cones;
    
    bool result = detector_->detectCones(noise_cloud_, detected_cones);
    
    EXPECT_TRUE(result) << "Noise detection should succeed";
    
    // Should not detect cones in random noise
    EXPECT_LE(detected_cones.size(), 1u) << "Should not detect cones in noise cloud";
    
    // If any detections, confidence should be low
    for (const auto& cone : detected_cones) {
        EXPECT_LT(cone.confidence, 0.7f) << "Noise detections should have low confidence";
    }
}

/**
 * @brief Test ROS message interface
 */
TEST_F(ConeDetectorTest, ROSMessageInterfaceTest) {
    // Convert PCL cloud to ROS message
    sensor_msgs::PointCloud2 input_msg;
    pcl::toROSMsg(*single_cone_cloud_, input_msg);
    input_msg.header.frame_id = "hesai_lidar";
    input_msg.header.stamp = ros::Time::now();
    
    std::vector<DetectedCone> detected_cones;
    sensor_msgs::PointCloud2::ConstPtr input_ptr = 
        boost::make_shared<sensor_msgs::PointCloud2>(input_msg);
    
    bool result = detector_->detectCones(input_ptr, detected_cones);
    
    EXPECT_TRUE(result) << "ROS message detection should succeed";
    // Results should be similar to PCL interface test
}

/**
 * @brief Test detection performance
 */
TEST_F(ConeDetectorTest, DetectionPerformanceTest) {
    std::vector<DetectedCone> detected_cones;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    bool result = detector_->detectCones(multiple_cone_cloud_, detected_cones);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    EXPECT_TRUE(result);
    
    // Detection should complete within reasonable time
    EXPECT_LT(duration.count(), 500) << "Detection should complete within 500ms for test data";
    
    std::cout << "Detection time: " << duration.count() << " ms for " 
              << multiple_cone_cloud_->size() << " input points" << std::endl;
}

} // namespace hesai_lidar_driver

/**
 * @brief Main function for running cone detector tests
 */
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_cone_detector");
    
    // Seed random number generator for consistent test data
    srand(42);
    
    // Run tests
    int result = RUN_ALL_TESTS();
    
    return result;
}
