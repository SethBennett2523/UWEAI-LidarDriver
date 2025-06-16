#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "hesai_lidar_driver/point_cloud_processor.h"

namespace hesai_lidar_driver {

/**
 * @brief Test fixture for PointCloudProcessor unit tests
 */
class PointCloudProcessorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create dummy node handles for testing
        nh_ = std::make_shared<ros::NodeHandle>();
        private_nh_ = std::make_shared<ros::NodeHandle>("~");
        
        processor_ = std::make_shared<PointCloudProcessor>(*nh_, *private_nh_);
        ASSERT_TRUE(processor_->initialize()) << "Failed to initialise point cloud processor";
        
        // Create test point cloud
        createTestPointCloud();
    }

    void TearDown() override {
        processor_.reset();
        private_nh_.reset();
        nh_.reset();
    }

    void createTestPointCloud() {
        test_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        
        // Create a grid of test points
        for (float x = -10.0f; x <= 10.0f; x += 1.0f) {
            for (float y = -10.0f; y <= 10.0f; y += 1.0f) {
                for (float z = -2.0f; z <= 5.0f; z += 1.0f) {
                    pcl::PointXYZI point;
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    point.intensity = 100.0f + (x + y + z) * 10.0f;  // Varying intensity
                    test_cloud_->points.push_back(point);
                }
            }
        }
        
        test_cloud_->width = test_cloud_->points.size();
        test_cloud_->height = 1;
        test_cloud_->is_dense = true;
    }

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<ros::NodeHandle> private_nh_;
    std::shared_ptr<PointCloudProcessor> processor_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr test_cloud_;
};

/**
 * @brief Test processor initialisation
 */
TEST_F(PointCloudProcessorTest, InitializationTest) {
    EXPECT_TRUE(processor_ != nullptr);
    
    // Test re-initialisation
    EXPECT_TRUE(processor_->initialize());
}

/**
 * @brief Test basic point cloud processing
 */
TEST_F(PointCloudProcessorTest, BasicProcessingTest) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    bool result = processor_->processPointCloud(test_cloud_, output_cloud);
    
    EXPECT_TRUE(result) << "Point cloud processing should succeed";
    EXPECT_TRUE(output_cloud->size() > 0) << "Processed cloud should contain points";
    EXPECT_LE(output_cloud->size(), test_cloud_->size()) << "Processed cloud should not be larger than input";
}

/**
 * @brief Test processing with empty point cloud
 */
TEST_F(PointCloudProcessorTest, EmptyCloudProcessingTest) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    bool result = processor_->processPointCloud(empty_cloud, output_cloud);
    
    // Processing should handle empty clouds gracefully
    EXPECT_TRUE(result || output_cloud->empty()) << "Empty cloud processing should handle gracefully";
}

/**
 * @brief Test ROS message processing interface
 */
TEST_F(PointCloudProcessorTest, ROSMessageProcessingTest) {
    // Convert PCL cloud to ROS message
    sensor_msgs::PointCloud2 input_msg;
    pcl::toROSMsg(*test_cloud_, input_msg);
    input_msg.header.frame_id = "hesai_lidar";
    input_msg.header.stamp = ros::Time::now();
    
    sensor_msgs::PointCloud2 output_msg;
    sensor_msgs::PointCloud2::ConstPtr input_ptr = 
        boost::make_shared<sensor_msgs::PointCloud2>(input_msg);
    
    bool result = processor_->processPointCloud(input_ptr, output_msg);
    
    EXPECT_TRUE(result) << "ROS message processing should succeed";
    EXPECT_EQ(output_msg.header.frame_id, input_msg.header.frame_id) << "Frame ID should be preserved";
    EXPECT_FALSE(output_msg.data.empty()) << "Output message should contain data";
}

/**
 * @brief Test point cloud size reduction through filtering
 */
TEST_F(PointCloudProcessorTest, FilteringEffectivenessTest) {
    // Count original points
    size_t original_size = test_cloud_->size();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    bool result = processor_->processPointCloud(test_cloud_, output_cloud);
    
    EXPECT_TRUE(result);
    
    // Filtering should reduce point count (due to voxel grid, outlier removal, etc.)
    EXPECT_LT(output_cloud->size(), original_size) << "Filtering should reduce point count";
    
    // But should retain some reasonable portion of points
    EXPECT_GT(output_cloud->size(), original_size / 10) << "Should retain reasonable number of points";
}

/**
 * @brief Test that intensity values are preserved
 */
TEST_F(PointCloudProcessorTest, IntensityPreservationTest) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    bool result = processor_->processPointCloud(test_cloud_, output_cloud);
    EXPECT_TRUE(result);
    
    if (!output_cloud->empty()) {
        // Check that intensity values are reasonable (not all zero)
        bool has_valid_intensity = false;
        for (const auto& point : output_cloud->points) {
            if (point.intensity > 0) {
                has_valid_intensity = true;
                break;
            }
        }
        EXPECT_TRUE(has_valid_intensity) << "Processed cloud should preserve intensity information";
    }
}

/**
 * @brief Test processing performance
 */
TEST_F(PointCloudProcessorTest, ProcessingPerformanceTest) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    bool result = processor_->processPointCloud(test_cloud_, output_cloud);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    EXPECT_TRUE(result);
    
    // Processing should complete within reasonable time (adjust threshold as needed)
    EXPECT_LT(duration.count(), 1000) << "Processing should complete within 1 second for test data";
    
    std::cout << "Processing time: " << duration.count() << " ms for " 
              << test_cloud_->size() << " input points" << std::endl;
}

} // namespace hesai_lidar_driver

/**
 * @brief Main function for running point cloud processor tests
 */
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_point_cloud_processor");
    
    // Run tests
    int result = RUN_ALL_TESTS();
    
    return result;
}
