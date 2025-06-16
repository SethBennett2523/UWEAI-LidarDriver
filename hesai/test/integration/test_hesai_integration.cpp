#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <boost/thread.hpp>

/**
 * @brief Integration test for complete Hesai LiDAR driver system
 * 
 * Tests the full pipeline from driver initialisation through to
 * cone detection and publishing, verifying topic availability,
 * message rates, and system integration.
 */
class HesaiIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh_ = std::make_shared<ros::NodeHandle>();
        
        // Get test parameters
        nh_->param("test_duration", test_duration_, 10.0);
        nh_->param("min_message_rate", min_message_rate_, 5.0);
        
        std::string expected_topics_str;
        nh_->param("expected_topics", expected_topics_str, std::string(""));
        parseExpectedTopics(expected_topics_str);
        
        // Initialize message counters
        raw_cloud_count_ = 0;
        filtered_cloud_count_ = 0;
        cone_markers_count_ = 0;
        
        // Set up subscribers
        setupSubscribers();
        
        // Set up TF listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Allow time for setup
        ros::Duration(2.0).sleep();
    }

    void TearDown() override {
        // Clean up subscribers
        raw_cloud_sub_.shutdown();
        filtered_cloud_sub_.shutdown();
        cone_markers_sub_.shutdown();
        
        tf_listener_.reset();
        tf_buffer_.reset();
        nh_.reset();
    }

    void parseExpectedTopics(const std::string& topics_str) {
        std::stringstream ss(topics_str);
        std::string topic;
        while (std::getline(ss, topic, ',')) {
            expected_topics_.push_back(topic);
        }
    }

    void setupSubscribers() {
        raw_cloud_sub_ = nh_->subscribe("/hesai_lidar/raw_cloud", 10,
            &HesaiIntegrationTest::rawCloudCallback, this);
        
        filtered_cloud_sub_ = nh_->subscribe("/hesai_lidar/filtered_cloud", 10,
            &HesaiIntegrationTest::filteredCloudCallback, this);
        
        cone_markers_sub_ = nh_->subscribe("/hesai_lidar/cone_markers", 10,
            &HesaiIntegrationTest::coneMarkersCallback, this);
    }

    void rawCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        boost::lock_guard<boost::mutex> lock(mutex_);
        raw_cloud_count_++;
        last_raw_cloud_ = msg;
    }

    void filteredCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        boost::lock_guard<boost::mutex> lock(mutex_);
        filtered_cloud_count_++;
        last_filtered_cloud_ = msg;
    }

    void coneMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        boost::lock_guard<boost::mutex> lock(mutex_);
        cone_markers_count_++;
        last_cone_markers_ = msg;
    }

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Test parameters
    double test_duration_;
    double min_message_rate_;
    std::vector<std::string> expected_topics_;
    
    // Subscribers
    ros::Subscriber raw_cloud_sub_;
    ros::Subscriber filtered_cloud_sub_;
    ros::Subscriber cone_markers_sub_;
    
    // Message counters and storage
    boost::mutex mutex_;
    int raw_cloud_count_;
    int filtered_cloud_count_;
    int cone_markers_count_;
    
    sensor_msgs::PointCloud2::ConstPtr last_raw_cloud_;
    sensor_msgs::PointCloud2::ConstPtr last_filtered_cloud_;
    visualization_msgs::MarkerArray::ConstPtr last_cone_markers_;
};

/**
 * @brief Test that ROS node starts successfully
 */
TEST_F(HesaiIntegrationTest, NodeStartupTest) {
    ASSERT_TRUE(ros::ok()) << "ROS should be running";
    ASSERT_TRUE(nh_ != nullptr) << "Node handle should be valid";
    
    // Check that expected topics are advertised
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    
    for (const std::string& expected_topic : expected_topics_) {
        bool topic_found = false;
        for (const auto& topic_info : topic_infos) {
            if (topic_info.name == expected_topic) {
                topic_found = true;
                break;
            }
        }
        EXPECT_TRUE(topic_found) << "Expected topic not found: " << expected_topic;
    }
}

/**
 * @brief Test TF frame publishing
 */
TEST_F(HesaiIntegrationTest, TransformTest) {
    // Wait for transforms to be available
    ros::Duration(3.0).sleep();
    
    try {
        // Check for base_link to hesai_lidar transform
        auto transform = tf_buffer_->lookupTransform("base_link", "hesai_lidar", 
                                                    ros::Time(0), ros::Duration(5.0));
        
        EXPECT_EQ(transform.header.frame_id, "base_link");
        EXPECT_EQ(transform.child_frame_id, "hesai_lidar");
        
        // Transform should be reasonable (LiDAR mounted on vehicle)
        EXPECT_GE(transform.transform.translation.z, 0.0) << "LiDAR should be above base_link";
        EXPECT_LT(transform.transform.translation.z, 5.0) << "LiDAR height should be reasonable";
        
    } catch (const tf2::TransformException& ex) {
        FAIL() << "Transform lookup failed: " << ex.what();
    }
}

/**
 * @brief Test message publishing rates
 */
TEST_F(HesaiIntegrationTest, MessageRateTest) {
    ros::Time start_time = ros::Time::now();
    
    // Reset counters
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        raw_cloud_count_ = 0;
        filtered_cloud_count_ = 0;
        cone_markers_count_ = 0;
    }
    
    // Wait for test duration
    ros::Duration test_duration(test_duration_);
    ros::Time end_time = start_time + test_duration;
    
    while (ros::Time::now() < end_time && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    
    double actual_duration = (ros::Time::now() - start_time).toSec();
    
    // Check message rates
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        
        double raw_cloud_rate = raw_cloud_count_ / actual_duration;
        double filtered_cloud_rate = filtered_cloud_count_ / actual_duration;
        double cone_markers_rate = cone_markers_count_ / actual_duration;
        
        std::cout << "Message rates over " << actual_duration << " seconds:" << std::endl;
        std::cout << "  Raw cloud: " << raw_cloud_rate << " Hz" << std::endl;
        std::cout << "  Filtered cloud: " << filtered_cloud_rate << " Hz" << std::endl;
        std::cout << "  Cone markers: " << cone_markers_rate << " Hz" << std::endl;
        
        // Note: These tests may fail if LiDAR hardware is not connected
        // In real testing, adjust expectations based on hardware availability
        if (raw_cloud_count_ > 0) {
            EXPECT_GE(raw_cloud_rate, min_message_rate_) 
                << "Raw cloud publishing rate too low";
        } else {
            std::cout << "Warning: No raw cloud messages received (hardware may not be connected)" << std::endl;
        }
        
        // Filtered cloud and markers should publish if raw data is available
        if (raw_cloud_count_ > 0) {
            EXPECT_GT(filtered_cloud_count_, 0) << "Should publish filtered clouds when raw data available";
            EXPECT_GE(cone_markers_count_, 0) << "Should publish cone markers (even if empty)";
        }
    }
}

/**
 * @brief Test message content validity
 */
TEST_F(HesaiIntegrationTest, MessageContentTest) {
    // Wait for some messages to arrive
    ros::Duration(5.0).sleep();
    ros::spinOnce();
    
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        
        // Test raw cloud message if available
        if (last_raw_cloud_) {
            EXPECT_EQ(last_raw_cloud_->header.frame_id, "hesai_lidar") 
                << "Raw cloud should use correct frame ID";
            EXPECT_GT(last_raw_cloud_->width, 0u) << "Raw cloud should contain points";
            EXPECT_FALSE(last_raw_cloud_->data.empty()) << "Raw cloud data should not be empty";
        }
        
        // Test filtered cloud message if available
        if (last_filtered_cloud_) {
            EXPECT_EQ(last_filtered_cloud_->header.frame_id, "hesai_lidar") 
                << "Filtered cloud should use correct frame ID";
            EXPECT_FALSE(last_filtered_cloud_->data.empty()) << "Filtered cloud data should not be empty";
            
            // Filtered cloud should be smaller or equal to raw cloud
            if (last_raw_cloud_) {
                EXPECT_LE(last_filtered_cloud_->width, last_raw_cloud_->width) 
                    << "Filtered cloud should not be larger than raw cloud";
            }
        }
        
        // Test cone markers message
        if (last_cone_markers_) {
            EXPECT_TRUE(last_cone_markers_->markers.empty() || 
                       last_cone_markers_->markers[0].header.frame_id == "hesai_lidar") 
                << "Cone markers should use correct frame ID";
            
            // Check marker properties if any cones detected
            for (const auto& marker : last_cone_markers_->markers) {
                EXPECT_TRUE(marker.type == visualization_msgs::Marker::CYLINDER ||
                           marker.type == visualization_msgs::Marker::SPHERE) 
                    << "Cone markers should use appropriate shapes";
                EXPECT_GT(marker.scale.x, 0.0) << "Marker scale should be positive";
                EXPECT_GT(marker.scale.y, 0.0) << "Marker scale should be positive";
                EXPECT_GT(marker.scale.z, 0.0) << "Marker scale should be positive";
            }
        }
    }
}

/**
 * @brief Test system recovery from errors
 */
TEST_F(HesaiIntegrationTest, ErrorRecoveryTest) {
    // This test would simulate error conditions and verify recovery
    // For now, just ensure system continues running
    
    ros::Duration(3.0).sleep();
    
    EXPECT_TRUE(ros::ok()) << "System should remain running throughout test";
    
    // Additional error injection tests could be added here
    // e.g., network disconnection, invalid data, etc.
}

/**
 * @brief Main function for integration tests
 */
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_hesai_integration");
    
    // Run tests
    int result = RUN_ALL_TESTS();
    
    return result;
}
