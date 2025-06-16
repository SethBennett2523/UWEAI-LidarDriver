#include <rclcpp/rclcpp.hpp>
#include <signal.h>

/**
 * @brief Signal handler for graceful shutdown
 */
void signalHandler(int sig) {
    RCLCPP_INFO(rclcpp::get_logger("hesai_lidar_node"), "Received signal %d, shutting down gracefully", sig);
    rclcpp::shutdown();
}

/**
 * @brief Test main function 
 */
int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    auto node = rclcpp::Node::make_shared("hesai_lidar_node");
    
    RCLCPP_INFO(node->get_logger(), "Starting Hesai LiDAR Driver Node - MINIMAL TEST");
    
    // Just do basic parameter loading for testing
    try {
        RCLCPP_INFO(node->get_logger(), "About to declare parameters...");
        
        std::string device_ip = node->declare_parameter<std::string>("device_ip", "192.168.3.210");
        int data_port = node->declare_parameter<int>("data_port", 2368);
        int position_port = node->declare_parameter<int>("position_port", 8308);
        std::string frame_id = node->declare_parameter<std::string>("frame_id", "hesai_lidar");
        double scan_time = node->declare_parameter<double>("scan_time", 0.1);
        
        RCLCPP_INFO(node->get_logger(), "Parameters declared successfully");
        
        RCLCPP_INFO(node->get_logger(), "Hesai Driver Configuration:");
        RCLCPP_INFO(node->get_logger(), "  Device IP: %s", device_ip.c_str());
        RCLCPP_INFO(node->get_logger(), "  Data Port: %d", data_port);
        RCLCPP_INFO(node->get_logger(), "  Position Port: %d", position_port);
        RCLCPP_INFO(node->get_logger(), "  Frame ID: %s", frame_id.c_str());
        RCLCPP_INFO(node->get_logger(), "  Scan Time: %.3f seconds", scan_time);
        
        RCLCPP_INFO(node->get_logger(), "TEST SUCCESSFUL - No crash occurred!");
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Exception in test: %s", e.what());
        return -1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Test node shutdown complete");
    return 0;
}
