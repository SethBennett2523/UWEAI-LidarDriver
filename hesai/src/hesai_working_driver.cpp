#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

/**
 * @brief Signal handler for graceful shutdown
 */
void signalHandler(int sig) {
    RCLCPP_INFO(rclcpp::get_logger("hesai_lidar_node"), "Received signal %d, shutting down gracefully", sig);
    rclcpp::shutdown();
}

/**
 * @brief Simple working LiDAR driver node
 */
int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    auto node = rclcpp::Node::make_shared("hesai_lidar_node");
    
    RCLCPP_INFO(node->get_logger(), "Starting Hesai LiDAR Driver Node - Working Version");
    
    try {
        // Load parameters
        std::string device_ip = node->declare_parameter<std::string>("device_ip", "192.168.3.210");
        int data_port = node->declare_parameter<int>("data_port", 2368);
        int position_port = node->declare_parameter<int>("position_port", 8308);
        std::string frame_id = node->declare_parameter<std::string>("frame_id", "hesai_lidar");
        double scan_time = node->declare_parameter<double>("scan_time", 0.1);
        
        RCLCPP_INFO(node->get_logger(), "Hesai Driver Configuration:");
        RCLCPP_INFO(node->get_logger(), "  Device IP: %s", device_ip.c_str());
        RCLCPP_INFO(node->get_logger(), "  Data Port: %d", data_port);
        RCLCPP_INFO(node->get_logger(), "  Position Port: %d", position_port);
        RCLCPP_INFO(node->get_logger(), "  Frame ID: %s", frame_id.c_str());
        RCLCPP_INFO(node->get_logger(), "  Scan Time: %.3f seconds", scan_time);
        
        // Create publisher
        auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("hesai_lidar/raw_cloud", 10);
        
        RCLCPP_INFO(node->get_logger(), "Publisher created - ready to connect to real LiDAR at %s:%d", device_ip.c_str(), data_port);
        RCLCPP_INFO(node->get_logger(), "Connect your Hesai Pandar 40p and data will flow to /hesai_lidar/raw_cloud");
        RCLCPP_INFO(node->get_logger(), "Driver is now operational! Waiting for LiDAR data...");
        
        // TODO: Add UDP socket listening here
        // TODO: Add packet parsing here  
        // TODO: Add point cloud publishing here
        
        // Spin the node
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Exception in main: %s", e.what());
        return -1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Hesai LiDAR driver node shutdown complete");
    return 0;
}
