#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include "hesai_lidar_driver/hesai_driver.h"

/**
 * @brief Signal handler for graceful shutdown
 */
void signalHandler(int sig) {
    RCLCPP_INFO(rclcpp::get_logger("hesai_lidar_node"), "Received signal %d, shutting down gracefully", sig);
    rclcpp::shutdown();
}

/**
 * @brief Main function for Hesai LiDAR driver node
 * 
 * Initialises the complete pipeline from raw UDP packet reception
 * through to cone detection and publishing.
 */
int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    auto node = rclcpp::Node::make_shared("hesai_lidar_node");
    
    RCLCPP_INFO(node->get_logger(), "Starting Hesai LiDAR Driver Node");
    
    try {
        // Create driver components
        auto driver = std::make_shared<hesai_lidar_driver::HesaiDriver>(node);
        
        // Initialize components
        if (!driver->initialize()) {
            RCLCPP_ERROR(node->get_logger(), "Failed to initialise LiDAR driver");
            return -1;
        }
        
        // Start the driver
        if (!driver->start()) {
            RCLCPP_ERROR(node->get_logger(), "Failed to start LiDAR driver");
            return -1;
        }
        
        RCLCPP_INFO(node->get_logger(), "Hesai LiDAR driver started successfully");
        
        // Spin the node
        rclcpp::spin(node);
        
        // Cleanup
        RCLCPP_INFO(node->get_logger(), "Stopping Hesai LiDAR driver...");
        driver->stop();
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Exception in main: %s", e.what());
        return -1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Hesai LiDAR driver node shutdown complete");
    return 0;
}
