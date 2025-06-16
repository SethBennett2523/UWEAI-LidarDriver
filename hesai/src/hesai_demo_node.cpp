#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include "hesai_lidar_driver/hesai_driver.h"

/**
 * @brief Signal handler for graceful shutdown
 */
void signalHandler(int sig) {
    RCLCPP_INFO(rclcpp::get_logger("hesai_demo_node"), "Received signal %d, shutting down gracefully", sig);
    rclcpp::shutdown();
}

/**
 * @brief Main function for Hesai LiDAR demonstration node
 * 
 * ⚠️  WARNING: DEMONSTRATION MODE ONLY ⚠️
 * 
 * This version is configured for mapping demonstrations with extended
 * point cloud persistence. It does NOT represent real-time operation
 * and should NOT be used for time-critical applications.
 * 
 * For real-time applications, use the standard hesai_lidar_node.
 */
int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    auto node = rclcpp::Node::make_shared("hesai_demo_node");
    
    // Display prominent warning messages
    RCLCPP_WARN(node->get_logger(), "");
    RCLCPP_WARN(node->get_logger(), "=====================================");
    RCLCPP_WARN(node->get_logger(), "  HESAI LIDAR DEMONSTRATION MODE");
    RCLCPP_WARN(node->get_logger(), "=====================================");
    RCLCPP_WARN(node->get_logger(), "");
    RCLCPP_WARN(node->get_logger(), "⚠️  WARNING: This is a DEMO version");
    RCLCPP_WARN(node->get_logger(), "⚠️  Point persistence optimised for mapping");
    RCLCPP_WARN(node->get_logger(), "⚠️  Shows complete 360° LiDAR coverage");
    RCLCPP_WARN(node->get_logger(), "⚠️  For production: use hesai_lidar_node");
    RCLCPP_WARN(node->get_logger(), "");
    RCLCPP_WARN(node->get_logger(), "This demo shows complete 360° mapping");
    RCLCPP_WARN(node->get_logger(), "with optimised point persistence timing.");
    RCLCPP_WARN(node->get_logger(), "=====================================");
    RCLCPP_WARN(node->get_logger(), "");
    
    RCLCPP_INFO(node->get_logger(), "Starting Hesai LiDAR Driver Node - DEMONSTRATION VERSION");
    
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
        
        RCLCPP_INFO(node->get_logger(), "Hesai LiDAR driver started successfully - DEMO MODE");
        RCLCPP_INFO(node->get_logger(), "Publishing point clouds to /hesai_lidar/raw_cloud");
        RCLCPP_INFO(node->get_logger(), "Use RViz with hesai_mapping_demo.rviz for visualization");
        
        // Periodic reminder that this is demo mode
        auto reminder_timer = node->create_wall_timer(
            std::chrono::seconds(30),
            [node]() {
                RCLCPP_WARN(node->get_logger(), 
                           "REMINDER: This is DEMONSTRATION mode with optimised persistence - shows complete 360° coverage!");
            }
        );
        
        // Spin the node
        rclcpp::spin(node);
        
        // Cleanup
        RCLCPP_INFO(node->get_logger(), "Stopping Hesai LiDAR driver...");
        driver->stop();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
        return -1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Hesai LiDAR demonstration node shutdown complete");
    return 0;
}
