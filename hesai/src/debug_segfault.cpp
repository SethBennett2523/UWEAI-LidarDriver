#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "hesai_lidar_driver/packet_parser.h"

// Test actual HesaiDriver creation without the full class
class ExactHesaiDriverTest {
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::atomic<bool> running_;
    std::vector<uint8_t> data_buffer_;
    std::vector<uint8_t> position_buffer_;
    std::shared_ptr<hesai_lidar_driver::PacketParser> packet_parser_;

public:
    explicit ExactHesaiDriverTest(std::shared_ptr<rclcpp::Node> node)
        : node_(node)
        , running_(false)
        , data_buffer_(1500)  // Standard ethernet MTU
        , position_buffer_(1500)
    {
        RCLCPP_INFO(node_->get_logger(), "ExactHesaiDriverTest constructor: Creating packet parser...");
        
        // Create packet parser FIRST (like real HesaiDriver)
        packet_parser_ = std::make_shared<hesai_lidar_driver::PacketParser>();
        
        RCLCPP_INFO(node_->get_logger(), "ExactHesaiDriverTest constructor: Packet parser created, loading parameters...");
        
        loadParameters();
        
        RCLCPP_INFO(node_->get_logger(), "ExactHesaiDriverTest constructor: Parameters loaded successfully");
    }
    
private:
    void loadParameters() {
        // Get parameters
        std::string device_ip = node_->declare_parameter<std::string>("device_ip", "192.168.3.210");
        int data_port = node_->declare_parameter<int>("data_port", 2368);
        int position_port = node_->declare_parameter<int>("position_port", 8308);
        std::string frame_id = node_->declare_parameter<std::string>("frame_id", "hesai_lidar");
        double scan_time = node_->declare_parameter<double>("scan_time", 0.1);
        std::string calibration_file = node_->declare_parameter<std::string>("calibration_file", "");
        
        RCLCPP_INFO(node_->get_logger(), "ExactHesaiDriverTest Configuration:");
        RCLCPP_INFO(node_->get_logger(), "  Device IP: %s", device_ip.c_str());
        RCLCPP_INFO(node_->get_logger(), "  Data Port: %d", data_port);
        RCLCPP_INFO(node_->get_logger(), "  Position Port: %d", position_port);
        RCLCPP_INFO(node_->get_logger(), "  Frame ID: %s", frame_id.c_str());
        RCLCPP_INFO(node_->get_logger(), "  Scan Time: %.3f seconds", scan_time);
        
        // THIS is where the crash might occur - initializing packet parser
        RCLCPP_INFO(node_->get_logger(), "ExactHesaiDriverTest: About to initialize packet parser...");
        
        if (!packet_parser_->initialize(calibration_file)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize packet parser");
            return;
        }
        
        RCLCPP_INFO(node_->get_logger(), "ExactHesaiDriverTest: Packet parser initialized successfully");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("debug_test_node");
    RCLCPP_INFO(node->get_logger(), "=== TESTING EXACT HESAI DRIVER PATTERN ===");
    
    try {
        // Test: Create ExactHesaiDriverTest 
        RCLCPP_INFO(node->get_logger(), "Creating ExactHesaiDriverTest...");
        auto exact_driver = std::make_shared<ExactHesaiDriverTest>(node);
        RCLCPP_INFO(node->get_logger(), "SUCCESS: ExactHesaiDriverTest created without crash");
        
        RCLCPP_INFO(node->get_logger(), "=== ALL TESTS PASSED ===");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception during testing: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
