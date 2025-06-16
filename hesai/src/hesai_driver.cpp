#include "hesai_lidar_driver/hesai_driver.h"
#include "hesai_lidar_driver/packet_parser.h"
#include <rclcpp/rclcpp.hpp>
#include <functional>

namespace hesai_lidar_driver {

HesaiDriver::HesaiDriver(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
    , running_(false)
    , data_buffer_(1500)  // Standard ethernet MTU
    , position_buffer_(1500)
{
    RCLCPP_INFO(node_->get_logger(), "HesaiDriver constructor: Creating packet parser...");
    
    // Create packet parser
    packet_parser_ = std::make_shared<PacketParser>();
    
    RCLCPP_INFO(node_->get_logger(), "HesaiDriver constructor: Packet parser created, loading parameters...");
    
    loadParameters();
    
    RCLCPP_INFO(node_->get_logger(), "HesaiDriver constructor: Parameters loaded successfully");
}

HesaiDriver::~HesaiDriver() {
    stop();
}

void HesaiDriver::loadParameters() {
    RCLCPP_INFO(node_->get_logger(), "Loading parameters...");
    
    // Network configuration
    device_ip_ = node_->declare_parameter<std::string>("device_ip", "192.168.3.210");
    data_port_ = node_->declare_parameter<int>("data_port", 2368);
    position_port_ = node_->declare_parameter<int>("position_port", 8308);
    
    // Frame configuration
    frame_id_ = node_->declare_parameter<std::string>("frame_id", "hesai_lidar");
    scan_time_ = node_->declare_parameter<double>("scan_time", 0.1);
    
    // Calibration file
    std::string calibration_file = node_->declare_parameter<std::string>("calibration_file", "");
    
    RCLCPP_INFO(node_->get_logger(), "Hesai Driver Configuration:");
    RCLCPP_INFO(node_->get_logger(), "  Device IP: %s", device_ip_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  Data Port: %d", data_port_);
    RCLCPP_INFO(node_->get_logger(), "  Position Port: %d", position_port_);
    RCLCPP_INFO(node_->get_logger(), "  Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(node_->get_logger(), "  Scan Time: %.3f seconds", scan_time_);
    
    RCLCPP_INFO(node_->get_logger(), "About to initialize packet parser...");
    
    // Initialize packet parser
    if (!packet_parser_->initialize(calibration_file)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to initialize packet parser with calibration file");
    }
    
    RCLCPP_INFO(node_->get_logger(), "Packet parser initialized successfully");
}

bool HesaiDriver::initialize() {
    try {
        // Create IO service
        io_service_ = std::make_shared<boost::asio::io_service>();
        
        // Setup network sockets
        if (!setupSockets()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to setup UDP sockets");
            return false;
        }
        
        // Create publishers
        raw_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("hesai_lidar/raw_cloud", 10);
        
        RCLCPP_INFO(node_->get_logger(), "Hesai driver initialised successfully");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception during driver initialisation: %s", e.what());
        return false;
    }
}

bool HesaiDriver::setupSockets() {
    try {
        // Create data socket
        data_socket_ = std::make_shared<boost::asio::ip::udp::socket>(*io_service_);
        data_socket_->open(boost::asio::ip::udp::v4());
        
        // Bind to data port
        boost::asio::ip::udp::endpoint data_endpoint(boost::asio::ip::udp::v4(), data_port_);
        data_socket_->bind(data_endpoint);
        
        // Set socket options for better performance
        boost::asio::socket_base::receive_buffer_size data_buffer_option(8192);
        data_socket_->set_option(data_buffer_option);
        
        RCLCPP_INFO(node_->get_logger(), "Data socket bound to port %d", data_port_);
        
        // Create position socket
        position_socket_ = std::make_shared<boost::asio::ip::udp::socket>(*io_service_);
        position_socket_->open(boost::asio::ip::udp::v4());
        
        // Bind to position port
        boost::asio::ip::udp::endpoint position_endpoint(boost::asio::ip::udp::v4(), position_port_);
        position_socket_->bind(position_endpoint);
        
        RCLCPP_INFO(node_->get_logger(), "Position socket bound to port %d", position_port_);
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception setting up sockets: %s", e.what());
        return false;
    }
}

bool HesaiDriver::start() {
    if (running_) {
        RCLCPP_WARN(node_->get_logger(), "Driver already running");
        return true;
    }
    
    try {
        // Start asynchronous packet reception
        startReceive();
        
        // Start IO service thread
        running_ = true;
        io_thread_ = std::make_shared<std::thread>(
            [this]() { this->runIOService(); });
        
        RCLCPP_INFO(node_->get_logger(), "Hesai driver started successfully");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception starting driver: %s", e.what());
        running_ = false;
        return false;
    }
}

void HesaiDriver::stop() {
    if (!running_) {
        return;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Stopping Hesai driver...");
    
    running_ = false;
    
    // Stop IO service
    if (io_service_) {
        io_service_->stop();
    }
    
    // Join IO thread
    if (io_thread_ && io_thread_->joinable()) {
        io_thread_->join();
    }
    
    // Close sockets
    if (data_socket_ && data_socket_->is_open()) {
        data_socket_->close();
    }
    
    if (position_socket_ && position_socket_->is_open()) {
        position_socket_->close();
    }
    
    RCLCPP_INFO(node_->get_logger(), "Hesai driver stopped");
}

void HesaiDriver::startReceive() {
    // Start async receive for data packets
    data_socket_->async_receive_from(
        boost::asio::buffer(data_buffer_),
        data_endpoint_,
        [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
            this->handleDataReceive(error, bytes_transferred);
        });
    
    // Start async receive for position packets
    position_socket_->async_receive_from(
        boost::asio::buffer(position_buffer_),
        position_endpoint_,
        [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
            this->handlePositionReceive(error, bytes_transferred);
        });
}

void HesaiDriver::handleDataReceive(const boost::system::error_code& error, 
                                   std::size_t bytes_transferred) {
    if (!running_) {
        return;
    }
    
    if (!error && bytes_transferred > 0) {
        // Process the received data packet
        try {
            std::vector<HesaiPoint> points;
            
            // Parse the packet using PacketParser
            if (packet_parser_->parseDataPacket(data_buffer_.data(), bytes_transferred, points)) {
                // Convert to ROS message and publish
                sensor_msgs::msg::PointCloud2 cloud_msg;
                packet_parser_->pointsToROS(points, cloud_msg, frame_id_);
                raw_cloud_pub_->publish(cloud_msg);
                
                static int packet_count = 0;
                packet_count++;
                
                if (packet_count % 100 == 0) {  // Log every 100th packet
                    RCLCPP_DEBUG(node_->get_logger(), "Published point cloud %d with %zu points from packet size %zu", 
                               packet_count, points.size(), bytes_transferred);
                }
            } else {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
                                    "Failed to parse data packet of size %zu", bytes_transferred);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception processing data packet: %s", e.what());
        }
    } else if (error != boost::asio::error::operation_aborted) {
        RCLCPP_ERROR(node_->get_logger(), "Data receive error: %s", error.message().c_str());
    }
    
    // Continue receiving if still running
    if (running_) {
        data_socket_->async_receive_from(
            boost::asio::buffer(data_buffer_),
            data_endpoint_,
            [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                this->handleDataReceive(error, bytes_transferred);
            });
    }
}

void HesaiDriver::handlePositionReceive(const boost::system::error_code& error,
                                       std::size_t bytes_transferred) {
    if (!running_) {
        return;
    }
    
    if (!error && bytes_transferred > 0) {
        // Process the received position packet
        try {
            // Parse position packet
            if (packet_parser_->parsePositionPacket(position_buffer_.data(), bytes_transferred)) {
                static int pos_packet_count = 0;
                pos_packet_count++;
                
                if (pos_packet_count % 10 == 0) {  // Log every 10th position packet
                    RCLCPP_DEBUG(node_->get_logger(), "Parsed position packet %d, size: %zu bytes", 
                               pos_packet_count, bytes_transferred);
                }
            } else {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, 
                                    "Failed to parse position packet of size %zu", bytes_transferred);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception processing position packet: %s", e.what());
        }
    } else if (error != boost::asio::error::operation_aborted) {
        RCLCPP_ERROR(node_->get_logger(), "Position receive error: %s", error.message().c_str());
    }
    
    // Continue receiving if still running
    if (running_) {
        position_socket_->async_receive_from(
            boost::asio::buffer(position_buffer_),
            position_endpoint_,
            [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                this->handlePositionReceive(error, bytes_transferred);
            });
    }
}

void HesaiDriver::runIOService() {
    RCLCPP_INFO(node_->get_logger(), "Starting IO service thread");
    
    try {
        while (running_ && rclcpp::ok()) {
            // Run IO service with timeout to allow periodic checks
            io_service_->reset();
            io_service_->run_for(std::chrono::milliseconds(100));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception in IO service thread: %s", e.what());
    }
    
    RCLCPP_INFO(node_->get_logger(), "IO service thread finished");
}

} // namespace hesai_lidar_driver
