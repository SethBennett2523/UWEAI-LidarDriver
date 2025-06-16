#ifndef HESAI_LIDAR_DRIVER_HESAI_DRIVER_H
#define HESAI_LIDAR_DRIVER_HESAI_DRIVER_H

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

namespace hesai_lidar_driver {

/**
 * @brief Main driver class for Hesai Pandar 40p LiDAR sensor
 * 
 * Handles UDP communication, packet reception, and real-time
 * point cloud publishing for the Hesai Pandar 40p LiDAR sensor.
 * 
 * Features:
 * - Multi-threaded UDP socket handling
 * - Asynchronous packet processing
 * - Real-time point cloud generation
 * - Configurable network parameters
 * - Robust error handling and recovery
 */
class HesaiDriver {
public:
    /**
     * @brief Construct a new Hesai Driver object
     * 
     * @param node ROS2 node handle for parameter access and publishing
     */
    explicit HesaiDriver(std::shared_ptr<rclcpp::Node> node);
    
    /**
     * @brief Destructor - ensures clean shutdown
     */
    ~HesaiDriver();
    
    /**
     * @brief Initialize the driver and setup network connections
     * 
     * @return true if initialization successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Start the driver and begin packet reception
     * 
     * @return true if started successfully, false otherwise
     */
    bool start();
    
    /**
     * @brief Stop the driver and cleanup resources
     */
    void stop();
    
    /**
     * @brief Check if the driver is currently running
     * 
     * @return true if running, false otherwise
     */
    bool isRunning() const { return running_; }

private:
    /**
     * @brief Load configuration parameters from ROS parameter server
     */
    void loadParameters();
    
    /**
     * @brief Setup UDP sockets for data and position packets
     * 
     * @return true if setup successful, false otherwise
     */
    bool setupSockets();
    
    /**
     * @brief Start asynchronous packet reception
     */
    void startReceive();
    
    /**
     * @brief Handle received data packets
     * 
     * @param error Boost error code
     * @param bytes_transferred Number of bytes received
     */
    void handleDataReceive(const boost::system::error_code& error, std::size_t bytes_transferred);
    
    /**
     * @brief Handle received position packets
     * 
     * @param error Boost error code
     * @param bytes_transferred Number of bytes received
     */
    void handlePositionReceive(const boost::system::error_code& error, std::size_t bytes_transferred);
    
    /**
     * @brief Main IO service loop running in separate thread
     */
    void runIOService();
    
    // ROS2 interface
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;
    
    // Configuration parameters
    std::string device_ip_;
    int data_port_;
    int position_port_;
    std::string frame_id_;
    double scan_time_;
    
    // Network components
    std::shared_ptr<boost::asio::io_service> io_service_;
    std::shared_ptr<boost::asio::ip::udp::socket> data_socket_;
    std::shared_ptr<boost::asio::ip::udp::socket> position_socket_;
    boost::asio::ip::udp::endpoint data_endpoint_;
    boost::asio::ip::udp::endpoint position_endpoint_;
    
    // Threading
    std::shared_ptr<boost::thread> io_thread_;
    bool running_;
    
    // Data buffers
    std::vector<uint8_t> data_buffer_;
    std::vector<uint8_t> position_buffer_;
    
    // Packet parser
    std::shared_ptr<class PacketParser> packet_parser_;
};

} // namespace hesai_lidar_driver

#endif // HESAI_LIDAR_DRIVER_HESAI_DRIVER_H
