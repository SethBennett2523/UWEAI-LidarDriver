#ifndef HESAI_DRIVER_H
#define HESAI_DRIVER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <memory>
#include <vector>

namespace hesai_lidar_driver {

/**
 * @brief Main driver class for Hesai Pandar 40p LiDAR sensor
 * 
 * Handles UDP communication, packet reception, and data parsing
 * for the Hesai Pandar 40p LiDAR connected via USB ethernet adaptor.
 */
class HesaiDriver {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     * @param private_nh Private ROS node handle for parameters
     */
    explicit HesaiDriver(std::shared_ptr<rclcpp::Node> node);
    
    /**
     * @brief Destructor
     */
    ~HesaiDriver();
    
    /**
     * @brief Initialize the driver
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Start data acquisition
     * @return true if successful, false otherwise
     */
    bool start();
    
    /**
     * @brief Stop data acquisition
     */
    void stop();
    
    /**
     * @brief Check if driver is running
     * @return true if running, false otherwise
     */
    bool isRunning() const { return running_; }

private:
    // ROS2 interface
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;
    
    // Parameters
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
    std::shared_ptr<std::thread> io_thread_;
    std::atomic<bool> running_;
    
    // Data buffers
    std::vector<uint8_t> data_buffer_;
    std::vector<uint8_t> position_buffer_;
    
    // Packet parser
    std::shared_ptr<class PacketParser> packet_parser_;
    
    /**
     * @brief Load parameters from ROS parameter server
     */
    void loadParameters();
    
    /**
     * @brief Setup UDP sockets for data and position packets
     * @return true if successful, false otherwise
     */
    bool setupSockets();
    
    /**
     * @brief Start asynchronous packet reception
     */
    void startReceive();
    
    /**
     * @brief Handle received data packets
     * @param error Boost error code
     * @param bytes_transferred Number of bytes received
     */
    void handleDataReceive(const boost::system::error_code& error, std::size_t bytes_transferred);
    
    /**
     * @brief Handle received position packets
     * @param error Boost error code
     * @param bytes_transferred Number of bytes received
     */
    void handlePositionReceive(const boost::system::error_code& error, std::size_t bytes_transferred);
    
    /**
     * @brief IO service run loop
     */
    void runIOService();
};

} // namespace hesai_lidar_driver

#endif // HESAI_DRIVER_H
