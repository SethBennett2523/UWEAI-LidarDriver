#ifndef PACKET_PARSER_H
#define PACKET_PARSER_H

#include <vector>
#include <cstdint>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace hesai_lidar_driver {

/**
 * @brief Point structure for Hesai LiDAR data
 */
struct HesaiPoint {
    float x, y, z;          ///< Cartesian coordinates (metres)
    float intensity;        ///< Reflection intensity
    double timestamp;       ///< Point timestamp
    uint16_t ring;          ///< Laser ring number (0-39 for Pandar 40p)
    float azimuth;          ///< Azimuth angle (degrees)
    float distance;         ///< Distance from sensor (metres)
    
    HesaiPoint() : x(0), y(0), z(0), intensity(0), timestamp(0), ring(0), azimuth(0), distance(0) {}
};

/**
 * @brief Packet parser for Hesai Pandar 40p UDP data packets
 * 
 * Parses raw UDP packets from the Hesai Pandar 40p and converts them
 * into structured point cloud data with calibration applied.
 */
class PacketParser {
public:
    /**
     * @brief Constructor
     */
    PacketParser();
    
    /**
     * @brief Destructor
     */
    ~PacketParser();
    
    /**
     * @brief Initialize parser with calibration data
     * @param calibration_file Path to calibration file
     * @return true if successful, false otherwise
     */
    bool initialize(const std::string& calibration_file = "");
    
    /**
     * @brief Parse a data packet into points
     * @param packet Raw packet data
     * @param packet_size Size of packet in bytes
     * @param points Output vector of parsed points
     * @return true if packet parsed successfully, false otherwise
     */
    bool parseDataPacket(const uint8_t* packet, std::size_t packet_size, std::vector<HesaiPoint>& points);
    
    /**
     * @brief Parse a position packet (GPS/IMU data)
     * @param packet Raw packet data
     * @param packet_size Size of packet in bytes
     * @return true if packet parsed successfully, false otherwise
     */
    bool parsePositionPacket(const uint8_t* packet, std::size_t packet_size);
    
    /**
     * @brief Convert parsed points to ROS PointCloud2 message
     * @param points Vector of parsed points
     * @param cloud Output ROS point cloud message
     * @param frame_id Frame ID for the point cloud
     */
    void pointsToROS(const std::vector<HesaiPoint>& points, sensor_msgs::msg::PointCloud2& cloud, const std::string& frame_id);
    
    /**
     * @brief Convert parsed points to PCL point cloud
     * @param points Vector of parsed points
     * @param cloud Output PCL point cloud
     */
    void pointsToPCL(const std::vector<HesaiPoint>& points, pcl::PointCloud<pcl::PointXYZI>& cloud);

private:
    // Calibration data
    struct CalibrationData {
        float elevation_angles[40];     ///< Elevation angles for each laser (degrees)
        float azimuth_offsets[40];      ///< Azimuth offsets for each laser (degrees)
        float distance_corrections[40]; ///< Distance corrections for each laser (metres)
        
        // Constructor to initialize arrays
        CalibrationData() {
            for (int i = 0; i < 40; ++i) {
                elevation_angles[i] = 0.0f;
                azimuth_offsets[i] = 0.0f;
                distance_corrections[i] = 0.0f;
            }
        }
    };
    
    CalibrationData calibration_;
    bool calibration_loaded_;
    
    // Packet structure constants
    static const std::size_t PACKET_SIZE = 1262;
    static const std::size_t HEADER_SIZE = 42;
    static const std::size_t BLOCK_SIZE = 130;
    static const std::size_t BLOCKS_PER_PACKET = 10;
    static const std::size_t CHANNELS_PER_BLOCK = 40;
    
    /**
     * @brief Load calibration data from file
     * @param calibration_file Path to calibration file
     * @return true if successful, false otherwise
     */
    bool loadCalibration(const std::string& calibration_file);
    
    /**
     * @brief Load default calibration data for Pandar 40p
     */
    void loadDefaultCalibration();
    
    /**
     * @brief Convert polar coordinates to Cartesian
     * @param distance Distance in metres
     * @param azimuth Azimuth angle in degrees
     * @param elevation Elevation angle in degrees
     * @param x Output X coordinate
     * @param y Output Y coordinate
     * @param z Output Z coordinate
     */
    void polarToCartesian(float distance, float azimuth, float elevation, float& x, float& y, float& z);
    
    /**
     * @brief Validate packet header and structure
     * @param packet Raw packet data
     * @param packet_size Size of packet in bytes
     * @return true if valid, false otherwise
     */
    bool validatePacket(const uint8_t* packet, std::size_t packet_size);
};

} // namespace hesai_lidar_driver

#endif // PACKET_PARSER_H
