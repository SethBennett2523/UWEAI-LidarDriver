#include "hesai_lidar_driver/packet_parser.h"
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <fstream>
#include <algorithm>

namespace hesai_lidar_driver {

PacketParser::PacketParser() : calibration_loaded_(false) {
    // Initialize with default calibration
    loadDefaultCalibration();
}

PacketParser::~PacketParser() = default;

bool PacketParser::initialize(const std::string& calibration_file) {
    if (!calibration_file.empty()) {
        if (loadCalibration(calibration_file)) {
            RCLCPP_INFO(rclcpp::get_logger("packet_parser"), "Loaded calibration from file: %s", calibration_file.c_str());
            return true;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("packet_parser"), "Failed to load calibration file, using default calibration");
        }
    }
    
    loadDefaultCalibration();
    RCLCPP_INFO(rclcpp::get_logger("packet_parser"), "Using default Pandar 40p calibration");
    return true;
}

void PacketParser::loadDefaultCalibration() {
    // Default Pandar 40p elevation angles (degrees) - approximation
    // Actual values should come from calibration file
    const float default_elevations[40] = {
        15.0f, 13.0f, 11.0f, 9.0f, 7.0f, 5.0f, 3.0f, 1.0f,
        -1.0f, -3.0f, -5.0f, -7.0f, -9.0f, -11.0f, -13.0f, -15.0f,
        15.0f, 13.0f, 11.0f, 9.0f, 7.0f, 5.0f, 3.0f, 1.0f,
        -1.0f, -3.0f, -5.0f, -7.0f, -9.0f, -11.0f, -13.0f, -15.0f,
        15.0f, 13.0f, 11.0f, 9.0f, 7.0f, 5.0f, 3.0f, 1.0f
    };
    
    // Copy default values
    for (int i = 0; i < 40; ++i) {
        calibration_.elevation_angles[i] = default_elevations[i];
        calibration_.azimuth_offsets[i] = 0.0f;  // No offset by default
        calibration_.distance_corrections[i] = 0.0f;  // No correction by default
    }
    
    calibration_loaded_ = true;
}

bool PacketParser::loadCalibration(const std::string& calibration_file) {
    try {
        std::ifstream file(calibration_file);
        if (!file.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("packet_parser"), "Cannot open calibration file: %s", calibration_file.c_str());
            return false;
        }
        
        // Simple CSV format: ring,elevation,azimuth_offset,distance_correction
        std::string line;
        std::getline(file, line);  // Skip header
        
        int ring = 0;
        while (std::getline(file, line) && ring < 40) {
            std::stringstream ss(line);
            std::string token;
            
            // Parse CSV fields
            std::getline(ss, token, ',');  // Ring number
            std::getline(ss, token, ',');  // Elevation
            calibration_.elevation_angles[ring] = std::stof(token);
            
            std::getline(ss, token, ',');  // Azimuth offset
            calibration_.azimuth_offsets[ring] = std::stof(token);
            
            std::getline(ss, token, ',');  // Distance correction
            calibration_.distance_corrections[ring] = std::stof(token);
            
            ring++;
        }
        
        file.close();
        calibration_loaded_ = true;
        return ring == 40;  // Ensure all 40 rings were loaded
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("packet_parser"), "Exception loading calibration: %s", e.what());
        return false;
    }
}

bool PacketParser::validatePacket(const uint8_t* packet, std::size_t packet_size) {
    if (!packet || packet_size < HEADER_SIZE) {
        return false;
    }
    
    // Check packet size
    if (packet_size != PACKET_SIZE) {
        return false;
    }
    
    // Check header magic bytes (Hesai specific)
    if (packet[0] != 0xEE || packet[1] != 0xFF) {
        return false;
    }
    
    return true;
}

bool PacketParser::parseDataPacket(const uint8_t* packet, std::size_t packet_size, 
                                  std::vector<HesaiPoint>& points) {
    points.clear();
    
    if (!validatePacket(packet, packet_size)) {
        return false;
    }
    
    try {
        // Parse Hesai Pandar 40p packet structure
        // Packet format (simplified):
        // Header: 42 bytes
        // Data blocks: 10 blocks × 130 bytes each
        // Each block contains 40 channels × 3 bytes (distance + intensity)
        
        const uint8_t* data_ptr = packet + HEADER_SIZE;
        
        for (std::size_t block = 0; block < BLOCKS_PER_PACKET; ++block) {
            // Extract azimuth from block header (first 2 bytes of block)
            uint16_t azimuth_raw = (data_ptr[1] << 8) | data_ptr[0];
            float azimuth = azimuth_raw * 0.01f;  // Convert to degrees
            
            // Skip block header (2 bytes azimuth + other headers)
            const uint8_t* channel_data = data_ptr + 4;
            
            // Parse each channel in the block
            for (std::size_t channel = 0; channel < CHANNELS_PER_BLOCK; ++channel) {
                // Extract distance (2 bytes) and intensity (1 byte)
                uint16_t distance_raw = (channel_data[1] << 8) | channel_data[0];
                uint8_t intensity_raw = channel_data[2];
                
                // Convert raw values
                float distance = distance_raw * 0.004f;  // 4mm resolution
                float intensity = static_cast<float>(intensity_raw);
                
                // Apply calibration corrections
                distance += calibration_.distance_corrections[channel];
                float elevation = calibration_.elevation_angles[channel];
                float azimuth_corrected = azimuth + calibration_.azimuth_offsets[channel];
                
                // Filter invalid points
                if (distance < 0.1f || distance > 200.0f) {  // Valid range
                    channel_data += 3;
                    continue;
                }
                
                // Convert to Cartesian coordinates
                HesaiPoint point;
                polarToCartesian(distance, azimuth_corrected, elevation, 
                               point.x, point.y, point.z);
                
                point.intensity = intensity;
                point.distance = distance;
                point.azimuth = azimuth_corrected;
                point.ring = static_cast<uint16_t>(channel);
                point.timestamp = rclcpp::Clock().now().seconds();  // TODO: Extract from packet
                
                points.push_back(point);
                
                channel_data += 3;  // Move to next channel
            }
            
            data_ptr += BLOCK_SIZE;  // Move to next block
        }
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("packet_parser"), "Exception parsing data packet: %s", e.what());
        return false;
    }
}

bool PacketParser::parsePositionPacket(const uint8_t* packet, std::size_t packet_size) {
    // Position packet parsing for GPS/IMU data
    // Implementation depends on specific Hesai position packet format
    
    if (!packet || packet_size < 32) {  // Minimum expected size
        return false;
    }
    
    try {
        // TODO: Implement position packet parsing based on Hesai documentation
        // This would extract GPS coordinates, IMU data, timestamp, etc.
        
        RCLCPP_DEBUG(rclcpp::get_logger("packet_parser"), "Position packet parsed, size: %zu", packet_size);
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("packet_parser"), "Exception parsing position packet: %s", e.what());
        return false;
    }
}

void PacketParser::polarToCartesian(float distance, float azimuth, float elevation,
                                   float& x, float& y, float& z) {
    // Convert angles to radians
    float azimuth_rad = azimuth * M_PI / 180.0f;
    float elevation_rad = elevation * M_PI / 180.0f;
    
    // Convert spherical to Cartesian coordinates
    // ROS coordinate system: X forward, Y left, Z up
    float cos_elevation = cos(elevation_rad);
    
    x = distance * cos_elevation * cos(azimuth_rad);
    y = distance * cos_elevation * sin(azimuth_rad);
    z = distance * sin(elevation_rad);
}

void PacketParser::pointsToROS(const std::vector<HesaiPoint>& points, 
                              sensor_msgs::msg::PointCloud2& cloud, 
                              const std::string& frame_id) {
    // Create PCL point cloud
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pointsToPCL(points, pcl_cloud);
    
    // Convert to ROS message
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = rclcpp::Clock().now();
}

void PacketParser::pointsToPCL(const std::vector<HesaiPoint>& points,
                              pcl::PointCloud<pcl::PointXYZI>& cloud) {
    cloud.clear();
    cloud.reserve(points.size());
    
    for (const auto& hesai_point : points) {
        pcl::PointXYZI pcl_point;
        pcl_point.x = hesai_point.x;
        pcl_point.y = hesai_point.y;
        pcl_point.z = hesai_point.z;
        pcl_point.intensity = hesai_point.intensity;
        
        cloud.push_back(pcl_point);
    }
    
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = false;  // May contain invalid points
}

} // namespace hesai_lidar_driver
