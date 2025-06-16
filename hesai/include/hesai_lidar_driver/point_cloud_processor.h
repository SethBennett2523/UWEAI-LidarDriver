#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "hesai_lidar_driver/packet_parser.h"

namespace hesai_lidar_driver {

/**
 * @brief Point cloud processing pipeline for LiDAR data
 * 
 * Applies filtering, downsampling, and preprocessing to raw LiDAR
 * point clouds to prepare them for cone detection algorithms.
 */
class PointCloudProcessor {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     * @param private_nh Private ROS node handle for parameters
     */
    PointCloudProcessor(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    
    /**
     * @brief Destructor
     */
    ~PointCloudProcessor();
    
    /**
     * @brief Initialize the processor
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Process raw point cloud data
     * @param input_cloud Input point cloud
     * @param output_cloud Processed output point cloud
     * @return true if processing successful, false otherwise
     */
    bool processPointCloud(const sensor_msgs::PointCloud2::ConstPtr& input_cloud, 
                          sensor_msgs::PointCloud2& output_cloud);
    
    /**
     * @brief Process PCL point cloud data
     * @param input_cloud Input PCL point cloud
     * @param output_cloud Processed output PCL point cloud
     * @return true if processing successful, false otherwise
     */
    bool processPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud);

private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Publishers for debugging
    ros::Publisher filtered_cloud_pub_;
    ros::Publisher ground_removed_pub_;
    
    // Processing parameters
    struct ProcessingParams {
        // Passthrough filter bounds
        float min_x, max_x;
        float min_y, max_y;
        float min_z, max_z;
        
        // Voxel grid downsampling
        float voxel_leaf_size;
        
        // Statistical outlier removal
        int sor_mean_k;
        float sor_std_dev_threshold;
        
        // Ground removal
        bool enable_ground_removal;
        float ground_height_threshold;
        
        // Ring-based filtering (for structured LiDAR)
        bool enable_ring_filter;
        int min_ring, max_ring;
        
        // Intensity filtering
        bool enable_intensity_filter;
        float min_intensity;
    };
    
    ProcessingParams params_;
    
    // PCL filter objects
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_;
    pcl::PassThrough<pcl::PointXYZI> pass_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter_;
    
    /**
     * @brief Load processing parameters from ROS parameter server
     */
    void loadParameters();
    
    /**
     * @brief Apply passthrough filtering to remove points outside ROI
     * @param cloud Input/output point cloud
     * @return true if successful, false otherwise
     */
    bool applyPassthroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    /**
     * @brief Apply voxel grid downsampling
     * @param cloud Input/output point cloud
     * @return true if successful, false otherwise
     */
    bool applyVoxelGridFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    /**
     * @brief Apply statistical outlier removal
     * @param cloud Input/output point cloud
     * @return true if successful, false otherwise
     */
    bool applyStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    /**
     * @brief Remove ground points using simple height threshold
     * @param cloud Input/output point cloud
     * @return true if successful, false otherwise
     */
    bool removeGroundPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    /**
     * @brief Apply ring-based filtering (for structured LiDAR data)
     * @param cloud Input/output point cloud
     * @return true if successful, false otherwise
     */
    bool applyRingFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    /**
     * @brief Apply intensity-based filtering
     * @param cloud Input/output point cloud
     * @return true if successful, false otherwise
     */
    bool applyIntensityFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
};

} // namespace hesai_lidar_driver

#endif // POINT_CLOUD_PROCESSOR_H
