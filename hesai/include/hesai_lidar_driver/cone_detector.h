#ifndef CONE_DETECTOR_H
#define CONE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <vector>

// Assuming bristol_msgs is available - adjust include path as needed
// #include <bristol_msgs/ConeArrayWithCovariance.h>

namespace hesai_lidar_driver {

/**
 * @brief Structure representing a detected cone
 */
struct DetectedCone {
    geometry_msgs::Point position;      ///< 3D position of cone centroid
    float confidence;                   ///< Detection confidence (0-1)
    std::string colour;                 ///< Cone colour ("unknown" for this implementation)
    float height;                       ///< Estimated cone height
    float radius;                       ///< Estimated cone base radius
    int point_count;                    ///< Number of points in cone cluster
    
    DetectedCone() : confidence(0.0f), colour("unknown"), height(0.0f), radius(0.0f), point_count(0) {}
};

/**
 * @brief Cone detection algorithm for LiDAR point clouds
 * 
 * Detects traffic cones in processed LiDAR point clouds using clustering
 * and geometric analysis. Publishes detected cones as ConeArrayWithCovariance
 * messages for autonomous vehicle navigation.
 */
class ConeDetector {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     * @param private_nh Private ROS node handle for parameters
     */
    ConeDetector(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    
    /**
     * @brief Destructor
     */
    ~ConeDetector();
    
    /**
     * @brief Initialize the cone detector
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Detect cones in a point cloud
     * @param input_cloud Input point cloud to process
     * @param detected_cones Output vector of detected cones
     * @return true if detection successful, false otherwise
     */
    bool detectCones(const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
                     std::vector<DetectedCone>& detected_cones);
    
    /**
     * @brief Detect cones in a PCL point cloud
     * @param input_cloud Input PCL point cloud to process
     * @param detected_cones Output vector of detected cones
     * @return true if detection successful, false otherwise
     */
    bool detectCones(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                     std::vector<DetectedCone>& detected_cones);

private:
    // ROS handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Publishers
    ros::Publisher cone_markers_pub_;
    ros::Publisher cone_clusters_pub_;
    // ros::Publisher cone_array_pub_;  // Uncomment when bristol_msgs available
    
    // Detection parameters
    struct DetectionParams {
        // Clustering parameters
        float cluster_tolerance;
        int min_cluster_size;
        int max_cluster_size;
        
        // Cone geometry constraints
        float min_cone_height;
        float max_cone_height;
        float min_cone_radius;
        float max_cone_radius;
        float max_cone_aspect_ratio;
        
        // Detection thresholds
        float min_detection_confidence;
        float max_detection_distance;
        
        // Geometric analysis
        bool enable_height_analysis;
        bool enable_shape_analysis;
        float shape_analysis_tolerance;
    };
    
    DetectionParams params_;
    
    // PCL clustering object
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_;
    
    /**
     * @brief Load detection parameters from ROS parameter server
     */
    void loadParameters();
    
    /**
     * @brief Perform Euclidean clustering on point cloud
     * @param cloud Input point cloud
     * @param cluster_indices Output cluster indices
     * @return true if successful, false otherwise
     */
    bool performClustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                          std::vector<pcl::PointIndices>& cluster_indices);
    
    /**
     * @brief Analyse a point cluster to determine if it's a cone
     * @param cloud Input point cloud
     * @param indices Cluster point indices
     * @param cone Output detected cone data
     * @return true if cluster is classified as cone, false otherwise
     */
    bool analyseConeCandidate(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                             const pcl::PointIndices& indices,
                             DetectedCone& cone);
    
    /**
     * @brief Calculate cluster geometric properties
     * @param cloud Input point cloud
     * @param indices Cluster point indices
     * @param centroid Output cluster centroid
     * @param height Output cluster height
     * @param radius Output cluster radius
     * @return true if successful, false otherwise
     */
    bool calculateClusterGeometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                                 const pcl::PointIndices& indices,
                                 geometry_msgs::Point& centroid,
                                 float& height,
                                 float& radius);
    
    /**
     * @brief Validate cone geometry against expected parameters
     * @param height Cone height
     * @param radius Cone radius
     * @param point_count Number of points in cluster
     * @return confidence score (0-1)
     */
    float validateConeGeometry(float height, float radius, int point_count);
    
    /**
     * @brief Publish cone detections as ROS markers for visualisation
     * @param cones Vector of detected cones
     */
    void publishConeMarkers(const std::vector<DetectedCone>& cones);
    
    /**
     * @brief Publish cone detections as ConeArrayWithCovariance message
     * @param cones Vector of detected cones
     * @param frame_id Frame ID for the message
     */
    // void publishConeArray(const std::vector<DetectedCone>& cones, const std::string& frame_id);
};

} // namespace hesai_lidar_driver

#endif // CONE_DETECTOR_H
