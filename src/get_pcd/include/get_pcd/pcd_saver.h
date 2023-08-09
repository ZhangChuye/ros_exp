#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
// #include <dynamic_filter/Submap.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>


#include <tf/message_filter.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <boost/functional/hash.hpp>

#include <omp.h>
#include <math.h>

#include <limits>
#include <iostream>
#include <string>
#include <queue>
#include <deque>
#include <vector>
#include <set>
#include <mutex>
#include <thread>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>

#include <boost/make_shared.hpp>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>


typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

class PcdSaver
{
public:
    explicit PcdSaver(ros::NodeHandle nh);

    virtual ~PcdSaver();

    void subscribePointcloud();

    void scanCallback(
        const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg);

    bool getTransformPose(
        const ros::Time &stamp,
        const std::string &target_frame,
        const std::string &source_frame,
        geometry_msgs::Pose &pose);


private:
    ros::NodeHandle nh_;

    ros::Subscriber map_sub_;

    ros::Publisher compare_map_pub_;
    ros::Publisher preprocess_scan_pub_;

    tf::TransformListener tf_listener_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> scan_sub_;
    std::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> scan_filter_;

    PointCloudPtr map_global_;

    std::string local_frame_;
    std::string global_frame_;

    std::string static_map_path_;

    std::mutex map_global_lock_;

    std::fstream file_;
    char s[200];

    long long scans_count_;

};
