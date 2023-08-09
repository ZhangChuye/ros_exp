#include "get_pcd/pcd_saver.h"

//  A Ros node that record rosbag and write point cloud into pcd file
//  As well as record the transformation
//  Prepartion to write rosbag into Kitti data structure 

PcdSaver::PcdSaver(ros::NodeHandle nh) : nh_(nh)
{
    map_global_.reset(new PointCloud());
    map_global_->clear();
    scans_count_ = 0;
    // file to write to, if there is something already in it, this will empty the file.  
    file_.open("/home/tingxfan/poses.txt",std::ios_base::out);
}

PcdSaver::~PcdSaver()
{
    file_.close();
}

bool PcdSaver::getTransformPose(
    const ros::Time &stamp,
    const std::string &target_frame,
    const std::string &source_frame,
    geometry_msgs::Pose &pose)
{
    tf::StampedTransform transform_msg;
    try
    {
        tf_listener_.lookupTransform(target_frame, source_frame, stamp, transform_msg);
    }

    catch (tf::TransformException ex)
    {
        ROS_WARN_STREAM(ex.what());
        return false;
    }

    pose.position.x = transform_msg.getOrigin().x();
    pose.position.y = transform_msg.getOrigin().y();
    pose.position.z = transform_msg.getOrigin().z();

    pose.orientation.x = transform_msg.getRotation().x();
    pose.orientation.y = transform_msg.getRotation().y();
    pose.orientation.z = transform_msg.getRotation().z();
    pose.orientation.w = transform_msg.getRotation().w();

    return true;
}

void PcdSaver::scanCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg)
{
    ros::Time stamp = pointcloud_msg->header.stamp;

    geometry_msgs::Pose frame_pose;
    if (!getTransformPose(stamp, "odom", "base_link", frame_pose))
    {
        return;
    }

    scans_count_ += 1;
    if (scans_count_ % 1 != 0)
    {
        return;
    }

    PointCloudPtr scan_local(new PointCloud());
    
    pcl::fromROSMsg(*pointcloud_msg, *scan_local);

	// The place to save pcd file
    sprintf(s, "/home/tingxfan/Tools_RosBag2KITTI/catkin_ws/output/pcd/%06lld.pcd", scans_count_-1); 
    
    ROS_INFO_STREAM("Saving point cloud to " << s);
    pcl::io::savePCDFileBinary(s, *scan_local);

    // from Quaternions to Homogeneous Transformation Matrices(first three rows)
    // file poses.txt in Kitti
    file_ << (1 - 2 * frame_pose.orientation.y * frame_pose.orientation.y - 2 * frame_pose.orientation.z * frame_pose.orientation.z);
    file_ << " ";
    file_ << (2 * frame_pose.orientation.x * frame_pose.orientation.y - 2 * frame_pose.orientation.w * frame_pose.orientation.z);
    file_ << " ";
    file_ << (2 * frame_pose.orientation.x * frame_pose.orientation.z + 2 * frame_pose.orientation.w * frame_pose.orientation.y);
    file_ << " ";
    file_ << frame_pose.position.x;
    file_ << " ";

    file_ << 2 * frame_pose.orientation.x * frame_pose.orientation.y + 2 * frame_pose.orientation.w * frame_pose.orientation.z;
    file_ << " ";
    file_ << 1 - 2 * frame_pose.orientation.x * frame_pose.orientation.x - 2 * frame_pose.orientation.z * frame_pose.orientation.z;
    file_ << " ";
    file_ << 2 * frame_pose.orientation.y * frame_pose.orientation.z - 2 * frame_pose.orientation.w * frame_pose.orientation.x;
    file_ << " ";
    file_ << frame_pose.position.y;
    file_ << " ";

    file_ << 2 * frame_pose.orientation.x * frame_pose.orientation.z - 2 * frame_pose.orientation.w * frame_pose.orientation.y;
    file_ << " ";
    file_ << 2 * frame_pose.orientation.y * frame_pose.orientation.z + 2 * frame_pose.orientation.w * frame_pose.orientation.x;
    file_ << " ";
    file_ << 1 - 2 * frame_pose.orientation.x * frame_pose.orientation.x - 2 * frame_pose.orientation.y * frame_pose.orientation.y;
    file_ << " ";
    file_ << frame_pose.position.z;
    file_ << " ";

    file_ << std::endl;

}

void PcdSaver::subscribePointcloud()
{
    // topic name
    scan_sub_.subscribe(nh_, "/os_cloud_node/points", 1);
    scan_filter_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(
        scan_sub_,
        tf_listener_,
        "odom",
        10));

    scan_filter_->registerCallback(boost::bind(&PcdSaver::scanCallback, this, _1));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_saver");
    ros::NodeHandle nh("~");

    PcdSaver pcd(nh);

    pcd.subscribePointcloud();

    ros::spin();

    return 0;
}
