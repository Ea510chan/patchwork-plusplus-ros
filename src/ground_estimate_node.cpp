#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "patchworkpp/patchworkpp.hpp"

// define XYZIRT point type
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D;  // This macro must be present for PCL to handle the point type
    float    intensity;
    uint16_t ring;
    float    time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Make sure our new allocators are aligned
} EIGEN_ALIGN16; // Ensure the custom point type is 16-bytes aligned

// Register the point type with PCL so it knows how to handle it
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, time, time))


using PointType = pcl::PointXYZI;
using CustomPointType = VelodynePointXYZIRT;
using namespace std;

boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

ros::Publisher pub_cloud;
ros::Publisher pub_ground;
ros::Publisher pub_non_ground;

// Function to transfer 'ring' and 'time' from nearest point in the original cloud
template <typename PointT, typename CustomPointT>
void transferRingAndTime(const pcl::PointCloud<PointT>& input_cloud,
                         const pcl::PointCloud<CustomPointT>& original_cloud,
                         pcl::PointCloud<CustomPointT>& output_cloud) {
    pcl::KdTreeFLANN<CustomPointT> kdtree;
    kdtree.setInputCloud(original_cloud.makeShared());

    for (const auto& point : input_cloud) {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        CustomPointT searchPoint;
        searchPoint.x = point.x;
        searchPoint.y = point.y;
        searchPoint.z = point.z;

        if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            const auto& nearest_point = original_cloud.points[pointIdxNKNSearch[0]];

            CustomPointT new_point = nearest_point;
            // new_point.intensity = point.intensity;

            output_cloud.push_back(new_point);
        }
    }
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, const ros::Time& stamp, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.stamp = stamp;
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    double time_taken;

    pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;
    pcl::PointCloud<CustomPointType> pc_raw;

    pcl::fromROSMsg(*cloud_msg, pc_raw);

    // Extract XYZI data from VelodynePointXYZIRT and populate the new cloud
    for (const auto& pt : pc_raw.points) {
        pcl::PointXYZI pt_curr;
        pt_curr.x = pt.x;
        pt_curr.y = pt.y;
        pt_curr.z = pt.z;
        pt_curr.intensity = pt.intensity;
        pc_curr.push_back(pt_curr);
    }
    // pcl::fromROSMsg(*cloud_msg, pc_curr);

    PatchworkppGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);

    ROS_INFO_STREAM("\033[1;32m" << "Input PointCloud: " << pc_curr.size() << " -> Ground: " << pc_ground.size() <<  "/ NonGround: " << pc_non_ground.size()
         << " (running_time: " << time_taken << " sec)" << "\033[0m");

    pcl::PointCloud<CustomPointType> pc_ground_xyzirt;
    transferRingAndTime(pc_ground, pc_raw, pc_ground_xyzirt);

    pub_cloud.publish(cloud2msg(pc_curr, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    pub_ground.publish(cloud2msg(pc_ground_xyzirt, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    pub_non_ground.publish(cloud2msg(pc_non_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
}

int main(int argc, char**argv) {

    ros::init(argc, argv, "Demo");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string cloud_topic;
    std::string robot_ns;
    pnh.param<string>("cloud_topic", cloud_topic, "/pointcloud");

    cout << "Operating patchwork++..." << endl;
    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&pnh));

    pub_cloud       = pnh.advertise<sensor_msgs::PointCloud2>("cloud", 100, true);
    pub_ground      = pnh.advertise<sensor_msgs::PointCloud2>("ground", 100, true);
    pub_non_ground  = pnh.advertise<sensor_msgs::PointCloud2>("nonground", 100, true);

    ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 100, callbackCloud);
    
    ros::spin();

    return 0;
}
