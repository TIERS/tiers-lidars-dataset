#include "livox_ros_driver/CustomMsg.h"
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
 
typedef pcl::PointXYZI PointType;

#define  PI  3.1415926535

using namespace std;

ros::Publisher pub_ros_points;
string frame_id = "avia_frame";
 

void livoxLidarHandler(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
    pcl::PointCloud<PointType> pcl_in; 
    for (unsigned int i = 0; i < livox_msg_in->point_num; ++i) {
        PointType pt;
        pt.x = livox_msg_in->points[i].x;
        pt.y = livox_msg_in->points[i].y;
        pt.z = livox_msg_in->points[i].z; 
        pt.intensity =  livox_msg_in->points[i].reflectivity;
        pcl_in.push_back(pt);
    }
  
    ros::Time timestamp(livox_msg_in->header.stamp.toSec());

    sensor_msgs::PointCloud2 pcl_ros_msg;
    pcl::toROSMsg(pcl_in, pcl_ros_msg);
    pcl_ros_msg.header.stamp = timestamp;
    pcl_ros_msg.header.frame_id = frame_id;

    pub_ros_points.publish(pcl_ros_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_format_converter");
    ros::NodeHandle nh;
   
    ros::Subscriber sub_livox_lidar = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, livoxLidarHandler);
    pub_ros_points = nh.advertise<sensor_msgs::PointCloud2>("/livox_ros_points", 100);

    ros::spin();
}
