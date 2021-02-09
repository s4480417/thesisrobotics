/*
* PCL Example using ROS and CPP
*/

// Include the ROS library
#include <ros/ros.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

static const std::string PUBLISH_TOPIC = "/pcl/points";

// ROS Publisher
ros::Publisher pub;

int main (int argc, char** argv)
{
    // Initialize the ROS Node "roscpp_pcl_example"
    ros::init(argc, argv, "roscpp_pcl_example");
    ros::NodeHandle nh;

    // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < 100; i++) {
        pcl::PointXYZ point;
        point.x = (rand() * 1.0) / RAND_MAX;
        point.y = (rand() * 1.0) / RAND_MAX;
        point.z = (rand() * 1.0) / RAND_MAX;
        cloud.points.push_back(point);
    }
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "root";
    cloud_msg.header.stamp = ros::Time::now();
    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);
    ros::Rate rate(30);
    while (ros::ok()) {
        cloud_msg.header.stamp = ros::Time::now();
        pub.publish(cloud_msg);
        ros::spinOnce();
        rate.sleep(); 
    }

    // Success
    return 0;
}