#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // This function will be called whenever a PointCloud2 message is received
    // Add your processing code here
    ROS_INFO("Node is running");
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "lidar_subscriber");
    ros::NodeHandle nh;

    // Create a subscriber to listen to the scan_3D topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("scan_3D", 1, callback);

    // Enter the main ROS loop
    ros::spin();

    return 0;
}
