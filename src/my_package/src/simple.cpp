#include <ros/ros.h>

int main(int argc, char** argv) {

    ros::init(argc,argv, "ObiWan");
    ros::NodeHandle nh;
    ROS_INFO("Posting");
    ros::spinOnce();
    return 0;
}