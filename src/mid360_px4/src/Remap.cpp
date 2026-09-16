#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped px4_pose;
ros::Publisher px4_pub;

void poseCb(const nav_msgs::Odometry::ConstPtr& msg) {
    px4_pose.header = msg->header;
    px4_pose.pose = msg->pose.pose;
    px4_pub.publish(px4_pose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Remap");
    ros::NodeHandle nh;
    px4_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    ros::Subscriber px4_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1, poseCb);

    ros::spin();
    return 0;
}


