#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

ros::Publisher pub_right, pub_left, pub_safe;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveL");
    ros::NodeHandle nh;

    pub_safe = nh.advertise<std_msgs::Bool>("safety_stop",1);
    std_msgs::Bool stop;
    stop.data = false;
    pub_safe.publish(stop);

    pub_left = nh.advertise<std_msgs::Float32>("/fw_asv0/left/cmd", 1);
    pub_right = nh.advertise<std_msgs::Float32>("/fw_asv0/right/cmd", 1);

    std_msgs::Float32 ctrl;
    ctrl.data = 1.0;

    while (ros::ok())
    {
        pub_right.publish(ctrl);
        pub_left.publish(ctrl);
    }
    
}