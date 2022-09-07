#include <ros/ros.h>
#include <std_msgs/Float32.h>

ros::Publisher pub_right, pub_left;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveL");
    ros::NodeHandle nh;

    pub_left = nh.advertise<std_msgs::Float32>("/fw_asv0/left/cmd", 1);
    pub_right = nh.advertise<std_msgs::Float32>("/fw_asv0/right/cmd", 1);

    std_msgs::Float32 ctrl;
    ctrl.data = 1.0;

    while (ros::ok())
    {
       pub_right.publish(ctrl);
        //pub_right.publish(ctrl);

        ros::spin();  
    }
    
}