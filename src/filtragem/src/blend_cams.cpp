#include <ros/ros.h>

#include <opencv4/opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

ros::Publisher pub;

float alpha;

void callback (const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right)
{
    cv_bridge::CvImagePtr cv_ptr_left, cv_ptr_right, dest_ptr;

    try
    {
        cv_ptr_left = cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::RGB8);
        cv_ptr_right = cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    pub.publish(cv_ptr_left->toImageMsg());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "blend_cams");

    ros::NodeHandle nh;

    nh.param<float>("/alpha", alpha, 0.5);
    //nh.param<int>("/max_search", max_search, 50);

    pub = nh.advertise<sensor_msgs::Image>("blended_image", 1);

    message_filters::Subscriber<sensor_msgs::Image> left_sub (nh, "/fw_asv0/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub (nh, "/fw_asv0/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, sensor_msgs::Image> myPolicy ;
    message_filters::Synchronizer <myPolicy> sync (myPolicy(10), left_sub, right_sub);
    sync.registerCallback(&callback);
    
    ros::spin();
}