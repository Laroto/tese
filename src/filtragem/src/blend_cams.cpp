#include <ros/ros.h>

#include <opencv4/opencv2/opencv.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

ros::Publisher pub;

void callback (const sensor_msgs::ImageConstPtr& cam_msg, const sensor_msgs::ImuConstPtr& imu_msg)
{
    cv_bridge::CvImagePtr cam, res;

    ROS_INFO("Subscribed");

    try
    {
        cam = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    tf2::Quaternion q;
    q.setX(imu_msg->orientation.x);
    q.setY(imu_msg->orientation.y);
    q.setZ(imu_msg->orientation.z);
    q.setW(imu_msg->orientation.w);
    q.inverse();

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    cv::Point2f pc(cam->image.cols/2., cam->image.rows/2.);
    cv::Mat rot;
    rot = cv::getRotationMatrix2D(pc, roll, 1.0);    
    cv::warpAffine (cam->image, res->image, rot, cam->image.size());

    cv::imwrite("rotated_im.png", res->image);

    pub.publish(res->toImageMsg());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "blend_cams");

    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::Image>("blended_image", 1);

    message_filters::Subscriber<sensor_msgs::Image> cam_sub (nh, "/fw_asv0/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub (nh, "/fw_asv0/imu", 1);
    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Image, sensor_msgs::Imu> myPolicy ;
    message_filters::Synchronizer <myPolicy> sync (myPolicy(1), cam_sub, imu_sub);
    sync.registerCallback(&callback);
    
    ros::spin();
}