#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


ros::Subscriber sub;
ros::Publisher pub;

void callback (const sensor_msgs::ImuConstPtr& imu, const sensor_msgs::PointCloud2ConstPtr& ptc)
{
    long int len = ptc->data.size();
    ROS_INFO("original size: %ld",len);
    ROS_INFO("calculated size: %d", ptc->width*ptc->height *ptc->point_step);

    tf2::Quaternion ori;
    ori.setX (imu->orientation.x);
    ori.setY (imu->orientation.y);
    ori.setZ (imu->orientation.z);
    ori.setW (imu->orientation.w);  
    
    std::vector<tf2::Quaternion> q_vec;

    for(int i=0; i<len; i+=3)
    {
        tf2::Quaternion q;
        q.setX (ptc->data[i]);
        q.setY (ptc->data[i+1]);
        q.setZ (ptc->data[i+2]);
        q.setW (1);
        q.normalize();

        q_vec.push_back( q.operator*=(ori.operator*=(q.inverse())) ); //problema
    }

    int counter = 0;
    std::vector<uint8_t> data;

    for(int i=0; i<q_vec.size(); i++)
    {
        data.push_back ( q_vec[counter].getX());
        data.push_back ( q_vec[counter].getY());
        data.push_back ( q_vec[counter].getZ());
        counter++;
    }

    sensor_msgs::PointCloud2 msg;
    msg = *ptc;   
    msg.data = data;    
    //msg.data = ptc->data;
    pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mat");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    pub = n.advertise<sensor_msgs::PointCloud2>("filtred_PointCloud2__mat", 1);

    message_filters::Subscriber<sensor_msgs::Imu> imu_sub (nh, "/fw_asv0/imu", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> ptc_sub (nh, "/fw_asv0/velodyne_points", 1);
    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::Imu, sensor_msgs::PointCloud2> myPolicy ;
    message_filters::Synchronizer <myPolicy> sync (myPolicy(10), imu_sub, ptc_sub);
    sync.registerCallback(&callback);
    
    ros::spin();
}