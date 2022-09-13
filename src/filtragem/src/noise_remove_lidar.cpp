#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/statistical_outlier_removal.h>

ros::Subscriber sub;
ros::Publisher pub;

float meanK;
float threshold;

void callback (const sensor_msgs::PointCloud2ConstPtr& ptc)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL (*ptc, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (temp_cloud);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh(threshold);
    sor.setNegative(false);
    sor.filter (*cloud_filtered);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_filtered,msg);
    pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "noise_remove_lidar");

    ros::NodeHandle nh;
    nh.param<float>("/meanK", meanK, 50);
    nh.param<float>("/threshold", threshold, 1.0);

    pub = nh.advertise<sensor_msgs::PointCloud2>("noise_free_PointCloud2", 10);

    sub = nh.subscribe("passthrough_PointCloud2", 10, callback);
    
    ros::spin();
}