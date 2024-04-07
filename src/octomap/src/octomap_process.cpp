#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
octomap::OcTree* octomap_or = new octomap::OcTree(0.05);
ros::Publisher* pubOctomapPointer = NULL;

void registered_scan_handler(const sensor_msgs::PointCloud2ConstPtr& pointcloud_map)
{
    cloud->clear();
    pcl::fromROSMsg(*pointcloud_map, *cloud);
     for (auto p:cloud->points)
    {
        // 将点云里的点插入到octomap中
        octomap_or->updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }
    double mapRecTime = pointcloud_map->header.stamp.sec;
    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.frame_id = "/map";
    octomap_msg.header.stamp =ros::Time().fromSec( mapRecTime);
    if (octomap_msgs::fullMapToMsg(*octomap_or,octomap_msg))
        pubOctomapPointer->publish(octomap_msg);
    else
        ROS_ERROR("Error serializing OctoMap");

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_process");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    ros::Subscriber registered_scan_sub = nh.subscribe<sensor_msgs::PointCloud2>
                                    ("/registered_scan", 5, registered_scan_handler);
    ros::Publisher pub_octomap = nh.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
    pubOctomapPointer=&pub_octomap;

    while (ros::ok())
    {
      //ROS_INFO("bad");
      ros::spinOnce();
    }
}