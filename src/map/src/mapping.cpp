#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include "../include/map_class.h"

using namespace Eigen;
float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool first_point_cloud=true;
double resolution=0.1;
World* world = NULL;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
ros::Publisher grid_map_vis_pub;
void odom_handler(const nav_msgs::Odometry::ConstPtr& odom)
{
  //odomTime = odom->header.stamp.toSec();
 
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;     //-x -y
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;

}

void pointCloud_handler(const sensor_msgs::PointCloud2ConstPtr& pointcloud_map)
{
  if(first_point_cloud)
  {
    //first_point_cloud=false;
    cloud->clear();
    pcl::fromROSMsg(*pointcloud_map, *cloud);
    pcl::PointXYZ point;
    int CloudSize = cloud->points.size();
    for (int i = 0; i < CloudSize; i++) {
      point = cloud->points[i];
      laserCloudCrop->push_back(point);
      }

    world->initGridMap(*laserCloudCrop);

    for (const auto& pt : *laserCloudCrop)
    {
      Vector3d obstacle(pt.x, pt.y, pt.z);
      world->setObs(obstacle);
    }
    
  }
  world->visWorld( &grid_map_vis_pub);
//   if (!useTerrainAnalysis) {
//     laserCloud->clear();
//     pcl::fromROSMsg(*laserCloud2, *laserCloud);

//     pcl::PointXYZI point;
//     laserCloudCrop->clear();
//     int laserCloudSize = laserCloud->points.size();
//     for (int i = 0; i < laserCloudSize; i++) {
//       point = laserCloud->points[i];

//       float pointX = point.x;
//       float pointY = point.y;
//       float pointZ = point.z;

//       float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
//       if (dis < adjacentRange) {
//         point.x = pointX;
//         point.y = pointY;
//         point.z = pointZ;
//         laserCloudCrop->push_back(point);
//       }
//     }

//     laserCloudDwz->clear();
//     laserDwzFilter.setInputCloud(laserCloudCrop);
//     laserDwzFilter.filter(*laserCloudDwz);

//     newLaserCloud = true;
//   }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
    nhPrivate.getParam("sensorOffsetY", sensorOffsetY);  
    nhPrivate.getParam("resolution", resolution);
    ros::Subscriber odom_sub= nh.subscribe<nav_msgs::Odometry>
                                    ("/odom1", 5, odom_handler);

    ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>
                                    ("/registered_scan", 5, pointCloud_handler);
    grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    
    world = new World(resolution);
    while (ros::ok())
    {
      //ROS_INFO("bad");
      ros::spinOnce();
    }
 }