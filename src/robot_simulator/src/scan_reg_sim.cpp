#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double sensorOffsetX = 0;
double sensorOffsetY = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());
std::vector<int> scanInd;
ros::Publisher* pubScanPointer = NULL;

void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odom,
                                  const sensor_msgs::PointCloud2ConstPtr& scanIn)

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


//   if (!systemInited) {
//     systemInitCount++;
//     if (systemInitCount > systemDelay) {
//       systemInited = true;
//     }
//     return;
//   }

  //double scanTime = scanIn->header.stamp.toSec();

//   if (odomSendIDPointer < 0)
//   {
//     return;
//   }
//   while (odomTimeStack[(odomRecIDPointer + 1) % stackNum] < scanTime &&
//          odomRecIDPointer != (odomSendIDPointer + 1) % stackNum)
//   {
//     odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
//   }

  double odomRecTime = scanIn->header.stamp.sec;
  float vehicleRecX = vehicleX;
  float vehicleRecY = vehicleY;
  float vehicleRecZ = vehicleZ;
  float vehicleRecRoll = vehicleRoll;
  float vehicleRecPitch = vehiclePitch;
  float vehicleRecYaw = vehicleYaw;

  ROS_INFO("X:%f       ,Y:%f,       Z%f",vehicleRecX,vehicleRecY,vehicleRecZ);
  ROS_INFO("Roll :%f       ,Pitch :%f,       Yaw%f",vehicleRecRoll,vehicleRecPitch,vehicleRecYaw);
//   float terrainRecRoll = terrainRoll;
//   float terrainRecPitch = terrainPitch;

//   if (use_gazebo_time)
//   {
//     odomRecTime = odomTimeStack[odomRecIDPointer];
//     vehicleRecX = vehicleXStack[odomRecIDPointer];
//     vehicleRecY = vehicleYStack[odomRecIDPointer];
//     vehicleRecZ = vehicleZStack[odomRecIDPointer];
//     vehicleRecRoll = vehicleRollStack[odomRecIDPointer];
//     vehicleRecPitch = vehiclePitchStack[odomRecIDPointer];
//     vehicleRecYaw = vehicleYawStack[odomRecIDPointer];
//     terrainRecRoll = terrainRollStack[odomRecIDPointer];
//     terrainRecPitch = terrainPitchStack[odomRecIDPointer];
//   }

//   float sinTerrainRecRoll = sin(terrainRecRoll);
//   float cosTerrainRecRoll = cos(terrainRecRoll);
//   float sinTerrainRecPitch = sin(terrainRecPitch);
//   float cosTerrainRecPitch = cos(terrainRecPitch);

float sinTerrainRecRoll = sin(vehicleRecRoll);
float cosTerrainRecRoll = cos(vehicleRecRoll);
float sinTerrainRecPitch = sin(vehicleRecPitch);
float cosTerrainRecPitch = cos(vehicleRecPitch);
float sinTerrainRecYaw = sin(vehicleRecYaw);
float cosTerrainRecYaw = cos(vehicleRecYaw);


  scanData->clear();
  pcl::fromROSMsg(*scanIn, *scanData);
  pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);

  int scanDataSize = scanData->points.size();
  for (int i = 0; i < scanDataSize; i++)
  {
    float pointX1 = scanData->points[i].x;
    float pointY1 = scanData->points[i].y * cosTerrainRecRoll - scanData->points[i].z * sinTerrainRecRoll;
    float pointZ1 = scanData->points[i].y * sinTerrainRecRoll + scanData->points[i].z * cosTerrainRecRoll;

    float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
    float pointY2 = pointY1;
    float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

    float pointX3 = pointX2* cosTerrainRecYaw  -pointY2*sinTerrainRecYaw;
    float pointY3 = pointX2* sinTerrainRecYaw  +pointY2*cosTerrainRecYaw;
    float pointZ3 =pointZ2;

    float pointX4 = pointX3 + vehicleRecX;
    float pointY4 = pointY3 + vehicleRecY;
    float pointZ4 = pointZ3 + vehicleRecZ;

    scanData->points[i].x = pointX4;
    scanData->points[i].y = pointY4;
    scanData->points[i].z = pointZ4;
  }

  // publish 5Hz registered scan messages
  sensor_msgs::PointCloud2 scanData2;
  pcl::toROSMsg(*scanData, scanData2);
  scanData2.header.stamp = ros::Time().fromSec(odomRecTime);
  scanData2.header.frame_id = "/map";
  pubScanPointer->publish(scanData2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_reg_sim");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
    nhPrivate.getParam("sensorOffsetY", sensorOffsetY);  

    // ros::Subscriber odom_sub= nh.subscribe<nav_msgs::Odometry>
    //                                 ("/odom1", 5, odom_handler);
    // ros::Subscriber point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, point_cloud_handler);
    ros::Publisher pubScan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 2);
    pubScanPointer = &pubScan;



    message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
    message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
    typedef message_filters::Synchronizer<syncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    subOdometry.subscribe(nh, "/odom1", 1);
    subLaserCloud.subscribe(nh, "/velodyne_points", 1);
    sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
    sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));

    while (ros::ok())
    {
      //ROS_INFO("bad");
      ros::spinOnce();
    }
}