#include <ros/ros.h>
#include "common/state/state.h"
//#include "../include/map_class.h"

std::vector<std::vector<common::State>> sur_discretePoints; 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_planning");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nh.param<int>("/ssc_map/obstacle_num", obstacle_num,0);
    ros::Rate rate(10);
   
    while (ros::ok())
    {
      
      ros::spinOnce();
      rate.sleep();
    }
 }