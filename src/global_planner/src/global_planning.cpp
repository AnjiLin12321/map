#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include "common/state/state.h"
//#include "../include/map_class.h"

std::vector<std::vector<common::State>>* sur_discretePoints=nullptr;; 
int  obstacle_num;
double pre_time;
double deltatime;
int nt;


std::vector<double> start_point(3,0);
//[0 -6 t]
std::vector<double> goal_point(3,0);
//[0 6 t]


void ob_state_all_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
   for (int i=0; i<obstacle_num; i++) 
  {
    std::vector<common::State> &traj =(*sur_discretePoints)[i];
    for(int j = 0; j<nt; j++){
      double t=j*deltatime;
      traj[j].vec_position[0]=msg->data[i*7*nt+j*7];
      traj[j].vec_position[1]=msg->data[i*7*nt+j*7+1];
      traj[j].angle=msg->data[i*7*nt+j*7+2];
      traj[j].vec_velocity[0]=msg->data[i*7*nt+j*7+3];
      traj[j].vec_velocity[1]=msg->data[i*7*nt+j*7+4];
      traj[j].r=msg->data[i*7*nt+j*7+5];
      traj[j].time_stamp=msg->data[i*7*nt+j*7+6];

      //  if(i==0)
      //   ROS_INFO("ob: %d t:%f x: %f y: %f Y: %f vx: %f vy: %f  ",i,t,traj[j].vec_position[0],traj[j].vec_position[1],traj[j].angle,
      // traj[j].vec_velocity[0],traj[j].vec_velocity[1]);
    }
      

    }

}

void odom_robot_cb(const  nav_msgs::Odometry::ConstPtr & msg)
{
  start_point[0]=msg->pose.pose.position.x;
  start_point[1]=msg->pose.pose.position.y;
  double t_robot=msg->header.stamp.toSec();
  double t_ob=(*sur_discretePoints)[0][0].time_stamp;
  if(t_robot-t_ob>0.5||t_robot-t_ob<-0.5)
  {
    ROS_ERROR("Excessive time interval between robot's odom and ob_state");
  }
  start_point[2]=t_robot;
  goal_point[2]=t_ob+pre_time;
  goal_point[1]=6;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_planning");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nh.param<int>("/global_planning/obstacle_num", obstacle_num,0);
    nh.param<double>("/global_planning/pre_time", pre_time,0);
    nh.param<double>("/global_planning/deltatime", deltatime,0);
    ROS_INFO("pre_time:%f",pre_time);
    ROS_INFO("deltatime:%f",deltatime);

    nt=pre_time/deltatime+1;
    sur_discretePoints=new std::vector<std::vector<common::State>>(obstacle_num, std::vector<common::State>(nt));

    ros::Subscriber ob_state_all_sub = nh.subscribe( "/ob_state_all",  1,ob_state_all_cb  );
    ros::Subscriber odom_robot_pub = nh.subscribe("/odom1", 1,odom_robot_cb);

    ros::Rate rate(100);
   
    while (ros::ok())
    {
      
      ros::spinOnce();
      rate.sleep();
    }
 }