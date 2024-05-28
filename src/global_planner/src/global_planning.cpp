#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include<thread>


#include "common/state/state.h"
#include "../include/planner_class.h"
//#include "../include/map_class.h"

using namespace global;







World_G* world_g = NULL;


double  global_work_rate=10;  /// test 0.01 for lont time

ros::Publisher tree_vis_pub;
ros::Publisher path_vis_pub;
ros::Publisher path_inter_pub;

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
    ROS_WARN("Excessive time interval between robot's odom and ob_state, t_robot  %f,  t_ob %f", t_robot,t_ob);
  }
  //ROS_INFO(" t_robot  %f,  t_ob %f", t_robot,t_ob);
  // robot start point 时间必须与障碍物时间对齐,
  // if t_robot<t_ob  robot stay at start point to wait  轨迹跟踪的时候要接上起始段
  // if t_robot>t_ob robot  plan at start point to wait  
  if (t_robot>t_ob ){
    skip_time=static_cast<int>(std::ceil((t_robot-t_ob)/deltatime));
    //ROS_INFO("%f,%d",t_robot-t_ob,skip_time);
  }
  // start_point[2]=t_ob;
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
    nh.param<double>("/global_planning/v_mean", v_mean,0);

    nh.param<double>("/global_planning/robot_r",  robot_r,0.2);
    nh.param<double>("/global_planning/safe_dis", safe_dis,0.1);
    nh.param<int>("/global_planning/iter_max_g", iter_max_g,2000000);
    nh.param<double>("/global_planning/goal_dis", goal_dis,1);
    nh.param<int>("/global_planning/interpolation_num", interpolation_num,10);
    ROS_INFO("pre_time:%f",pre_time);
    ROS_INFO("deltatime:%f",deltatime);

    nt=pre_time/deltatime+1;
    sur_discretePoints=new std::vector<std::vector<common::State>>(obstacle_num, std::vector<common::State>(nt));
    
    ros::Subscriber ob_state_all_sub = nh.subscribe( "/ob_state_all",  1,ob_state_all_cb  );
    ros::Subscriber odom_robot_pub = nh.subscribe("/odom1", 1,odom_robot_cb);

   tree_vis_pub = nh.advertise<visualization_msgs::Marker>("tree_vis", 1);
 path_vis_pub = nh.advertise<visualization_msgs::Marker>("path_vis", 1);
path_inter_pub = nh.advertise<std_msgs::Float32MultiArray>("path_inter", 1);
    //for world_g
    double lowerbound_x;
    double lowerbound_y;
    double upperbound_x;
    double upperbound_y;
    nh.param<double>("/global_planning/lowerbound_x", lowerbound_x,-6);
    nh.param<double>("/global_planning/lowerbound_y", lowerbound_y,-6);
    nh.param<double>("/global_planning/upperbound_x", upperbound_x,6);
    nh.param<double>("/global_planning/upperbound_y", upperbound_y,6);
    double resloution_x;
    double resloution_y;
    nh.param<double>("/global_planning/resloution_x", resloution_x,0.3);
    //resloution_x=v_mean*deltatime
    //resloution_y=v_mean*deltatime
    nh.param<double>("/global_planning/resloution_y", resloution_y,0.3);
    Eigen::Vector3d resolution(resloution_x,resloution_y,deltatime);
    world_g = new World_G(resolution);
    Eigen::Vector3d lowerbound(lowerbound_x,lowerbound_y,0);
    Eigen::Vector3d upperbound(upperbound_x,upperbound_y,pre_time);
    world_g->initGridMap(lowerbound,upperbound);

    AStar3D planner(obstacle_num,pre_time,deltatime,v_mean,nt,world_g,global_work_rate);
    planner.sur_discretePoints_=new std::vector<std::vector<common::State>>(obstacle_num, std::vector<common::State>(nt));
    planner.tree_vis_pub_ = &tree_vis_pub;
    planner.path_vis_pub_ = &path_vis_pub;
     planner.path_inter_pub_ = &path_inter_pub;
    std::thread(&AStar3D::planthread,&planner).detach();
    
    ros::Rate rate(100);
   
    while (ros::ok())
    {
      planner.visTree();
      ros::spinOnce();
      rate.sleep();
    }
 }