#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include<thread>


#include "common/state/state.h"
#include "../include/planner_class.h"
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h> 
//#include "../include/map_class.h"

using namespace global;





World_G* world_g = NULL;


double  global_work_rate=10;  /// test 0.01 for lont time

ros::Publisher sub_goal_vis;
ros::Publisher tree_vis_pub;
ros::Publisher path_vis_pub;
ros::Publisher path_inter_pub;

ros::Publisher bound_pub_vis;

void ob_state_all_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  int nt_ori=pre_time/delta_time+1 ; //12+1
  int sample_ind=delta_time/sampletime;//2

  for (int i=0; i<obstacle_num; i++) 
  {
    std::vector<common::State> &traj =(*sur_discretePoints)[i];
    common::State pre_state;
    pre_state.vec_position[0]=msg->data[i*7*nt_ori];
    pre_state.vec_position[1]=msg->data[i*7*nt_ori+1];
    pre_state.angle=msg->data[i*7*nt_ori+2];
    pre_state.vec_velocity[0]=msg->data[i*7*nt_ori+3];
    pre_state.vec_velocity[1]=msg->data[i*7*nt_ori+4];
    pre_state.r=msg->data[i*7*nt_ori+5];
    pre_state.time_stamp=msg->data[i*7*nt_ori+6];

    traj[0].vec_position[0]=msg->data[i*7*nt_ori];
    traj[0].vec_position[1]=msg->data[i*7*nt_ori+1];
    traj[0].angle=msg->data[i*7*nt_ori+2];
    traj[0].vec_velocity[0]=msg->data[i*7*nt_ori+3];
    traj[0].vec_velocity[1]=msg->data[i*7*nt_ori+4];
    traj[0].r=msg->data[i*7*nt_ori+5];
    traj[0].time_stamp=msg->data[i*7*nt_ori+6];

    for(int j = 1; j<nt_ori; j++){
      common::State cur_state;
      cur_state.vec_position[0]=msg->data[i*7*nt_ori+j*7];
      cur_state.vec_position[1]=msg->data[i*7*nt_ori+j*7+1];
      cur_state.angle=msg->data[i*7*nt_ori+j*7+2];
      cur_state.vec_velocity[0]=msg->data[i*7*nt_ori+j*7+3];
      cur_state.vec_velocity[1]=msg->data[i*7*nt_ori+j*7+4];
      cur_state.r=msg->data[i*7*nt_ori+j*7+5];
      cur_state.time_stamp=msg->data[i*7*nt_ori+j*7+6];

      common::State  delta_state;
      delta_state.vec_position[0]= cur_state.vec_position[0]-pre_state.vec_position[0];
      delta_state.vec_position[1]=cur_state.vec_position[1]- pre_state.vec_position[1];
      delta_state.angle= cur_state.angle- pre_state.angle;
      delta_state.vec_velocity[0]=cur_state.vec_velocity[0]- pre_state.vec_velocity[0];
      delta_state.vec_velocity[1]=cur_state.vec_velocity[1]-pre_state.vec_velocity[1];
      delta_state.r=cur_state.r- pre_state.r;
      delta_state.time_stamp= cur_state.time_stamp- pre_state.time_stamp;
      for(int k=1;k<=sample_ind;k++)
      {
        int ind =(j-1)*sample_ind+k;
         traj[ind].vec_position[0]=delta_state.vec_position[0]*k/sample_ind+pre_state.vec_position[0];
          traj[ind].vec_position[1]=delta_state.vec_position[1]*k/sample_ind+ pre_state.vec_position[1];
          traj[ind].angle=delta_state.angle*k/sample_ind+pre_state.angle;
          traj[ind].vec_velocity[0]=delta_state.vec_velocity[0]*k/sample_ind+pre_state.vec_velocity[0];
          traj[ind].vec_velocity[1]=delta_state.vec_velocity[1]*k/sample_ind+pre_state.vec_velocity[1];
          traj[ind].r=delta_state.r*k/sample_ind+pre_state.r;
          traj[ind].time_stamp=delta_state.time_stamp*k/sample_ind+pre_state.time_stamp;
        
      }
      //(int k=(j-1)*delta_time;k<=j*delta_time;j+=sampletime)

      //  if(i==0)
      //   ROS_INFO("ob: %d t:%f x: %f y: %f Y: %f vx: %f vy: %f  ",i, traj[j].time_stamp,traj[j].vec_position[0],traj[j].vec_position[1],traj[j].angle,
      // traj[j].vec_velocity[0],traj[j].vec_velocity[1]);
      pre_state.vec_position[0]=cur_state.vec_position[0];
      pre_state.vec_position[1]=cur_state.vec_position[1];
      pre_state.angle=cur_state.angle;
      pre_state.vec_velocity[0]=cur_state.vec_velocity[0];
      pre_state.vec_velocity[1]=cur_state.vec_velocity[1];
      pre_state.r=cur_state.r;
      pre_state.time_stamp=cur_state.time_stamp;
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
  // if (t_robot>t_ob ){
  //   skip_time=static_cast<int>(std::ceil((t_robot-t_ob)/delta_time));
  //   ROS_INFO("%f,%d",t_robot-t_ob,skip_time);
  // }
  // start_point[2]=t_ob;
  start_point[2]=t_robot;
  goal_point[2]=t_ob+pre_time;
  //goal_point[1]=-3;
}

void rcvWaypointsCallback(const nav_msgs::Path& wp)
{
  // if (!world->has_map_)
  //   return;
  // has_goal = true;
  // target_pt = Vector3d(wp.poses[0].pose.position.x, wp.poses[0].pose.position.y, wp.poses[0].pose.position.z);
  goal_point[0]=wp.poses[0].pose.position.x;
  goal_point[1]=wp.poses[0].pose.position.y;
  ROS_INFO("Receive the planning target");
}

void Publish_boundry(double lowerbound_x,double lowerbound_y,double upperbound_x,double upperbound_y) {
  visualization_msgs::MarkerArray surtrajs;
  visualization_msgs::Marker traj;
  traj.action = visualization_msgs::Marker::ADD;
  traj.id = 0;
  traj.type = visualization_msgs::Marker::LINE_STRIP;
  traj.pose.orientation.w = 1.00;
  traj.color.r = 0.00;
  traj.color.g = 0.00;
  traj.color.b = 1.00;
  traj.color.a = 1.00;
  traj.scale.x = 0.1;
  traj.scale.y = 0.1;
  traj.scale.z = 0.1;
  traj.header.frame_id = "map";
  //traj.header.stamp =ros::Time().fromSec(cur_time);
  geometry_msgs::Point point1;
  point1.x=lowerbound_x;
  point1.y=lowerbound_y;
  traj.points.push_back(point1);

  geometry_msgs::Point point2;
  point2.x=upperbound_x;
  point2.y=lowerbound_y;
  traj.points.push_back(point2);

  geometry_msgs::Point point3;
  point3.x=upperbound_x;
  point3.y=upperbound_y;
  traj.points.push_back(point3);

  geometry_msgs::Point point4;
  point4.x=lowerbound_x;
  point4.y=upperbound_y;
  traj.points.push_back(point4);

  geometry_msgs::Point point5;
  point5.x=lowerbound_x;
  point5.y=lowerbound_y;
  traj.points.push_back(point5);
  surtrajs.markers.push_back(traj);


  bound_pub_vis.publish(surtrajs);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_planning");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nh.param<int>("/global_planning/obstacle_num", obstacle_num,0);
    nh.param<double>("/global_planning/pre_time", pre_time,0);
    nh.param<double>("/global_planning/delta_time", delta_time,0);
    nh.param<double>("/global_planning/sampletime", sampletime,0);
    nh.param<double>("/global_planning/v_mean", v_mean,0);

    nh.param<double>("/global_planning/robot_r",  robot_r,0.4);
    nh.param<double>("/global_planning/safe_dis", safe_dis,1);
    nh.param<int>("/global_planning/iter_max_g", iter_max_g,2000000);
    nh.param<double>("/global_planning/goal_dis", goal_dis,0.5);
    nh.param<int>("/global_planning/interpolation_num", interpolation_num,5);
    nh.param<int>("/global_planning/max_subgoal_num",  max_subgoal_num,10);
 
    nh.param<bool>("/global_planning/type_a3d",  type_a3d,true);  //1 for a3d   ;  2 for cbf
    
    
    nt=pre_time/sampletime+1;
    sur_discretePoints=new std::vector<std::vector<common::State>>(obstacle_num, std::vector<common::State>(nt));
    
    ros::Subscriber ob_state_all_sub = nh.subscribe( "/ob_state_all",  1,ob_state_all_cb  );
    ros::Subscriber odom_robot_sub = nh.subscribe("/odom1", 1,odom_robot_cb);
     ros::Subscriber wp_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallback);

  sub_goal_vis= nh.advertise<visualization_msgs::MarkerArray>("sub_goal_vis", 1);
  //<visualization_msgs::MarkerArray>
  //<geometry_msgs::PoseArray>
   tree_vis_pub = nh.advertise<visualization_msgs::Marker>("tree_vis", 1);
   path_vis_pub = nh.advertise<visualization_msgs::Marker>("path_vis", 1);
  path_inter_pub = nh.advertise<std_msgs::Float32MultiArray>("path_inter", 1);

  
    //for world_g
    double lowerbound_x;
    double lowerbound_y;
    double upperbound_x;
    double upperbound_y;
    nh.param<double>("/global_planning/lowerbound_x", lowerbound_x,-10);
    nh.param<double>("/global_planning/lowerbound_y", lowerbound_y,-10);
    nh.param<double>("/global_planning/upperbound_x", upperbound_x,10);
    nh.param<double>("/global_planning/upperbound_y", upperbound_y,10);

    bound_pub_vis = nh.advertise<visualization_msgs::MarkerArray>("bound_vis", 10);
    
    double resloution_x;
    double resloution_y;
    
    resloution_x=v_mean*sampletime;
    resloution_y=v_mean*sampletime;
    // nh.param<double>("/global_planning/resloution_x", resloution_x,0.3);
    // nh.param<double>("/global_planning/resloution_y", resloution_y,0.3);
    Eigen::Vector3d resolution(resloution_x,resloution_y,sampletime);
    world_g = new World_G(resolution);
    Eigen::Vector3d lowerbound(lowerbound_x,lowerbound_y,0);
    Eigen::Vector3d upperbound(upperbound_x,upperbound_y,pre_time);
    world_g->initGridMap(lowerbound,upperbound);

    // planner 
    AStar3D planner(obstacle_num,pre_time,sampletime,v_mean,nt,world_g,global_work_rate);
    planner.sur_discretePoints_=new std::vector<std::vector<common::State>>(obstacle_num, std::vector<common::State>(nt));
    planner.sub_goal_vis_ = &sub_goal_vis;
    planner.tree_vis_pub_ = &tree_vis_pub;
    planner.path_vis_pub_ = &path_vis_pub;
    planner.path_inter_pub_ = &path_inter_pub;

    std::thread(&AStar3D::planthread,&planner).detach();
    
    ros::Rate rate(100);
   
    while (ros::ok())
    {
      //planner.visTree();
      Publish_boundry(lowerbound_x,lowerbound_y,upperbound_x,upperbound_y);
      ros::spinOnce();
      rate.sleep();
    }
 }