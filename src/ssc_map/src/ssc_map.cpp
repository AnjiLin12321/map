#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>  
#include <visualization_msgs/Marker.h>  
#include<math.h>
#include <std_msgs/Float32MultiArray.h>
#include "common/state/state.h"

#include "../include/read_compare_truth.h"
//#include "../include/map_class.h"

common::State s;
//s.angle=1;
typedef Eigen::Matrix<float,5,2> ob_data_type;  //x y vx vy t 
std::vector<ob_data_type> ob_data_all ;

double inittime;
bool flag_fisttime=true;

//using namespace Eigen;
int  obstacle_num;

double pre_time;
double deltatime;


ros::Publisher ob_traj_pub_vis;
ros::Publisher ob_state_pub;

std::vector<Eigen::Vector2d> ped_0;
std::vector<Eigen::Vector2d> ped_1;
std::vector<Eigen::Vector2d> ped_2;
std::vector<Eigen::Vector2d> ped_3;
std::vector<Eigen::Vector2d> ped_4;

int time_index;


void Publish_ob() {
  //publish sur trajs;
  visualization_msgs::MarkerArray surtrajs;
  if(obstacle_num!=ob_data_all.size()) {
    ROS_WARN("different obstacle num   %d",ob_data_all.size());
    return;
  }
  for(int i = 0; i < obstacle_num; i++){
    visualization_msgs::Marker traj;
    traj.action = visualization_msgs::Marker::ADD;
    traj.id = i;
    traj.type = visualization_msgs::Marker::LINE_STRIP;
    traj.pose.orientation.w = 1.00;
    traj.color.r = 1.00;
    traj.color.g = 0.00;
    traj.color.b = 0.00;
    traj.color.a = 1.00;
    traj.scale.x = 0.1;
    traj.scale.y = 0.1;
    traj.scale.z = 0.1;
    traj.header.frame_id = "map";
    traj.header.stamp =ros::Time().fromSec(ob_data_all[i](4,1));
    //traj.header.stamp =ros::Time().fromSec(cur_time);
    geometry_msgs::Point point1;
     int nt=pre_time/deltatime+1;
     for(int j = 0; j<nt; j++){
      double t=j*deltatime;
      //State state = getState(cur_time + t, inittime, i);

      point1.x = ob_data_all[i](0,1)+ob_data_all[i](2,1)*t ;   //x+vx*t
      point1.y =  ob_data_all[i](1,1)+ob_data_all[i](3,1)*t ; //y+vy*t
     // if(point1.x) point1.x+=0.1;
      point1.z =t; 
     // point1.z=ob_data_all[i](4,1)-inittime+t;
      // point1.z =atan2( ob_data_all[i](3,1),ob_data_all[i](2,1));
      traj.points.push_back(point1);
    }
  surtrajs.markers.push_back(traj);

  }
  ob_traj_pub_vis.publish(surtrajs);


  

}


void od_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //ROS_INFO("[node] receive the path");
    
    if(flag_fisttime){
      inittime=msg->data[3+2];
      flag_fisttime=false;
    }
    if(msg->data.size()/3 != obstacle_num) 
    {
    ROS_ERROR("obstacle_num Wrong!");
    return;
    }

    for (int i=0; i<obstacle_num; i++) 
    {
        ob_data_type& ob_i=ob_data_all[i];
        double dtime=msg->data[3*i+2]- ob_i(4, 0); 
        for (int i = 0; i < ob_i.rows(); ++i) {  
            // 将第二列的值赋给第一列  
            // 第一列 pre 第二列 cur
            ob_i(i, 0) = ob_i(i, 1);  
        } 
        ob_i(0,1) =msg->data[3*i];
        ob_i(1,1) =msg->data[3*i+1];
        ob_i(4,1) =msg->data[3*i+2];
        ob_i(2,1) =( ob_i(0,1)- ob_i(0,0))/dtime;  //vx
        ob_i(3,1) =( ob_i(1,1)- ob_i(1,0))/dtime;  //vy
        //ROS_INFO("%d: x:%f  y:%f  vx:%f  vy:%f t:%f  dt:%f", i+1,ob_i(0,1) , ob_i(1,1), ob_i(2,1), ob_i(3,1),ob_i(4,1),dtime);
	}
}

void DyObsMap(){
  std::vector<std::vector<common::State>> sur_trajs;

   std_msgs::Float32MultiArray ob_state_all_msg;
  int nt=pre_time/deltatime+1;
  ob_state_all_msg.data.resize(  7*nt*obstacle_num);
  for (int i=0; i<obstacle_num; i++) 
  {
    std::vector<common::State> traj;
    double cur_time = ob_data_all[i](4,1);
    for(int j = 0; j<nt; j++){
      double t=j*deltatime;
      common::State state;
      state.vec_position[0] = ob_data_all[i](0,1)+ob_data_all[i](2,1)*t;
      state.vec_position[1] = ob_data_all[i](1,1)+ob_data_all[i](3,1)*t;
      state.angle = atan2( ob_data_all[i](3,1),ob_data_all[i](2,1));
      //state.curvature = 1.0/radiuss[idx];
      state.vec_velocity[0] = ob_data_all[i](2,1);
      state.vec_velocity[1] = ob_data_all[i](3,1);
      //state.acceleration = 0.0; 
      state.time_stamp =cur_time+ t;
       state.r=0.2;
      traj.push_back(state);
      
      //pub
      ob_state_all_msg.data[i*7*nt+j*7]=state.vec_position[0];  
      ob_state_all_msg.data[i*7*nt+j*7+1]= state.vec_position[1];
      ob_state_all_msg.data[i*7*nt+j*7+2]= state.angle ;
      ob_state_all_msg.data[i*7*nt+j*7+3]= state.vec_velocity[0] ;
      ob_state_all_msg.data[i*7*nt+j*7+4]=  state.vec_velocity[1] ;
      ob_state_all_msg.data[i*7*nt+j*7+5]= state.r;
      ob_state_all_msg.data[i*7*nt+j*7+6]= state.time_stamp ;
      
      // if(i==4)
      //   ROS_INFO("ob: %d t:%f x: %f y: %f Y: %f vx: %f vy: %f r: %f t:%f ",i,t,state.vec_position[0],state.vec_position[1],state.angle,
      // state.vec_velocity[0],state.vec_velocity[1], state.r,state.time_stamp);
    }
    sur_trajs.push_back(traj);
  }
  
  //p_smm_->set_sur_points(sur_trajs);

  ob_state_pub.publish(ob_state_all_msg );

}


void ob1_compare_cb(const  nav_msgs::Odometry::ConstPtr & msg)
{
  double ob1_x=msg->pose.pose.position.x;
  double ob1_y=msg->pose.pose.position.y;
  double err=10000;
  for(int i=0;i<ped_0.size();i++){
    double err1=(ob1_x-ped_0[i][0])*(ob1_x-ped_0[i][0])+(ob1_y-ped_0[i][1])*(ob1_y-ped_0[i][1]);
    if(err1<err)
    {
      err=err1;
      time_index=i;
    }
  }
  //ROS_INFO("time_index %d",time_index);
  
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ssc_map");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nh.param<int>("/ssc_map/obstacle_num", obstacle_num,0);
    ros::Subscriber ob_sub = nh.subscribe( "/odom_all",  1,od_cb  );
    ob_data_all.resize(obstacle_num); 
    nh.param<double>("/ssc_map/pre_time", pre_time,0);
    nh.param<double>("/ssc_map/deltatime", deltatime,0);
    ROS_INFO("pre_time:%f",pre_time);
    ROS_INFO("deltatime:%f",deltatime);
    // for visulization
    ob_traj_pub_vis = nh.advertise<visualization_msgs::MarkerArray>("/vis/parking_surround_trajs", 10);


   ob_state_pub = nh.advertise<std_msgs::Float32MultiArray>("/ob_state_all_1", 1000);
   // read from   orca_circle_crossing_5ped_1scenes_.txt
   read_truth_path();
   ros::Subscriber ob1_compare = nh.subscribe( "/actor1_odom",  1,ob1_compare_cb  );
   //test
  //  printf("-------------------- \n");
  //  for(int i=0;i<ped_0.size();i++){
  //   printf("i %d x %f, y:%f  \n",i,ped_0[i][0],ped_0[i][1]);
  //  }
  //   printf("-------------------- \n");
  //  for(int i=0;i<ped_0.size();i++){
  //   printf("i %d x %f, y:%f  \n",i,ped_1[i][0],ped_1[i][1]);
  //  }
  //   printf("-------------------  \n");
  //  for(int i=0;i<ped_0.size();i++){
  //   printf("i %d x %f, y:%f   \n",i,ped_2[i][0],ped_2[i][1]);
  //  }
  //   printf("--------------------  \n");
  //  for(int i=0;i<ped_0.size();i++){
  //   printf("i %d x %f, y:%f  \n",i,ped_3[i][0],ped_3[i][1]);
  //  }
  //   printf("--------------------  \n");
  //  for(int i=0;i<ped_0.size();i++){
  //   printf("i %d x %f, y:%f   \n",i,ped_4[i][0],ped_4[i][1]);
  //  }
    ros::Rate rate(100);
   
    while (ros::ok())
    {
      
      // ROS_INFO("inittime :%f",inittime);
      // ROS_INFO("now:%f",ros::Time::now().toSec());
      //Publish_ob();
      //DyObsMap();
      ros::spinOnce();
      rate.sleep();
    }
 }