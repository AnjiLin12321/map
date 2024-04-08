#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>  
#include <visualization_msgs/Marker.h>  
#include<math.h>
//#include "../include/map_class.h"


typedef Eigen::Matrix<float,5,2> ob_data_type;  //x y vx vy t 
std::vector<ob_data_type> ob_data_all ;

double inittime;
bool flag_fisttime=true;

//using namespace Eigen;
int  obstacle_num;

double pre_time;
double deltatime;


ros::Publisher ob_traj_pub;

//state should be a seperate class
// struct State {
//   double time_stamp{0.0};
//   Eigen::Matrix<double, 2, 1> vec_position{Eigen::Matrix<double, 2, 1> ::Zero()};
//   double angle{0.0};  // heading angle
//   double curvature{0.0};
//   Eigen::Matrix<double, 2, 1> vec_velocity{Eigen::Matrix<double, 2, 1> ::Zero()};
//   //double velocity{0.0};
//   //double acceleration{0.0};
//   double steer{0.0};  // steering angle
//   void print() const {
//     printf("State:\n");
//     printf(" -- time_stamp: %lf.\n", time_stamp);
//     printf(" -- vec_position: (%lf, %lf).\n", vec_position[0], vec_position[1]);
//     printf(" -- angle: %lf.\n", angle);
//     printf(" -- curvature: %lf.\n", curvature);
//     printf(" -- vec_velocity:  (%lf, %lf).\n", vec_velocity[0], vec_velocity[1]);
//     //printf(" -- acceleration: %lf.\n", acceleration);
//     printf(" -- steer: %lf.\n", steer);
//   }

//   Vec3f ToXYTheta() const {
//     return Vec3f(vec_position(0), vec_position(1), angle);
//   }
// };
// State getState(double now_t, double init_t, int id){
//   //
//   double px = vehicles[id].radius * cos(vehicles[id].inityaw + (now_t- init_t)*vehicles[id].desiredOmg) + vehicles[id].center[0];
//   double py = vehicles[id].radius * sin(vehicles[id].inityaw + (now_t- init_t)*vehicles[id].desiredOmg) + vehicles[id].center[1];
//   double cur_yaw = vehicles[id].inityaw + (now_t- init_t)*vehicles[id].desiredOmg + M_PI_2;
//   State state;
//   //state.acceleration = 0.0;
//   //state.angle = cur_yaw;
//   state.curvature  = 1.0/vehicles[id].radius;
//   state.time_stamp = ros::Time::now().toSec();
//   state.vec_position << px,py;
//   state.velocity = vehicles[id].desiredV;
//   return state;
// }

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
    for(double t = 0.0; t<=pre_time; t += deltatime){
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
  ob_traj_pub.publish(surtrajs);

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
        ROS_INFO("%d: x:%f  y:%f  vx:%f  vy:%f t:%f  dt:%f", i+1,ob_i(0,1) , ob_i(1,1), ob_i(2,1), ob_i(3,1),ob_i(4,1),dtime);
	}
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
    ob_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("/vis/parking_surround_trajs", 10);
    ros::Rate rate(10);
   
    while (ros::ok())
    {
      ROS_INFO("inittime :%f",inittime);
      ROS_INFO("now:%f",ros::Time::now().toSec());
       Publish_ob();
      ros::spinOnce();
      rate.sleep();
    }
 }