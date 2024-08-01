/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-10-27 20:24:01
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-10-27 20:36:39
 * @FilePath: /src/ros_control/src/ros_car/src/ros_car_control.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <strstream>

 
using namespace std;
using namespace boost::asio;



int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"ros_car_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",10,doMsg);

     while (ros::ok())
    {
        ros::spin();
        a
    }

    return 0;
}


void doMsg(const geometry_msgs::Twist::ConstPtr& msg_p){
    ROS_INFO("v: %f",msg_p->linear.x);
    ROS_INFO("w:%f",msg_p->angular.z );


}
