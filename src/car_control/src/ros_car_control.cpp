/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-10-27 20:41:16
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-10-28 15:31:11
 * @FilePath: /src/putn/src/car_control/src/ros_car_control.cpp
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

boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");     
boost::system::error_code err;


void doMsg(const geometry_msgs::Twist::ConstPtr& msg_p);
void serialInit(void)
{
    sp.set_option(serial_port::baud_rate(115200));      
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}

bool writeData(void)
{
    //测试数据
    unsigned char buff[15] = {0x7b, 0xaa, 0x11, 0x12, 0x11, 0x11, 0x11, 0x0d, 0x0a}; 
    // 通过串口下发数据
    boost::asio::write(sp, boost::asio::buffer(buff));
    return true;
}


int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"ros_car_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",10,doMsg);
    serialInit();
    while (ros::ok())
    {
        ros::spin();

    }

    return 0;
}


void doMsg(const geometry_msgs::Twist::ConstPtr& msg_p){
    ROS_INFO("v: %f",msg_p->linear.x);
    ROS_INFO("w:%f",msg_p->angular.z );

    stringstream s;
    s<<msg_p->linear.x<<","<<msg_p->angular.z;
    string str1;
    s>>str1;
    ROS_INFO("%s",str1.c_str());

    unsigned char data[30]={0x00};
    int len=str1.length();
    int  i=0;
    ROS_INFO("length: %d",len);
    for(i=1;i<len+1;i++)
    {
        data[i]=str1[i-1];
        ROS_INFO("str1%d: %0x",i,str1[i-1]);
    }
    data[0]=0x0f;
    data[len+1]=0x0d;
    for(i=0;i<len+2;i++)
    {

        ROS_INFO("data%d:  %0x",i,data[i]);
    }


    boost::asio::write(sp, boost::asio::buffer(data));
    //writeData();
}