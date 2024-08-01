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

// typedef union{   //定义一个共用体，用于double数据与16进制的转换
// unsigned char cvalue[4];
// float fvalue;
// }float_union;

 boost::asio::io_service iosev;
    boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");     //
    boost::system::error_code err;
    const unsigned char ender[2] = {0x0d, 0x0a};           
    const unsigned char header[2] = {0x55, 0xaa};          

short transition=0;

void doMsg(const geometry_msgs::Twist::ConstPtr& msg_p);
bool writeData(void);
void serialInit(void);

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"ros_car_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel",10,doMsg);

    serialInit();
    ros::spin();

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
    // unsigned char data_trans[20];
    // data_trans[0]=0x0a;
    // data_trans[19]=0x0d;
    
    // char data_trans1[18]={"0"};
    // //sprintf(data_trans1,str1.c_str());
    // for (int i=0;i<18;i++)
    // {
    //      data_trans[i+1]=data_trans1[i];
    // }
    // ROS_INFO("%s",data_trans1);
    
//     static uint8_t s_buffer[10];
//     float_union linear_x ,angular_z;
//     memset(s_buffer,0,sizeof(s_buffer));
//     linear_x.fvalue = msg_p->linear.x;
// 	angular_z.fvalue = msg_p->angular.z;

//     s_buffer[0] = 0xff;//帧头
// 	s_buffer[1] = linear_x.cvalue[0];
// 	s_buffer[2] = linear_x.cvalue[1];
// 	s_buffer[3] = linear_x.cvalue[2];
// 	s_buffer[4] = linear_x.cvalue[3];

// 	s_buffer[5] = angular_z.cvalue[0];
// 	s_buffer[6] = angular_z.cvalue[1];
// 	s_buffer[7] = angular_z.cvalue[2];
// 	s_buffer[8] = angular_z.cvalue[3];
	
// 	s_buffer[9] = 0x0d;//帧尾
//     //s_buffer[10] = 0x0a;//帧尾
//  ROS_INFO("v: %0x",s_buffer[1]);
//     boost::asio::write(sp, boost::asio::buffer(s_buffer));

//正数
    // unsigned char buff[15] ;
    // memset(buff,0,sizeof(buff));

    // buff[0]=0x7b;
    // buff[1]=0;

    // transition=0;
    // transition=msg_p->linear.x*1000;
    // ROS_INFO("transition_v  %d",transition);
    // buff[3]=transition;
    // buff[2]=transition>>8;

    // transition=0;
    // transition=msg_p->angular.z*1000;
    // ROS_INFO("transition_w  %d",transition);
    // buff[5]=transition;
    // buff[4]=transition>>8;

    // buff[6]=0x0d;
    // buff[7]=0x0a;

    boost::asio::write(sp, boost::asio::buffer(str1));

//正数
    //writeData();
}





void serialInit(void)
{
    sp.set_option(serial_port::baud_rate(115200));      
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}
 

bool writeData()
{
    
    unsigned char buff[15] = {0x7b, 0xaa, 0x11, 0x12, 0x11, 0x11, 0x11, 0x0d, 0x0a}; 
    
    boost::asio::write(sp, boost::asio::buffer(buff));
    return true;
}
 

bool readData()
{
    unsigned char buff[11]={0};
    uint8_t buffer[11]={0};
    
    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(sp, response, "",err);   
        copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),
        istream_iterator<unsigned char>(),
        buff); 
    }  
    catch(boost::system::system_error &err)
    {
        ROS_INFO("read_until error");
    }
    //buffer = buff[0];
    for(int i = 0; i < sizeof(buffer); i++)
{
buffer[i] = buff[i];
ROS_INFO("buffer[%d]: %d", i, buffer[i]);
}
return true; 
}