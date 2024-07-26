#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "iostream"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/transform_broadcaster.h>  
#include <geometry_msgs/TransformStamped.h>  
using namespace std;

ros::Subscriber ar_sub_;
int obstacle_num;

ros::Publisher robot_odom_publisher;
ros::Publisher odom_all_pub;
std_msgs::Float32MultiArray ob_all_msg;
double time_move=1721971000;


class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
      ar_sub_ = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, &Localizer::number_callback, this);
  }

tf2_ros::TransformBroadcaster tf_broadcaster_;  
geometry_msgs::TransformStamped transformStamped;  
  void number_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
  {

    if(msg->detections.size()>0)
    {
        nav_msgs::Odometry robot_odom;
        
        int match[3]={0};
        for(int i=0;i<msg->detections.size()/2;i++){
            switch (msg->detections[i].id[0])
            {
                case 0: 
                    robot_odom.header.frame_id="/map";
                    robot_odom.child_frame_id="/odom1";
                    robot_odom.pose.pose = msg->detections[i].pose.pose.pose;
                    //obot_odom.pose.pose.position.z = 0;
                    robot_odom.header.stamp=msg->header.stamp;
                    robot_odom.header.stamp.sec=robot_odom.header.stamp.sec-time_move;
                    robot_odom_publisher.publish(robot_odom);
         
                    match[0]=1;



                    
                    transformStamped.header.stamp = msg->header.stamp;  
                    transformStamped.header.frame_id =  "/map";  
                    transformStamped.child_frame_id = "/base_footprint";  
                    transformStamped.transform.translation.x =msg->detections[i].pose.pose.pose.position.x;  
                    transformStamped.transform.translation.y =msg->detections[i].pose.pose.pose.position.y;  
                    transformStamped.transform.translation.z = msg->detections[i].pose.pose.pose.position.z;  
            
                    transformStamped.transform.rotation.x =msg->detections[i].pose.pose.pose.orientation.x;  
                     transformStamped.transform.rotation.y =msg->detections[i].pose.pose.pose.orientation.y;  
                      transformStamped.transform.rotation.z =msg->detections[i].pose.pose.pose.orientation.z;  
                     transformStamped.transform.rotation.w =msg->detections[i].pose.pose.pose.orientation.w;  

                    // 广播变换  
                    tf_broadcaster_.sendTransform(transformStamped);
                    //ROS_ERROR("tf ");
                    break;
                case 1: 
                    ob_all_msg.data[0*3]=msg->detections[i].pose.pose.pose.position.x;  
                    ob_all_msg.data[0*3+1]=msg->detections[i].pose.pose.pose.position.y;
                    ob_all_msg.data[0*3+2]=msg->header.stamp.toSec()-time_move;
                    ROS_INFO("ob_all_msg 1:%.2f",ob_all_msg.data[0*3+2]);
                    match[1]=1;
                    break;
                case 2: 
                    ob_all_msg.data[1*3]=msg->detections[i].pose.pose.pose.position.x;  
                    ob_all_msg.data[1*3+1]=msg->detections[i].pose.pose.pose.position.y;
                    ob_all_msg.data[1*3+2]=msg->header.stamp.toSec()-time_move;
                    ROS_INFO("ob_all_msg 2:%.2f",ob_all_msg.data[0*3+2]);
                    match[2]=1;
                    break;
                default:
                    std::cout << "april tag is other than 0, 1 and 2" << std::endl;
                    break;
            }
        }
        for(int i=0;i<obstacle_num+1;i++){
            if(match[i]==0){
                ROS_WARN("tag %d undetected",i);
            }
        }
        ros::Time right_now = ros::Time::now();
    
        ROS_INFO("now:%.2f",msg->header.stamp.toSec());
        ROS_INFO("now:%d",right_now.sec);

        odom_all_pub.publish(ob_all_msg);
    }
    else{
        ROS_WARN("No April_tag");
    }
  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv,"april_read");
    ros::NodeHandle nh;
    Localizer localizer(nh);    
    nh.param<int>("/april_read/obstacle_num", obstacle_num,2);
    robot_odom_publisher = nh.advertise<nav_msgs::Odometry>("/odom1", 10);
    odom_all_pub = nh.advertise<std_msgs::Float32MultiArray>("/odom_all", 10);
    ob_all_msg.data.resize(obstacle_num * 3);  
    ROS_INFO("节点开始");
    ros::spin();
    return 0;
}

