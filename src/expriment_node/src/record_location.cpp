#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>  
#include <random> 
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>  
#include <fstream>  


int obstacle_num;



int main(int argc, char** argv)
{
    ros::init(argc, argv, "record_location");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    
    nh.param<int>("/record_location/obstacle_num", obstacle_num,0);
    // nh.param<double>("/record_location/robot_r", car_r,0);
    // nh.param<double>("/record_location/obstacle_radius", ob_r,0);
    // nh.param<double>("/record_location/goal_dis_exper", goal_dis,0);
    // nh.param<double>("/record_location/time_max_exper", time_max,0);
    
 


    
    ros::Publisher goal_p =nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);  
    goal_pub=&goal_p;

    ros::Subscriber odom_robot_sub = nh.subscribe("/odom1", 1,odom_robot_cb);
    ros::Subscriber ob_sub = nh.subscribe( "/odom_all",  1,od_cb  );
    ros::Subscriber path_sub = nh.subscribe( "/path_inter",  1,path_cb  );
    ros::Subscriber v_sub = nh.subscribe( "/local_plan",  1,v_cb  );
    // 设置发布频率  
    ros::Rate loop_rate(10); // 10Hz 

    


    
    while (ros::ok())  
    {  
  
        
        ros::spinOnce();  
        loop_rate.sleep();  
    }  

  

  
    return 0;  
}