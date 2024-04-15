#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <std_msgs/Float32MultiArray.h>




ros::Publisher* actor1_odom_pubPointer = NULL;
int obstacle_num=0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_process");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    ros::ServiceClient states_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state"); //客户端获取机器人模型状态

    nh.param<int>("/actor2odom/obstacle_num", obstacle_num,0);
    std::vector<std::unique_ptr<ros::Publisher>> actor_odom_pubs;  

    for (int i = 1; i <= obstacle_num; ++i) {  
        std::string topic_name = "/actor" + std::to_string(i) + "_odom";  
        actor_odom_pubs.emplace_back(  
            new ros::Publisher(nh.advertise<nav_msgs::Odometry>(topic_name, 10))  
        );  
          
        // 可选：打印出创建的发布者信息  
       // ROS_INFO("Created publisher for topic: %s", topic_name.c_str());  
    }  

    //用于将多个障碍物存在一个里面，便于后续调用
    ros::Publisher odom_all_pub = nh.advertise<std_msgs::Float32MultiArray>("/odom_all", 1000);
    std_msgs::Float32MultiArray ob_all_msg;
    ob_all_msg.data.resize(obstacle_num * 3);  //x y t 
    
   // ros::Publisher actor1_odom_pub = nh.advertise<nav_msgs::Odometry>("/actor1_odom", 10);  //定义里程计
    //actor1_odom_pubPointer=& actor1_odom_pub;


     ros::Publisher ob1_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_ob1",10);
    ros::Publisher ob2_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_ob2",10);
    ros::Publisher ob3_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_ob3",10);
    ros::Publisher ob4_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_ob4",10);
    float tag= 1;
    int count=0;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        for (int i = 0; i < obstacle_num; ++i) {  
            gazebo_msgs::GetModelState model_states;
            nav_msgs::Odometry actor_odom;
            std::string ator_name = "actor" + std::to_string(i+1) ;  
            model_states.request.model_name =ator_name; 
            states_client.call(model_states); //客户端发送请求
            actor_odom.pose.pose = model_states.response.pose;
            //actor_odom.pose.pose.position.z = 0;
            actor_odom.header.stamp=model_states.response.header.stamp;
            //actor1_odom.twist.twist = model_states.response.twist;
            actor_odom.header.frame_id="map";
            std::string odom_id = "/actor" + std::to_string(i+1) + "_odom"; 
            actor_odom.child_frame_id=odom_id;
            actor_odom_pubs[i]->publish(actor_odom);


            ob_all_msg.data[i*3]=actor_odom.pose.pose.position.x;  
            ob_all_msg.data[i*3+1]=actor_odom.pose.pose.position.y;
            ob_all_msg.data[i*3+2]=actor_odom.header.stamp.toSec();
            //ROS_INFO("%d: x:%f  y:%f  t:%f",i+1, ob_all_msg.data[i*3] , ob_all_msg.data[i*3+1], ob_all_msg.data[i*3+2]);

        }  
        odom_all_pub.publish(ob_all_msg);



        geometry_msgs::Twist ob1_msg;
		ob1_msg.linear.x =tag* 0.5;
        geometry_msgs::Twist ob2_msg;
		ob2_msg.linear.x = tag*0.3;
        geometry_msgs::Twist ob3_msg;
		ob3_msg.linear.x = tag*0.5;
        geometry_msgs::Twist ob4_msg;
		ob4_msg.linear.x =tag* 0.4;
		
        count++;
        if(count==200)
        {
            count=0;
            tag=-tag;
        }
        //tag=-tag;
		//发布消息
		// turtle_vel_pub是发布者， publish（）是方法
		ob1_pub.publish(ob1_msg);
        ob2_pub.publish(ob2_msg);
        ob3_pub.publish(ob3_msg);
        ob4_pub.publish(ob4_msg);


         loop_rate.sleep();
        ros::spinOnce();
        // ROS_INFO("Created publisher for topic:");  
        //ROS_INFO("num: %d", obstacle_num);  
    }
      return 0;
}