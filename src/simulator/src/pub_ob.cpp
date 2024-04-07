#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
 
int main(int argc, char** argv)
{
	ros::init(argc,argv,"pub_ob");
	// ros::NodeHandle nh;
 
	// ros::Publisher ob1_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_ob1",10);
    // ros::Publisher ob2_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_ob2",10);
    // ros::Publisher ob3_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_ob3",10);
    // ros::Publisher ob4_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_ob4",10);
 
	//1/20hz  20s
	ros::Rate loop_rate(10);
 
	float tag= 1;
 	
 
	//封装数据并且发布出去，延时满足频率
	while(ros::ok() ) 
	{
		//初始化geometry_msgs::Twist类型的消息  geometry_msgs::Twist是类，我们创建一个对象叫 vel_msg
		// geometry_msgs::Twist ob1_msg;
		// ob1_msg.linear.x =tag* 0.5;

        // geometry_msgs::Twist ob2_msg;
		// ob2_msg.linear.x = tag*0.3;

        // geometry_msgs::Twist ob3_msg;
		// ob3_msg.linear.x = tag*0.5;
		
        // geometry_msgs::Twist ob4_msg;
		// ob4_msg.linear.x =tag* 0.4;
		
        // tag=-tag;
		// //发布消息
		// // turtle_vel_pub是发布者， publish（）是方法
		// ob1_pub.publish(ob1_msg);
        // ob2_pub.publish(ob2_msg);
        // ob3_pub.publish(ob3_msg);
        // ob4_pub.publish(ob4_msg);

        ROS_INFO("watch");

 
		//按照循环频率延时
		loop_rate.sleep();
 
	}
 
	return 0;
}