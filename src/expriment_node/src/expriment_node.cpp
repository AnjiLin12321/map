#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>  
#include <random> 
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>  
#include <fstream>  


const double MAX_DIS=1000;
int obstacle_num;
double car_r=0.4;
double ob_r=0.25;
double goal_dis=1;
//double last_goal_dis=2;
double time_max=20.0;
double result[200][4];

std::vector<std::vector<double>> collision_record;
 std::string filename_collision = "/home/linanji/src/map/cp0_5_collison.txt";  
std::ofstream outfile_collison(filename_collision);  
double v[200];

std::random_device rd_wait;  
std::mt19937 gen(rd_wait());  
std::uniform_real_distribution<> dis(0.0, 4.0);  
  

ros::Publisher* goal_pub = NULL;;
double calculateVariance(const double* data, size_t size) {  
    if (size <= 1) {  
        // 方差未定义或始终为0（当只有一个元素时）  
        return 0.0;  
    }  
  
    double sum = 0.0;  
    for (size_t i = 0; i < size; ++i) {  
        sum += data[i];  
    }  
    double mean = sum / size;  
  
    double varianceSum = 0.0;  
    for (size_t i = 0; i < size; ++i) {  
        varianceSum += std::pow(data[i] - mean, 2);  
    }  
  
    // 注意：这是样本方差（n-1作为分母），如果是总体方差则使用size作为分母  
    return varianceSum / (size - 1);  
}  


class Exper{
    public:
        double goal_x=-6;
        double goal_y=6;

        double last_goal_x;
        double last_goal_y;

        double odom_x;
        double odom_y;

        bool collision_tag=false;
        bool timeout_tag=false;
        ros::Time start_time;
        ros::Time end_time;  // ros::Time::now(); 



        bool goal_begin_tag=false;
        int global_path_times=0;
        double e_global_time;
        double e_global_time_overall;

        double e_time=0;  

        int N_expre=100;
        int n_cur=0;
        int n_cal=0;



        int collision=0;
        int time_out=0;

        double min_dis=MAX_DIS;
        double e_min_dis=0;

        int v_times=0;
        double v_var=0;
        double e_v_var=0;
        
        

        Exper(){
            initRNG();
        }
        
       
        void pub_rand_goal(){
            random_goal();
            //collision_tag=false;
            geometry_msgs::PoseStamped pose_msg;  
            pose_msg.header.stamp = ros::Time::now();  
            pose_msg.header.frame_id = "map";  
            pose_msg.pose.position.x = goal_x;  
            pose_msg.pose.position.y = goal_y;  
            pose_msg.pose.position.z = 0.0;  
    
            pose_msg.pose.orientation.x = 0.0;  
            pose_msg.pose.orientation.y = 0.0;  
            pose_msg.pose.orientation.z = 0.0;  
            pose_msg.pose.orientation.w = 1.0; // 表示没有旋转，即四元数为[0, 0, 0, 1]  
    
            // 发布消息  
            goal_pub->publish(pose_msg);  
        }
    private:  
        // 随机数生成器  
        std::mt19937 rng;  
        // 均匀分布的随机数生成器  
        std::uniform_real_distribution<double> uniformDisX{4.5, 5.5}; // x的范围  
        std::uniform_real_distribution<double> uniformDisY{4.5, 5.5}; // 假设y的范围和x相同，也可以不同  
        
        // 初始化随机数生成器  
        void initRNG() {  
            rng = std::mt19937(42); // 使用固定的种子值42  
    }  
        void random_goal() { 
            last_goal_x=goal_x;
            last_goal_y=goal_y;
            if(n_cur%2){
                // goal_x = -uniformDisX(rng); 
                // goal_y = uniformDisY(rng); 

                goal_x = -6; 
                goal_y = 6; 
            }
            else{
                // goal_x = uniformDisX(rng); 
                // goal_y =- uniformDisY(rng); 
                goal_x = 6; 
                goal_y = -6; 

            }
            

    
            // 输出新的x和y值（可选）  
            //std::cout<< "n_cur: " << n_cur << ", New goal_x: " << goal_x << ", New goal_y: " << goal_y << std::endl;  
        } 

        
};
Exper exper_node;
Exper* exper_ptr = new Exper();  

double cal_dis(double x1,double y1, double x2,double y2){
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}
void odom_robot_cb(const  nav_msgs::Odometry::ConstPtr & msg)
{
    exper_node.odom_x=msg->pose.pose.position.x;
    exper_node.odom_y=msg->pose.pose.position.y;

    if((exper_node.odom_x-exper_node.goal_x)*(exper_node.odom_x-exper_node.goal_x)+
    (exper_node.odom_y-exper_node.goal_y)*(exper_node.odom_y-exper_node.goal_y)<goal_dis*goal_dis)  //arrive goal
    {
        if(exper_node.n_cur<=exper_node.N_expre){  //i < 200
            if(exper_node.n_cur==0){ //start
               exper_node.end_time=ros::Time::now();
               exper_node.start_time=ros::Time::now();
               
               exper_node.collision=0;

                std::cout <<"start_time:"<< exper_node.start_time<<std::endl;
            }
            else{
                exper_node.start_time=exper_node.end_time;
                exper_node.end_time=ros::Time::now();

                
                
                
                double time_cost=exper_node.end_time.toSec()-exper_node.start_time.toSec();

                double a = dis(gen);  
                // ROS_INFO("Waiting for %.2f seconds...", a);  
                ros::Duration(a).sleep();  
                // ROS_INFO("Waited for %.2f seconds...", a);  
                
                exper_node.end_time=ros::Time::now();

                if(time_cost>time_max){
                    exper_node.timeout_tag=true;
                }

                if(exper_node.collision_tag)//(exper_node.n_cal<exper_node.n_cur)
                {
                     std::cout <<"n_cur:"<< exper_node.n_cur<<" time:" <<exper_node.end_time.toSec()<<" time cost: "<<time_cost<<"collison! pass"<<std::endl;
                    result[exper_node.n_cur-1][0]=-1;
                    if (!outfile_collison.is_open()) {  
                        std::cerr << "无法打开文件 " << filename_collision << std::endl;  
                    }
                    outfile_collison<<"n_cur:"<< exper_node.n_cur<<" time:" <<exper_node.end_time.toSec()<<std::endl; 
                    for (int i = 0; i < collision_record.size(); ++i) {  
                        for (int j = 0; j < 2+obstacle_num*2; ++j) {  
                            outfile_collison << collision_record[i][j]; // 写入元素  
                            if (j < 2+obstacle_num*2-1) {  
                                outfile_collison << " "; // 元素之间用空格分隔（可选）  
                            }  
                        }  
                        outfile_collison << std::endl; // 每行结束后换行  
                    }  
                
                    // 关闭文件  
                    //outfile_collison.close();  
                    
                }
                else if(exper_node.timeout_tag){
                        exper_node.time_out++;  //无碰撞才考虑超时！！！！
                        std::cout <<"n_cur:"<< exper_node.n_cur<<" time:" <<exper_node.end_time.toSec()<<" time cost: "<<time_cost<<" time out times: "<<exper_node.time_out<<std::endl;
                        exper_node.n_cal=exper_node.n_cur;
                        result[exper_node.n_cur-1][1]=-1;
                }
                else{
                    exper_node.e_time=(exper_node.e_time*(exper_node.n_cur-exper_node.collision-exper_node.time_out-1)+time_cost)/(exper_node.n_cur-exper_node.collision-exper_node.time_out);
                    exper_node.e_min_dis=(exper_node.e_min_dis*(exper_node.n_cur-exper_node.collision-exper_node.time_out-1)+exper_node.min_dis)/(exper_node.n_cur-exper_node.collision-exper_node.time_out);
                    exper_node.e_global_time_overall=(exper_node.e_global_time_overall*(exper_node.n_cur-exper_node.collision-exper_node.time_out-1)+exper_node.e_global_time)/(exper_node.n_cur-exper_node.collision-exper_node.time_out);
                    exper_node.v_var=calculateVariance(v,exper_node.v_times);
                    exper_node.e_v_var=(exper_node.e_v_var*(exper_node.n_cur-exper_node.collision-exper_node.time_out-1)+exper_node.v_var)/(exper_node.n_cur-exper_node.collision-exper_node.time_out);
                    

                    std::cout <<"n_cur:"<< exper_node.n_cur<<" time:" <<exper_node.end_time.toSec()<<" time cost: "<<time_cost<<" average time:"<<exper_node.e_time
                    <<std::endl
                    <<" min_dis: "<<exper_node.min_dis<<" average min_dis: "<<exper_node.e_min_dis
                    <<" goal time: "<<exper_node.e_global_time<<" average goal time: "<<exper_node.e_global_time_overall
                    <<" v_var: "<<exper_node.v_var<<" average v_var: "<<exper_node.e_v_var
                    <<std::endl;

                    result[exper_node.n_cur-1][0]=time_cost;
                    result[exper_node.n_cur-1][1]=exper_node.min_dis;
                    result[exper_node.n_cur-1][2]=exper_node.e_global_time;
                    result[exper_node.n_cur-1][3]=exper_node.v_var;
                    exper_node.n_cal=exper_node.n_cur;
                }
                   
                   
                

               

            }
            exper_node.collision_tag=false;
            exper_node.timeout_tag=false;

            exper_node.goal_begin_tag=true;
            exper_node.global_path_times=0;
            exper_node.pub_rand_goal() ;
            exper_node.n_cur++;
            //std::cout <<"v_times:"<< exper_node.v_times<<std::endl;
            exper_node.v_times=0;

            collision_record.clear();
          
        }
        
        else{
            std::cout <<"over all: N="<< exper_node.N_expre<<" collision="<< exper_node.collision<<" Timeout(without collision)="<< exper_node.time_out<<
            " average time="<< exper_node.e_time<<" average min_dis:="<<exper_node.e_min_dis<<" average goal  time: "<<exper_node.e_global_time_overall<<" average v_var: "<<exper_node.e_v_var<<std::endl;
            std::string filename = "/home/linanji/src/map/cp0_5.txt";  
  
            // 打开文件以写入数据  
            std::ofstream outfile(filename);  
            if (!outfile.is_open()) {  
                std::cerr << "无法打开文件 " << filename << std::endl;  
            }  
        

            for (int i = 0; i < exper_node.N_expre; ++i) {  
                for (int j = 0; j < 4; ++j) {  
                    outfile << result[i][j]; // 写入元素  
                    if (j < 4 - 1) {  
                        outfile << " "; // 元素之间用空格分隔（可选）  
                    }  
                }  
                outfile << std::endl; // 每行结束后换行  
            }  
        
            // 关闭文件  
            outfile.close();  
            ros::shutdown();
            outfile_collison.close();
        }
    }
    
}
void od_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for (int i=0; i<obstacle_num; i++) 
    {
        double ob_x=msg->data[3*i];
        double ob_y=msg->data[3*i+1];
        
        double distance=cal_dis(exper_node.odom_x,exper_node.odom_y,ob_x,ob_y);
        exper_node.min_dis=exper_node.min_dis<distance?exper_node.min_dis:distance;

     }
    std::vector<double> odom_collison; 
    odom_collison.push_back(exper_node.odom_x);
    odom_collison.push_back(exper_node.odom_y);
    
     for (int i=0; i<obstacle_num; i++) 
    {
            double ob_x=msg->data[3*i];
            double ob_y=msg->data[3*i+1];
            odom_collison.push_back(ob_x);
            odom_collison.push_back(ob_y);
    }
    collision_record.push_back(odom_collison);


    if(!exper_node.collision_tag)//(exper_node.n_cal<exper_node.n_cur)
    {
        // if((exper_node.odom_x-exper_node.last_goal_x)*(exper_node.odom_x-exper_node.last_goal_x)
        // +(exper_node.odom_y-exper_node.last_goal_y)*(exper_node.odom_y-exper_node.last_goal_y)<last_goal_dis*last_goal_dis){

        // }
        
        for (int i=0; i<obstacle_num; i++) 
        {
            double ob_x=msg->data[3*i];
            double ob_y=msg->data[3*i+1];
            
            if((ob_x-exper_node.odom_x)*(ob_x-exper_node.odom_x)+(ob_y-exper_node.odom_y)*(ob_y-exper_node.odom_y)
            <(car_r+ob_r)*(car_r+ob_r)){
                if((cal_dis(exper_node.odom_x,exper_node.odom_y,-0.9,6.9)<1.5) ||(cal_dis(exper_node.odom_x,exper_node.odom_y,2.4,6.5)<1.5))
                {
                    continue;
                }
        
                exper_node.collision_tag=true;
                exper_node.collision++;
                exper_node.n_cal++;
                std::cout<< "collision!   collison times: "<<exper_node.collision<<std::endl;  
            }
        }
        
    }
   
    
}

void path_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if( exper_node.goal_begin_tag){
        double global_time=msg->data[msg->data.size()-1];
        exper_node.global_path_times++;
        //std::cout<<"glo time cost: "<<global_time<<std::endl;
        exper_node.e_global_time=(exper_node.e_global_time*(exper_node.global_path_times-1)+global_time)/exper_node.global_path_times;
    }
}
void v_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    v[exper_node.v_times]=msg->data[0];
    exper_node.v_times++;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "experiment_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    
    nh.param<int>("/expriment_node/obstacle_num", obstacle_num,0);
    nh.param<double>("/expriment_node/robot_r", car_r,0);
    nh.param<double>("/expriment_node/obstacle_radius", ob_r,0);
    nh.param<double>("/expriment_node/goal_dis_exper", goal_dis,0);
    nh.param<double>("/expriment_node/time_max_exper", time_max,0);
    
 


    
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