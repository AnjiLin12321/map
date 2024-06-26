#ifndef PLANNER_CLASS_H
#define PLANNER_CLASS_H
#include <Eigen/Dense>  
#include <algorithm> 
#include <vector>
#include <ros/ros.h>
#include <queue>
#include <functional>
#include<math.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include "common/state/state.h"
#include<thread>
#include<mutex>
namespace global
{
   extern Eigen::Vector3d start_point;
//[0 -6 t]
    extern  Eigen::Vector3d goal_point;

    extern std::vector<std::vector<common::State>>* sur_discretePoints;
    //[0 6 t]
    extern  std::mutex mtx; // 互斥锁  


    extern  int  obstacle_num;
    extern double pre_time;
    extern double deltatime;
    extern double v_mean;
    extern int nt;
    extern int skip_time;
    extern double cur_time;

    extern double robot_r;
    extern double safe_dis;
    extern int iter_max_g;

    extern double goal_dis;
    extern int interpolation_num;

    class World_G;
    class Node ;
     class AStar3D;
    const float INF= std::numeric_limits<float>::max();

    enum class STATUS : uint8_t
    {
        NONE,
        OPEN,
        CLOSED,
        OBS
    };

    class World_G{
        public:
            bool has_map_=false;
            World_G(Eigen::Vector3d &resolution);
            ~World_G();
            void initGridMap(const Eigen::Vector3d &lowerbound,const Eigen::Vector3d &upperbound);


            Eigen::Vector3i coord2index(const Eigen::Vector3d &coord)
            {
                Eigen::Vector3d resultDouble =(coord-lowerbound_).array()/resolution_.array();  
                Eigen::Vector3i index = (resultDouble.array().round()).cast<int>();  
                //Eigen::Vector3i index= ( (coord-lowerbound_).array/resolution_.array).cast<int>(); 
                index[0] = std::min(std::max(0, index[0]), idx_count_[0]);   
                index[1] = std::min(std::max(0, index[1]), idx_count_[1]);     
                index[2] = std::min(std::max(0, index[2]), idx_count_[2]);             
                return index;
            }
            Eigen::Vector3i coord2index_n(const Eigen::Vector3d &coord)
            {
                Eigen::Vector3d resultDouble =(coord-lowerbound_).array()/resolution_.array();  
                Eigen::Vector3i index = (resultDouble.array().round()).cast<int>();  
                //Eigen::Vector3i index= ( (coord-lowerbound_).array/resolution_.array).cast<int>();         
                return index;
            }

            bool isInsideBorder(const Eigen::Vector3i &index);
            bool isInsideBorder(const Eigen::Vector3d &point){return isInsideBorder(coord2index_n(point));}
            Eigen::Vector3d index2coord(const Eigen::Vector3i &index)
            {
                
                 Eigen::Vector3d coord=resolution_.array()*index.cast<double>() .array();
                coord += lowerbound_;
                //+ 0.5*resolution_.*Eigen::Vector3d::Ones();
                return coord;
            }

            void clear_gridmap(){
                for(int i=0;i < idx_count_(0);i++)
                {
                    for(int j=0;j < idx_count_(1);j++)
                    {
                        for(int k=0;k< idx_count_(2);k++){
                            if(grid_map_[i][j][k]){
                                delete grid_map_[i][j][k];
                                grid_map_[i][j][k]=nullptr;
                            }
                        }
                    }
                }
            }
            Node*  ***grid_map_=NULL;

            Eigen::Vector3d resolution_;
            Eigen::Vector3i idx_count_;
            Eigen::Vector3d lowerbound_;
            Eigen::Vector3d upperbound_;

            
    };
    class Node 
    {
    public: 
        /*
        child_node.id = child_node_ind;
        child_node.g = child_g;
        child_node.h = child_h;
        child_node.f = child_f;
        child_node.is_in_openlist = 1;
        child_node.is_in_closedlist = 0;
        child_node.parent_id = cur_ind;
        child_node.parent_operation = expansion_pattern(ii, :);
        */
        std::vector<Node*> children_;
        Node* parent_=NULL;

        Eigen::Vector3d position_;
        Eigen::Vector3i ind_;
        float g;
        float h;
        float f;
        STATUS status = STATUS::NONE;
        bool operator>(const Node& right) const
        {
            return f > right.f;
        }
        struct cmp
        {
            bool operator()(Node *&a, Node *&b) const
            {
                return a->f > b->f;//小顶堆
            }
        };

        Node(){};
        Node(const Node &node);
        //~Node();
    };

    class Path
    {
    public:
        std::vector<Node*> nodes_;
        float dis_;
        float cost_;
        enum Type{Global,Sub,Empty}type_;

        Path();
        ~Path();
    };
    class AStar3D{
        public:

             AStar3D(const int &obstacle_num,const double &pre_time,const double &deltatime,const double &v_mean,
                const int &nt,World_G* world_g,const double & work_rate):obstacle_num(obstacle_num),pre_time(pre_time),
                deltatime(deltatime),v_mean(v_mean),nt(nt),world_g_(world_g), work_rate_(work_rate),start_point_(0,0,0),goal_point_(0,0,0){}

            void planthread();

            void dy_ob_get(std::vector<std::vector<common::State>>* sur_discretePoints);

            void A3d_planner();

            bool check_no_collision(const Node & node);
             bool check_no_collision(const Eigen::Vector3i  &indx);

             bool check_close_goal(const Node & node);

             void init();
            World_G* world_g_;
            std::vector<std::vector<common::State>>* sur_discretePoints_=nullptr;
            int  obstacle_num;
            double pre_time;
            double deltatime;
            double v_mean;
            int nt;
            double work_rate_;

           double weight_for_time=0.5;
           int Nring=1;
            // mutex M_Lock;//互斥量

           Eigen::Vector3d  start_point_;
            //[0 -6 t]
           Eigen::Vector3d  goal_point_;
            //[0 6 t]

            std::priority_queue<Node*, std::vector<Node*>,Node::cmp> openlist_;


            ros::Publisher* tree_vis_pub_=NULL;
             ros::Publisher* path_vis_pub_=NULL;
              ros::Publisher* path_inter_pub_=NULL;
            void visTree();
            Path path_;

            void  generatePath( Node * node);
            void visPath(const std:: vector<Node*>& solution);
            void pubInterpolatedPath(const std::vector<Node*>& solution);

            inline float EuclideanDistance(const Eigen::VectorXd &p,const Eigen::VectorXd &q){
                Eigen::Vector2d p2,q2;
                p2[0]=p[0];
                p2[1]=p[1];
                q2[0]=q[0];
                q2[1]=q[1];
                return (p2-q2).norm();
                }
            inline float EuclideanDistance(const Node* p,const Node* q){return EuclideanDistance(p->position_,q->position_);}

    };

}  // namespace common


#endif