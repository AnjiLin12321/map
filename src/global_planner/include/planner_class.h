#ifndef PLANNER_CLASS_H
#define PLANNER_CLASS_H
#include <Eigen/Dense>  
#include <algorithm> 
#include <vector>
#include "common/state/state.h"
#include<thread>
#include<mutex>
namespace global
{
    const float INF= std::numeric_limits<float>::max();
    class World{
        public:
            bool has_map_=false;
            World(Eigen::Vector3d &resolution);
            ~World();
            void World::initGridMap(const Eigen::Vector3d &lowerbound,const Eigen::Vector3d &upperbound);


            Eigen::Vector3i coord2index(const Eigen::Vector3d &coord)
            {
                Eigen::Vector3i index = ( (coord-lowerbound_)/resolution_).cast<int>(); 
                index[0] = std::min(std::max(0, index[0]), idx_count_[0]);   
                index[1] = std::min(std::max(0, index[1]), idx_count_[1]);     
                index[2] = std::min(std::max(0, index[2]), idx_count_[2]);             
                return index;
            }
            bool ***grid_map_=NULL;

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


        //Node();
        Node(const Node &node);
        //~Node();
    };
    class 3DAStar{
        public:

            PFRRTStar(const int &obstacle_num,const double &pre_time,const double &deltatime,const double &v_mean,
                const int &nt,World* world):obstacle_num(obstacle_num),pre_time(pre_time),
                deltatime(deltatime),v_mean(v_mean),nt(nt),world_(world){}

            void planthread(std::vector<std::vector<common::State>>* sur_discretePoints);

            World* world_;
            std::vector<std::vector<common::State>>* sur_discretePoints_=nullptr;; 
            int  obstacle_num;
            double pre_time;
            double deltatime;
            double v_mean;
            int nt;
            mutex M_Lock;//互斥量

    };

}  // namespace common


#endif