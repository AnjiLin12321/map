#include "../include/planner_class.h"

namespace global
{
    World::World(Eigen::Vector3d &resolution):resolution_(resolution)
    {
        lowerbound_=INF*Eigen::Vector3d::Ones();
        upperbound_=-INF*Eigen::Vector3d::Ones();
        idx_count_=Eigen::Vector3i::Zero();
    }
    World::~World()
    {
        if(has_map_)
        {
            for(int i=0;i < idx_count_(0);i++)
            {
                for(int j=0;j < idx_count_(1);j++)
                {
                    delete[] grid_map_[i][j];
                    grid_map_[i][j]=NULL;
                }
                delete[] grid_map_[i];
                grid_map_[i]=NULL;
            }
            delete[] grid_map_;
            grid_map_=NULL;
        }
    }
    void World::initGridMap(const Eigen::Vector3d &lowerbound,const Eigen::Vector3d &upperbound)
    {
        lowerbound_=lowerbound;
        upperbound_=upperbound;
        idx_count_=((upperbound_-lowerbound_)/resolution_).cast<int>()+Eigen::Vector3i::Ones();   //!!!
        grid_map_=new bool**[idx_count_(0)];
        for(int i=0;i < idx_count_(0);i++)
        {
            grid_map_[i]=new bool*[idx_count_(1)];
            for(int j=0;j < idx_count_(1);j++)
            {
                grid_map_[i][j]=new bool[idx_count_(2)];
                memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool));
            }
        }
        has_map_=true;
    }

    Node::Node(const Node &node)
    {
        children_=node.children_;
        parent_=node.parent_;
        position_=node.position_;
        ind_=node.ind_;
        g=node.g;
        h=node.h;
        f=node.f;
    }

    void 3DAStar::planthread(std::vector<std::vector<common::State>>* sur_discretePoints)
    {
        sur_discretePoints_=sur_discretePoints
        M_Lock.lock();//加锁

		M_Lock.unlock();//解锁

    }

}  //