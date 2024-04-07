#include "../include/map_class.h"
using namespace Eigen;

World::World(const float &resolution):resolution_(resolution)
{
    lowerbound_=INF*Vector3d::Ones();
    upperbound_=-INF*Vector3d::Ones();
    idx_count_=Vector3i::Zero();
}

World::~World(){clearMap();}

void World::clearMap()
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
void World::initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{   
    if(cloud.points.empty())
    {
        ROS_ERROR("Can not initialize the map with an empty point cloud!");
        return;
    }
    clearMap();

    for(const auto&pt:cloud.points)
    {
        if(pt.x < lowerbound_(0)) lowerbound_(0)=pt.x;
        if(pt.y < lowerbound_(1)) lowerbound_(1)=pt.y;
        if(pt.z < lowerbound_(2)) lowerbound_(2)=pt.z;
        if(pt.x > upperbound_(0)) upperbound_(0)=pt.x;
        if(pt.y > upperbound_(1)) upperbound_(1)=pt.y;
        if(pt.z + 1.0 > upperbound_(2)) upperbound_(2)=pt.z+1.0;
    }

    idx_count_ = ((upperbound_-lowerbound_)/resolution_).cast<int>() + Eigen::Vector3i::Ones();

    grid_map_=new bool**[idx_count_(0)];
    for(int i = 0 ; i < idx_count_(0) ; i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j = 0 ; j < idx_count_(1) ; j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool));
        }
    }
    has_map_=true;
}
void World::setObs(const Vector3d &point)
{   
    Vector3i idx=coord2index(point);
    grid_map_[idx(0)][idx(1)][idx(2)]=false;
}
void World::visWorld( ros::Publisher* world_vis_pub)
{
  if ( !has_map_)
    return;
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  for (int i = 0; i < idx_count_(0); i++)
  {
    for (int j = 0; j < idx_count_(1); j++)
    {
      for (int k = 0; k < idx_count_(2); k++)
      {
        Vector3i index(i, j, k);
        if (!grid_map_[index(0)][index(1)][index(2)])
        {
          Vector3d coor_round =index2coord(index);
          pcl::PointXYZ pt_add;
          pt_add.x = coor_round(0);
          pt_add.y = coor_round(1);
          pt_add.z = coor_round(2);
          cloud_vis.points.push_back(pt_add);
        }
      }
    }
  }

 cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;

  sensor_msgs::PointCloud2 map_vis;
  pcl::toROSMsg(cloud_vis, map_vis);

  map_vis.header.frame_id = "/map";
  world_vis_pub->publish(map_vis);
}