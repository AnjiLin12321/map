#ifndef PUTN_CLASSES_H
#define PUTN_CLASSES_H

#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

const float INF= std::numeric_limits<float>::max();
const float PI = 3.14151f;

class World;
class World
{
public:
    
    bool has_map_=false;

    World(const float &resolution=0.1f);
    ~World();
    void visWorld(ros::Publisher* world_vis_pub);
    /**
     * @brief Automatically determine the upperbound and lowerbound of the grid map according to the
     *        information of the input point cloud.
     * @param pcl::PointCloud<pcl::PointXYZ> point cloud input
     * @return void
     */
    void initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud);

    /**
     * @brief Manually specify the upperbound and lowerbound.
     * @param Vector3d
     * @param Vector3d
     * @return void
     */
    void setObs(const Eigen::Vector3d &point);
    Eigen::Vector3d index2coord(const Eigen::Vector3i &index)
    {
        Eigen::Vector3d coord = resolution_*index.cast<double>() + lowerbound_+ 0.5*resolution_*Eigen::Vector3d::Ones();
        return coord;
    }
   Eigen::Vector3i coord2index(const Eigen::Vector3d &coord)
    {
        Eigen::Vector3i index = ( (coord-lowerbound_)/resolution_).cast<int>();            
        return index;
    }
//protected:
    bool ***grid_map_=NULL;

    float resolution_;

    Eigen::Vector3i idx_count_;

    Eigen::Vector3d lowerbound_;
    Eigen::Vector3d upperbound_;

    void clearMap();
};

#endif