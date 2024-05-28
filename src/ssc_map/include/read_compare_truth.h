#ifndef PLANNER_CLASS_H
#define PLANNER_CLASS_H
#include <Eigen/Dense>  
#include <vector>
#include <fstream>  
#include <sstream>  
#include <iostream>
#include <tuple>  
#include <unistd.h>
extern std::vector<Eigen::Vector2d> ped_0;
extern std::vector<Eigen::Vector2d> ped_1;
extern std::vector<Eigen::Vector2d> ped_2;
extern std::vector<Eigen::Vector2d> ped_3;
extern std::vector<Eigen::Vector2d> ped_4;

extern int time_index;
int read_truth_path(int ped_num=5); 

#endif