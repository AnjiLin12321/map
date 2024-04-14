#ifndef _COMMON_INC_COMMON_STATE_STATE_H__
#define _COMMON_INC_COMMON_STATE_STATE_H__

#include <Eigen/Core>
namespace common {
struct State {
  double time_stamp{0.0};
  Eigen::Matrix<double, 2, 1> vec_position{Eigen::Matrix<double, 2, 1> ::Zero()};
  double angle{0.0};  // heading angle
  //double curvature{0.0};
  Eigen::Matrix<double, 2, 1> vec_velocity{Eigen::Matrix<double, 2, 1> ::Zero()};
  //double velocity{0.0};
  //double acceleration{0.0};
  //double steer{0.0};  // steering angle
  void print() const {
    printf("State:\n");
    printf(" -- time_stamp: %lf.\n", time_stamp);
    printf(" -- vec_position: (%lf, %lf).\n", vec_position[0], vec_position[1]);
    printf(" -- angle: %lf.\n", angle);
    //printf(" -- curvature: %lf.\n", curvature);
    printf(" -- vec_velocity:  (%lf, %lf).\n", vec_velocity[0], vec_velocity[1]);
    //printf(" -- acceleration: %lf.\n", acceleration);
    //printf(" -- steer: %lf.\n", steer);
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};



}  // namespace common

#endif  // _COMMON_INC_COMMON_STATE_STATE_H__