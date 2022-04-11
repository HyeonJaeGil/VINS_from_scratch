#pragma once
#include "Eigen/Core"
#include "common/time.h"

namespace VINS{
namespace sensor {

struct ImuData {
  common::Time time;
  Eigen::Vector3d linear_acceleration;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d accleration_bias;
  Eigen::Vector3d gyroscope_bias;
};



} // sensor
} // VINS