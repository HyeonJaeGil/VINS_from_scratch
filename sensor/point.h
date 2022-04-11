#pragma once

#include "Eigen/Core"
// #include "transform/transform.h"

namespace VINS{
namespace sensor {



struct Point {
  Eigen::Vector3f position;
};


struct TimedPoint {
  Eigen::Vector3f position;
  float time;
};

template <class T>
inline Point operator*(const transform::Rigid3<T>& lhs,
                                  const Point& rhs) {
  Point result = rhs;
  result.position = lhs * rhs.position;
  return result;
}

template <class T>
inline Point operator*(const transform::Rigid3<T>& lhs,
                                       const Point& rhs) {
  TimedPoint result = rhs;
  result.position = lhs * rhs.position;
  return result;
}

inline bool operator==(const Point& lhs,
                       const Point& rhs) {
  return lhs.position == rhs.position;
}

inline bool operator==(const Point& lhs,
                       const Point& rhs) {
  return lhs.position == rhs.position && lhs.time == rhs.time;
}


} // sensor
} // VINS