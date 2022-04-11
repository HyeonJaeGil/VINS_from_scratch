#pragma once

#include <vector>

#include "Eigen/Core"
#include "sensor/point.h"

namespace VINS{
namespace sensor {



class PointCloud {

 public:
  using PointType = Point;

  PointCloud();
  explicit PointCloud(std::vector<PointType> points);
  PointCloud(std::vector<PointType> points, std::vector<float> intensities);

  Eigen::Vector3f position;

  // Returns the number of points in the point cloud.
  size_t size() const;
  // Checks whether there are any points in the point cloud.
  bool empty() const;

  const std::vector<PointType>& points() const;
  const std::vector<float>& intensities() const;
  const PointType& operator[](const size_t index) const;

  // Iterator over the points in the point cloud.
  using ConstIterator = std::vector<PointType>::const_iterator;
  ConstIterator begin() const;
  ConstIterator end() const;

  void push_back(PointType value);

  // Creates a PointCloud consisting of all the points for which `predicate`
  // returns true, together with the corresponding intensities.
  template <class UnaryPredicate>
  PointCloud copy_if(UnaryPredicate predicate) const {
    std::vector<PointType> points;
    std::vector<float> intensities;

    // Note: benchmarks show that it is better to have this conditional outside
    // the loop.
    if (intensities_.empty()) {
      for (size_t index = 0; index < size(); ++index) {
        const PointType& point = points_[index];
        if (predicate(point)) {
          points.push_back(point);
        }
      }
    } else {
      for (size_t index = 0; index < size(); ++index) {
        const PointType& point = points_[index];
        if (predicate(point)) {
          points.push_back(point);
          intensities.push_back(intensities_[index]);
        }
      }
    }

    return PointCloud(points, intensities);
  }
  
 private:
  // For 2D points, the third entry is 0.f.
  std::vector<PointType> points_;
  // Intensities are optional. If non-empty, they must have the same size as
  // points.
  std::vector<float> intensities_;



};


// PointCloud TransformPointCloud(const PointCloud& point_cloud,
//                                const transform::Rigid3f& transform);

} // sensor
} // VINS