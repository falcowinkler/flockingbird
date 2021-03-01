#pragma once
#include "nearest_neighbors/visible_proximity.h"

namespace FlockSimulation {

struct Point {
  double x, y;
};

struct Flock {
  std::vector<Point> points;

  typedef Flock Derived;
  inline size_t kdtree_get_point_count() const { return points.size(); }

  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
      if (dim == 0) return points[idx].x;
      else return points[idx].y;
  }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const {
      return false;
  }
};
}  // namespace FlockSimulation
