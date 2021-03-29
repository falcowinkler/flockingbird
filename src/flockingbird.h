#pragma once
#include "nearest_neighbors/visible_proximity.h"

namespace FlockSimulation {

class Point {
public:
  double x, y;
};


class Boid {
public:
  Point position;
  double bearing;
};

class Flock {
public:
    Flock() {
        std::vector<Boid> emptyBoids;
        boids = emptyBoids;
    }
    Flock(int numBoids) {
        // TODO: random init
      std::vector<Boid> emptyBoids;
      boids = emptyBoids;
    }

    std::vector<Boid> boids;

    typedef Flock Derived;
    inline size_t kdtree_get_point_count() const { return boids.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0)
            return boids[idx].position.x;
        else
            return boids[idx].position.y;
  }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const {
      return false;
  }
};
}  // namespace FlockSimulation
