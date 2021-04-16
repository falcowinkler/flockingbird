#include "utility/random_numbers.h"
/*
 * Datatype definitions for Points, boids and flocks.
 */
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
      Flock(int numBoids, int maxX, int maxY) {
      std::vector<Boid> emptyBoids;
      for (int i = 0; i < numBoids; i++) {
        Boid randomBoid;
        randomBoid.bearing    = randomInBounds(0, 360);
        randomBoid.position.x = randomInBounds(0, maxX);
        randomBoid.position.y = randomInBounds(0, maxY);
        emptyBoids.push_back(randomBoid);
      }
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
