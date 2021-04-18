#include "utility/random_numbers.h"
#include <vector>
/*
 * Datatype definitions for Points, boids and flocks.
 */
namespace FlockSimulation {

class Point {
public:
  Point(){}
  Point(double xCoord, double yCoord): x(xCoord), y(yCoord) {
  }
  double x, y;
};


class Boid {
public:
  Boid() {}
  Boid(Point positionIn, std::vector<double> velocity) :
    position(positionIn),
    velocity(velocity) {
  }
  Point position;
  std::vector<double> velocity;
};

class Flock {
public:
    Flock() {
        std::vector<Boid> emptyBoids;
        boids = emptyBoids;
    }
      Flock(int numBoids, int maxX, int maxY) {
      std::vector<Boid> result;
      for (int i = 0; i < numBoids; i++) {
        Boid randomBoid(Point(randomInBounds(0, maxX), randomInBounds(0, maxY)),
                        std::vector<double> {randomInBounds(0, 1), randomInBounds(0, 1)}
                        );
        result.push_back(randomBoid);
      }
      boids = result;
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
