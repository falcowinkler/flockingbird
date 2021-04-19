#include "utility/random_numbers.h"
#include <vector>
/*
 * Datatype definitions for Points, boids and flocks.
 */
namespace FlockSimulation {

struct Vector2D {
    Vector2D(double xIn, double yIn)
        : x(xIn)
        , y(yIn) {}
    double x, y;
};

class Boid {
public:
    Boid(Vector2D positionIn, Vector2D velocity)
        : position(positionIn)
        , velocity(velocity) {}
    Vector2D position;
    Vector2D velocity;
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
            Boid randomBoid(Vector2D(randomInBounds(0, maxX), randomInBounds(0, maxY)),
                            Vector2D(randomInBounds(0, 1), randomInBounds(0, 1)));
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
