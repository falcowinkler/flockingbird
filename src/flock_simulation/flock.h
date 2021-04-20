#include "utility/random_numbers.h"
#include <vector>

/*
 * Datatype definitions for Points, boids and flocks.
 */
namespace FlockSimulation {

struct Vector2D {
    Vector2D(const Vector2D& other)
        : x(other.x)
        , y(other.y) {}
    Vector2D(double xIn, double yIn)
        : x(xIn)
        , y(yIn) {}
    double x, y;

    friend std::ostream& operator<<(std::ostream& outputStream, const Vector2D& p);
};

inline std::ostream& operator<<(std::ostream& outputStream, const Vector2D& p) {
  outputStream << "[" << p.x << ", " << p.y << "]";
    return outputStream;
}

class Boid {
public:
    Boid(const Boid& other)
        : velocity(Vector2D(other.velocity))
        , position(Vector2D(other.position)) {}
    Boid(Vector2D positionIn, Vector2D velocity)
        : position(positionIn)
        , velocity(velocity) {}
    Vector2D position;
    Vector2D velocity;

  friend std::ostream& operator<<(std::ostream& outputStream, const Boid& p);
};

inline std::ostream& operator<<(std::ostream& outputStream, const Boid& p) {
    outputStream << "(pos" << p.position << ") (dir: " << p.velocity << ")";
    return outputStream;
}

class Flock {
public:
    Flock(const Flock& other) {
        for (auto it = other.boids.begin(); it != other.boids.end(); it++) {
            boids.push_back(Boid(*it));
        }
    }
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
