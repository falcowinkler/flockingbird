#pragma once

#include "../utility/random_numbers.hpp"
#include "../utility/vector_operations.hpp"
#include "boid.hpp"

namespace flockingbird {

class Flock {
public:
    // constructors
    Flock(const flockingbird::Flock& other) {
        for (auto it = other.boids.begin(); it != other.boids.end(); it++) {
            boids.push_back(flockingbird::Boid(*it));
        }
    }
    Flock() {
        std::vector<flockingbird::Boid> emptyBoids;
        boids = emptyBoids;
    }
    Flock(std::vector<flockingbird::Boid> boids) { boids = boids; }
    Flock(int numBoids, int maxX, int maxY) {
        std::vector<flockingbird::Boid> result;
        for (int i = 0; i < numBoids; i++) {
            flockingbird::Boid randomBoid(Vector2D(randomInBounds(0, maxX),
                                                   randomInBounds(0, maxY)),
                                          Vector2D(randomInBounds(-1, 1), randomInBounds(-1, 1)));
            result.push_back(randomBoid);
        }
        boids = result;
    }

    // members
    std::vector<flockingbird::Boid> boids;

    // nanoflann API
    inline size_t kdtree_get_point_count() const { return boids.size(); }

    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
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
}  // namespace flockingbird
