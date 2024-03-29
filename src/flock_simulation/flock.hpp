#pragma once

#include "../utility/random_numbers.hpp"
#include "../utility/vector_operations.hpp"
#include "boid.hpp"
#include "configuration.hpp"
namespace flockingbird {

class Flock {
public:
    flockingbird::FlockSimulationParameters configuration;
    // constructors
    Flock() {
        std::vector<flockingbird::Boid> emptyBoids;
        boids = emptyBoids;
    }
    Flock(std::vector<flockingbird::Boid> boids)
        : boids(boids) {}
    Flock(int numBoids, float maxX, float maxY, float maxZ) {
        std::vector<flockingbird::Boid> result;
        for (int i = 0; i < numBoids; i++) {

            if (configuration.twoD) {
                flockingbird::Boid randomBoid
                    = flockingbird::Boid(Vector3D(randomInBounds(0+configuration.avoidanceRadius*3, maxX-configuration.avoidanceRadius*3),
                                                  randomInBounds(0+configuration.avoidanceRadius*3, maxY-configuration.avoidanceRadius*3),
                                                  0),
                                         Vector3D(randomInBounds(-1, 1), randomInBounds(-1, 1), 0));
                result.push_back(randomBoid);
            } else {
                flockingbird::Boid randomBoid
                    = flockingbird::Boid(Vector3D(randomInBounds(0+configuration.avoidanceRadius*3, maxX-configuration.avoidanceRadius*3),
                                                  randomInBounds(0+configuration.avoidanceRadius*3, maxY-configuration.avoidanceRadius*3),
                                                  randomInBounds(0+configuration.avoidanceRadius*3, maxZ-configuration.avoidanceRadius*3)),
                                         Vector3D(randomInBounds(-1, 1),
                                                  randomInBounds(-1, 1),
                                                  randomInBounds(-1, 1)));
                result.push_back(randomBoid);
            }
        }
        boids = result;
    }

    // members
    std::vector<flockingbird::Boid> boids;

    // nanoflann API
    inline size_t kdtree_get_point_count() const { return boids.size(); }

    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (configuration.twoD) {
            if (dim == 0)
                return boids[idx].position.x;
            else if (dim == 1)
                return boids[idx].position.y;
            else{
                return boids[idx].position.y;
            }
        } else {
            if (dim == 0)
                return boids[idx].position.x;
            else if (dim == 1)
                return boids[idx].position.y;
            else
                return boids[idx].position.z;
        }
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const {
        return false;
    }
};
}  // namespace flockingbird
