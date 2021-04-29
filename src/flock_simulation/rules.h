#include "utility/vector_operations.h"
#include <iostream>
#include <vector>

using namespace FlockSimulation;
using namespace VectorOperations;

namespace Rules {
// Boids try to keep a small distance away from other objects (including other boids).
inline Vector2D seperation(Boid boidToUpdate, std::vector<Boid> closeProximity) {
    Vector2D c = Vector2D(0, 0);
    for (auto it = closeProximity.begin(); it != closeProximity.end(); it++) {
        c = vecDiff(c, vecDiff(boidToUpdate.position, it->position));
    }
    return c;
}

// Boids try to match velocity with near boids.
inline Vector2D alignment(Boid boidToUpdate, std::vector<Boid> proximity) {
    if (proximity.empty()) {
        return Vector2D(0, 0);
    }

    Vector2D aggregatedVelocity(0, 0);
    for (auto it = proximity.begin(); it != proximity.end(); it++) {
        aggregatedVelocity = vecSum(aggregatedVelocity, it->velocity);
    }
    Vector2D     averageVelocity = vecMulScalar(aggregatedVelocity, 1.0 / proximity.size());
    Vector2D     diff            = vecDiff(averageVelocity, boidToUpdate.velocity);
    const double scalingFactor   = 1.0 / 8;
    return vecMulScalar(diff, scalingFactor);
}

// Boids try to fly towards the centre of mass of neighbouring boids.
inline Vector2D cohesion(Boid boidToUpdate, std::vector<Boid> proximity) {
    if (proximity.empty()) {
        return Vector2D(0, 0);
    }

    proximity.size();
    Vector2D aggregatedPosition(0, 0);
    for (auto it = proximity.begin(); it != proximity.end(); it++) {
        aggregatedPosition = vecSum(aggregatedPosition, it->position);
    }
    const Vector2D averagedPosition = vecMulScalar(aggregatedPosition, 1.0 / proximity.size());
    const Vector2D diff             = vecDiff(averagedPosition, boidToUpdate.position);
    const double   scalingFactor    = 1.0 / 100;  // One percent
    return vecMulScalar(diff, scalingFactor);
}
};  // namespace Rules
