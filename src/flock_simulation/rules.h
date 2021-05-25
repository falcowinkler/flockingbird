#include "boid.h"
#include "utility/vector_operations.h"
#include <iostream>
#include <vector>
#pragma once

using namespace VectorOperations;

class Rule {
public:
    virtual Vector2D
    operator()(Boid boidToUpdate, std::vector<Boid> proximity, std::vector<Boid> closeProximity)
        = 0;
};


namespace Rules {
const double maxForce = 0.06;
// Boids try to keep a small distance away from other objects (including other boids).
inline Vector2D seperation(Boid boidToUpdate, std::vector<Boid> closeProximity) {
    Vector2D c = Vector2D(0, 0);
    if (closeProximity.empty()) {
      return c;
    }
    for (auto it = closeProximity.begin(); it != closeProximity.end(); it++) {
      c = c - (it->position - boidToUpdate.position);
    }
    return steer(c.normalized(), boidToUpdate.velocity, maxForce);
}

// Boids try to match velocity with near boids.
inline Vector2D alignment(Boid boidToUpdate, std::vector<Boid> proximity) {
    if (proximity.empty()) {
        return Vector2D(0, 0);
    }

    Vector2D aggregatedVelocity(0, 0);
    for (auto it = proximity.begin(); it != proximity.end(); it++) {
        aggregatedVelocity = aggregatedVelocity + it->velocity;
    }
    Vector2D     averageVelocity = aggregatedVelocity * (1.0 / proximity.size());
    Vector2D     diff            = averageVelocity - boidToUpdate.velocity;
    const double scalingFactor   = 1.0/8;
    Vector2D alignment = diff * scalingFactor;
    return steer(alignment.normalized(), boidToUpdate.velocity, maxForce);
}

// Boids try to fly towards the centre of mass of neighbouring boids.
inline Vector2D cohesion(Boid boidToUpdate, std::vector<Boid> proximity) {
    if (proximity.empty()) {
        return Vector2D(0, 0);
    }

    proximity.size();
    Vector2D aggregatedPosition(0, 0);
    for (auto it = proximity.begin(); it != proximity.end(); it++) {
        aggregatedPosition = aggregatedPosition + it->position;
    }
    const Vector2D averagedPosition = aggregatedPosition * (1.0 / proximity.size());
    const Vector2D diff             = averagedPosition - boidToUpdate.position;
    const double   scalingFactor    = 1.0;
    Vector2D cohesion = diff * scalingFactor;
    return steer(cohesion.normalized(), boidToUpdate.velocity, maxForce);
}
};  // namespace Rules
