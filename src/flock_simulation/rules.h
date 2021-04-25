#include <iostream>
#include <vector>
#include "utility/vector_operations.h"

using namespace FlockSimulation;
using namespace VectorOperations;

namespace Rules {
  // Boids try to keep a small distance away from other objects (including other boids).
  Vector2D seperation(Boid boidToUpdate, std::vector<Boid> closeProximity) {
    Vector2D c = Vector2D(0, 0);
    for (auto it = closeProximity.begin(); it!=closeProximity.end(); it++) {
      c = vecDiff(c, vecDiff(boidToUpdate.position, it->position));
    }
    return c;
  }

  Vector2D alignment(Boid boidToUpdate, std::vector<Boid> proximity) {
      proximity.size();
      return boidToUpdate.velocity;
  }

  // Boids try to fly towards the centre of mass of neighbouring boids.
  Vector2D cohesion(Boid boidToUpdate, std::vector<Boid> proximity) {
      proximity.size();
      Vector2D aggregatedPosition(0, 0);
      for (auto it = proximity.begin(); it != proximity.end(); it++) {
        aggregatedPosition = vecSum(aggregatedPosition, it->position);
      }
      const int N = proximity.size();
      const Vector2D averagedPosition = vecMulScalar(aggregatedPosition, 1.0/N);
      const Vector2D diff = vecDiff(averagedPosition, boidToUpdate.position);
      const double scalingFactor = 1.0 / 100; // One percent
      return vecMulScalar(diff, scalingFactor);
  }
};
