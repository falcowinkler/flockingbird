#include <vector>
#include "utility/vector_operations.h"

using namespace FlockSimulation;

namespace Rules {
    Vector2D seperation(Boid boidToUpdate, std::vector<Boid> proximity) {
    proximity.size();
    return boidToUpdate.velocity;
  }

  Vector2D alignment(Boid boidToUpdate, std::vector<Boid> proximity) {
      proximity.size();
      return boidToUpdate.velocity;
  }

  Vector2D cohesion(Boid boidToUpdate, std::vector<Boid> proximity) {
      proximity.size();
      Vector2D aggregatedPosition(0, 0);
      for (auto it = proximity.begin(); it != proximity.end(); it++) {
        aggregatedPosition = VectorOperations::vecSum(aggregatedPosition, it->position);
      }
      const int N = proximity.size();
      const double scalingFactor = 1.0 / 100; // One percent
      return Vector2D((aggregatedPosition.x/N)*scalingFactor,
                      (aggregatedPosition.y/N)*scalingFactor);
  }
};
