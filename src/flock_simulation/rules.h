#include <iostream>
#include <vector>
#include "utility/vector_operations.h"

using namespace FlockSimulation;
using namespace VectorOperations;

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
        aggregatedPosition = vecSum(aggregatedPosition, it->position);
      }
      const int N = proximity.size();
      const Vector2D averagedPosition = vecMulScalar(aggregatedPosition, 1.0/N);
      const Vector2D diff = vecDiff(averagedPosition, boidToUpdate.position);
      std::cout << "diff: " << diff << std::endl;
      const double scalingFactor = 1.0 / 100; // One percent
      return vecMulScalar(diff, scalingFactor);
  }
};
