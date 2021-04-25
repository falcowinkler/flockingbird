#include "flock_simulation/flock.h"

using namespace FlockSimulation;

namespace VectorOperations {

  inline Vector2D vecSum(Vector2D a, Vector2D b) {
    return Vector2D(a.x + b.x, a.y+b.y);
  }
}
