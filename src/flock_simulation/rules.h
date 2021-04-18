#include "flock.h"
#include <vector>


using namespace FlockSimulation;

class Rules {
  static std::vector<double> seperation(Boid boidToUpdate, std::vector<Boid> proximity) {
    proximity.size();
    return boidToUpdate.velocity;
  }

  static std::vector<double> alignment(Boid boidToUpdate, std::vector<Boid> proximity) {
      proximity.size();
      return boidToUpdate.velocity;
  }

  static std::vector<double> cohesion(Boid boidToUpdate, std::vector<Boid> proximity) {
      proximity.size();
      return boidToUpdate.velocity;
  }
};
