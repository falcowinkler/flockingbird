#include "flock_simulation/rules.h"
#include "nearest_neighbors/visible_proximity.h"
#include <vector>

using namespace FlockSimulation;
using namespace VectorOperations;

const double SPEED_LIMIT                       = 500;
const double POSITION_INCREMENT_SCALING_FACTOR = 2.0;
// ranges in squared euclidean distance
const double AVOIDANCE_RADIUS = 1000.0;
const double VISION_RANGE     = 10000.0;

struct FlockSimulationParameters {
  double speedLimit;
  double positionIncrementScalingFactor;
  double avoidanceRadius;
  double visionRange;
FlockSimulationParameters(double speedLimitIn,
                          double positionIncrementScalingFactorIn,
                          double avoidanceRadiusIn,
                          double visionRangeIn)
  : speedLimit(speedLimitIn),
    positionIncrementScalingFactor(positionIncrementScalingFactorIn),
    avoidanceRadius(avoidanceRadiusIn),
    visionRange(visionRangeIn){}
};

class Simulation {
 private:
     FlockSimulationParameters configuration;
     std::vector<Rule*> rules;
 public:
 Simulation(FlockSimulationParameters configurationIn, Flock& flockIn, std::vector<Rule*> rules):
     flock(flockIn), configuration(configurationIn), rules(rules) {}
  Flock& flock;
  void step() {
      VisibleProximity visibleProximity(flock);
      for (int i = 0; i < flock.boids.size(); i++) {
          for (int r = 0; r < rules.size(); r++) {
              Rule*             rule           = rules[r];
              std::vector<Boid> proximity = visibleProximity.of(i, configuration.visionRange);
              std::vector<Boid> closeProximity = visibleProximity.of(i, configuration.avoidanceRadius);
              flock.boids[i].position = flock.boids[i].position +
                                               (*rule)(flock.boids[i], proximity, closeProximity);
          }
    }
  }
};

inline void step(Flock &flock) {
   VisibleProximity visibleProximity(flock);
   std::vector<Vector2D> velocityCorrections;
   std::vector<Vector2D> positionUpdates;
    for (int i = 0; i < flock.boids.size(); i++) {
        Boid              boidToUpdate        = flock.boids[i];
        std::vector<Boid> proximity      = visibleProximity.of(i, VISION_RANGE);
        std::vector<Boid> closeProximity      = visibleProximity.of(i, AVOIDANCE_RADIUS);

        Vector2D          cohesion       = Rules::cohesion(boidToUpdate, proximity);
        Vector2D          separation     = Rules::seperation(boidToUpdate, closeProximity);
        Vector2D          alignment      = Rules::alignment(boidToUpdate, proximity);
        Vector2D currentVelocity         = flock.boids[i].velocity;

        // std::cout << "cohesion: " << cohesion << std::endl;
        // std::cout << "separation: " << separation << std::endl;
        // std::cout << "alignment: " << alignment << std::endl;

        Vector2D          velocityCorrection
            = boidToUpdate.velocity + cohesion + separation + alignment;

        /*  if (magnitude(velocityCorrection) > SPEED_LIMIT) {
            velocityCorrection
                = vecMulScalar(velocityCorrection, SPEED_LIMIT / magnitude(velocityCorrection));
                } */
        // Add wind?
        //  velocityCorrection = vecSum(velocityCorrection, Vector2D(-1, -1));
        velocityCorrections.push_back(velocityCorrection);
        Vector2D scaledVelocity = velocityCorrection * POSITION_INCREMENT_SCALING_FACTOR;
        positionUpdates.push_back(flock.boids[i].position + scaledVelocity);
    }
    for (int i = 0; i < flock.boids.size(); i++) {

      flock.boids[i].velocity = velocityCorrections[i].normalized();
        flock.boids[i].position = positionUpdates[i];
    }
}
