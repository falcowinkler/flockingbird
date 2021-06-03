#include "rules.hpp"
#include "../nearest_neighbors/visible_proximity.hpp"
#include <vector>
#include "configuration.hpp"

namespace flockingbird {

class FlockSimulation {
private:
    flockingbird::Flock&                    flock;
    flockingbird::FlockSimulationParameters configuration;
    std::vector<Rule*>                      rules;


    float   wrap(float val, float max) { return val - max * floor(val / max); }
    Vector2D wrap(Vector2D position, float maxX, float maxY) {
        return Vector2D(wrap(position.x, maxX), wrap(position.y, maxY));
     }

 public:
     FlockSimulation(flockingbird::FlockSimulationParameters configurationIn,
                     flockingbird::Flock&                    flockIn,
                     std::vector<Rule*>                      rules)
         : flock(flockIn)
         , configuration(configurationIn)
         , rules(rules) {}

     void                 step() {
         VisibleProximity visibleProximity(flock);
         for (auto it = flock.boids.begin(); it != flock.boids.end(); it++) {
             int i = std::distance(flock.boids.begin(), it);
             Vector2D velocityUpdate(0, 0);
             for (Rule* rule : rules) {
                 std::vector<flockingbird::Boid> proximity
                     = visibleProximity.of(i, pow(configuration.visionRange, 2));
                 velocityUpdate
                     = velocityUpdate + (*rule)(flock.boids[i], proximity, configuration);
             }
             Boid * boid = &flock.boids[i];
             boid->velocity
                 = (boid->velocity + velocityUpdate).limit(configuration.speedLimit);
             boid->position = boid->position + boid->velocity;
             if (configuration.maxX > 0 && configuration.maxY > 0) {
                 boid->position
                     = wrap(boid->position, configuration.maxX, configuration.maxY);
             }
         }
  }
};
}
