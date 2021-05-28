#include "flock_simulation/rules.h"
#include "nearest_neighbors/visible_proximity.h"
#include <vector>
#include "configuration.h"


class FlockSimulation {
 private:
     FlockSimulationParameters configuration;
     std::vector<Rule*> rules;
     double wrap(double val, double max) { return val - max * floor(val / max); }
     Vector2D wrap(Vector2D position, double maxX, double maxY) {
        return Vector2D(wrap(position.x, maxX), wrap(position.y, maxY));
     }

 public:
 FlockSimulation(FlockSimulationParameters configurationIn, Flock& flockIn, std::vector<Rule*> rules):
     flock(flockIn), configuration(configurationIn), rules(rules) {}
  Flock& flock;
  void step() {
      VisibleProximity visibleProximity(flock);
      for (int i = 0; i < flock.boids.size(); i++) {
        Vector2D velocityUpdate(0, 0);
          for (int r = 0; r < rules.size(); r++) {
              Rule*             rule           = rules[r];
              std::vector<Boid> proximity = visibleProximity.of(i, pow(configuration.visionRange, 2));
              velocityUpdate = velocityUpdate +
                                               (*rule)(flock.boids[i], proximity, configuration);
          }
          flock.boids[i].velocity = (flock.boids[i].velocity + velocityUpdate).limit(configuration.speedLimit);
          flock.boids[i].position = flock.boids[i].position + flock.boids[i].velocity;
          if (configuration.maxX > 0 && configuration.maxY > 0) {
            flock.boids[i].position = wrap(flock.boids[i].position, configuration.maxX, configuration.maxY);
          }
    }
  }
};
