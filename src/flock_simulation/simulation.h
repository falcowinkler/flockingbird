#include "flock_simulation/rules.h"
#include "nearest_neighbors/visible_proximity.h"
#include <vector>
#include "configuration.h"


class FlockSimulation {
 private:
     FlockSimulationParameters configuration;
     std::vector<Rule*> rules;
 public:
 FlockSimulation(FlockSimulationParameters configurationIn, Flock& flockIn, std::vector<Rule*> rules):
     flock(flockIn), configuration(configurationIn), rules(rules) {}
  Flock& flock;
  void step() {
      VisibleProximity visibleProximity(flock);
      for (int i = 0; i < flock.boids.size(); i++) {
          for (int r = 0; r < rules.size(); r++) {
              Rule*             rule           = rules[r];
              std::vector<Boid> proximity = visibleProximity.of(i, configuration.visionRange);
              flock.boids[i].position = flock.boids[i].position +
                                               (*rule)(flock.boids[i], proximity, configuration);
          }
    }
  }
};
