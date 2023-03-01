#include "../nearest_neighbors/visible_proximity.hpp"
#include "configuration.hpp"
#include "rules.hpp"
#include <vector>

namespace flockingbird {

class FlockSimulation {
private:
    flockingbird::FlockSimulationParameters configuration;
    std::vector<Rule*>                      rules;
    std::vector<Vector3D>                   viewDirections;
    std::vector<Vector3D>                   dirs;

    float wrap(float val, float max) {
        double newval = (double)val;
        double newMax = (double)max;
        if (val >= max) {
            return val - max + configuration.avoidanceRadius;
        }
        if (val < 0) {
            return val + max - configuration.avoidanceRadius;
        } else

        {
            return val;
        }
    }

    Vector3D wrap(Vector3D position, float maxX, float maxY, float maxZ) {
        return Vector3D(wrap(position.x, maxX), wrap(position.y, maxY), wrap(position.z, maxZ));
    }

public:
    flockingbird::Flock& flock;
    FlockSimulation(flockingbird::FlockSimulationParameters configurationIn,
                    flockingbird::Flock&                    flockIn,
                    std::vector<Rule*>                      rules)
        : configuration(configurationIn)
        , rules(rules)
        , flock(flockIn) {}

    const int numViewDirections = 300;

    // Simulation function
    void step(float dt) {
        VisibleProximity visibleProximity(flock);
        for (auto it = flock.boids.begin(); it != flock.boids.end(); it++) {
            int i = std::distance(flock.boids.begin(), it);

            Vector3D acceleration(0, 0, 0);

            for (Rule* rule : rules) {
                std::vector<flockingbird::Boid> proximity
                    = visibleProximity.of(i, pow(configuration.visionRange, 2));
                acceleration = acceleration + (*rule)(flock.boids[i], proximity, configuration);
            }

            Boid* boid     = &flock.boids[i];
            boid->velocity = boid->velocity + acceleration * dt;
            float    speed = boid->velocity.magnitude();
            Vector3D dir   = boid->velocity.normalized();
            speed          = std::clamp(speed, 2.0f, configuration.speedLimit);
            boid->velocity = dir * speed;

            boid->position   = boid->position + boid->velocity;

           //   std::cout << "postion :" << boid->position << std::endl;
            // Check if next update is out of bound //replace with collision detection
            // boid->position = wrap(boid->position, configuration.maxX, configuration.maxY,
            // configuration.maxZ);
        }
    }

    std::vector<Vector3D> getDirections() { return dirs; }
};
}  // namespace flockingbird
