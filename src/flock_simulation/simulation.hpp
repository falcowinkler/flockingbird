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

    FlockSimulation() = default;
    FlockSimulation(flockingbird::FlockSimulationParameters configurationIn,
                    flockingbird::Flock&                    flockIn,
                    std::vector<Rule*>                      rules)
        : configuration(configurationIn)
        , rules(rules)
        , flock(flockIn) {}

    // A simplistic implementation of operator= (see better implementation below)
    /*FlockSimulation& FlockSimulation::operator=(const FlockSimulation& flockSimIn) {
         do the copy
        configuration = flockSimIn.configuration;
        flock         = flockSimIn.flock;
        rules         = flockSimIn.rules;
         return the existing object so we can chain this operator
        return *this;
    }*/

    //variable to declare amount of rays to calculate
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

            Boid* boid = &flock.boids[i];
            //Two or three dimensions
            if (configuration.twoD) {
                boid->position.z = 0;
                boid->velocity.z = 0;
                acceleration.z   = 0;
            }

            Vector3D targetAcceleration = {0, 0, 0};
            if (configuration.targetPosition.x != -1 && configuration.targetPosition.y != -1
                && configuration.targetPosition.z != -1) {
                Vector3D offsetToTarget = (configuration.targetPosition - boid->position);
                targetAcceleration
                    = offsetToTarget.normalized() * configuration.speedLimit - boid->velocity;
                targetAcceleration = targetAcceleration.limit(configuration.forceLimit)
                    * configuration.directionWeight;
            }
            acceleration = acceleration + targetAcceleration;

            boid->velocity = boid->velocity + acceleration * dt;
            float    speed = boid->velocity.magnitude();
            Vector3D dir   = boid->velocity.normalized();
            speed          = std::clamp(speed, 2.0f, configuration.speedLimit);
            boid->velocity = dir * speed;

            boid->position = boid->position + boid->velocity;

            // Check if next update is out of bound 
            boid->position
                = wrap(boid->position, configuration.maxX, configuration.maxY, configuration.maxZ);
        }
    }

    std::vector<Vector3D> getDirections() { return dirs; }
};
}  // namespace flockingbird
