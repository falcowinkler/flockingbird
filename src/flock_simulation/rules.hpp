#include "../utility/vector_operations.hpp"
#include "boid.hpp"
#include "configuration.hpp"
#include <iostream>
#include <vector>
#pragma once

class Rule {
public:
    virtual Vector3D operator()(flockingbird::Boid                      boidToUpdate,
                                std::vector<flockingbird::Boid>         proximity,
                                flockingbird::FlockSimulationParameters configuration)
        = 0;
};

class SeparationRule : public Rule {
public:
    virtual Vector3D operator()(flockingbird::Boid                      boidToUpdate,
                                std::vector<flockingbird::Boid>         proximity,
                                flockingbird::FlockSimulationParameters configuration) {
        int      count = 0;
        Vector3D steer(0, 0, 0);

        for (flockingbird::Boid boid : proximity) {
            float d = boidToUpdate.position.distanceTo(boid.position);
            if (d > 0 && d < configuration.avoidanceRadius) {
                Vector3D diff = (boidToUpdate.position - boid.position).normalized() / d;
                steer         = steer + diff;
                count += 1;
            }
        }
        if (count > 0) {
            steer = steer / count;
        }
        if (steer.magnitude() > 0) {
            steer = steer.normalized() * configuration.speedLimit - boidToUpdate.velocity;

            return steer.limit(configuration.forceLimit) * configuration.separationWeight;
        }
        return steer;
    };
};

class AlignmentRule : public Rule {
public:
    virtual Vector3D operator()(flockingbird::Boid                      boidToUpdate,
                                std::vector<flockingbird::Boid>         proximity,
                                flockingbird::FlockSimulationParameters configuration) {
        Vector3D sum(0, 0, 0);
        int      count = 0;
        for (flockingbird::Boid boid : proximity) {
            sum = sum + boid.velocity.normalized();
            count++;
        }
        if (count > 0) {
            sum            = sum / count;
            sum            = sum.normalized() * configuration.speedLimit;
            Vector3D steer = sum - boidToUpdate.velocity;
            return steer.limit(configuration.forceLimit) * configuration.alignmentWeight;
        }
        return sum;
    }
};

class CohesionRule : public Rule {
public:
    virtual Vector3D operator()(flockingbird::Boid                      boidToUpdate,
                                std::vector<flockingbird::Boid>         proximity,
                                flockingbird::FlockSimulationParameters configuration) {
        Vector3D sum(0, 0, 0);
        int      count = 0;
        for (flockingbird::Boid boid : proximity) {
            sum = sum + boid.position;
            count++;
        }
        // Steer towards average position
        if (count > 0) {
            Vector3D target  = sum / count;
            Vector3D desired = target - boidToUpdate.position;
            desired          = desired.normalized() * configuration.speedLimit;
            Vector3D steer   = desired - boidToUpdate.velocity;
            return steer.limit(configuration.forceLimit) * configuration.cohesionWeight;
        }
        return sum;
    }
};

static SeparationRule separationRule;
static AlignmentRule  alignmentRule;
static CohesionRule   cohesionRule;

static std::vector<Rule*> defaultRules{&separationRule, &alignmentRule, &cohesionRule};
