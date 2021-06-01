#include "boid.h"
#include "configuration.h"
#include "utility/vector_operations.h"
#include <iostream>
#include <vector>
#pragma once

using namespace flockingbird;

class Rule {
public:
    virtual Vector2D
    operator()(Boid boidToUpdate, std::vector<Boid> proximity, FlockSimulationParameters configuration)
        = 0;
};

class SeparationRule: public Rule {
  public:
      virtual Vector2D operator()(Boid                      boidToUpdate,
                std::vector<Boid>         proximity,
                                  FlockSimulationParameters configuration) {
        int count = 0;
        Vector2D steer(0, 0);
        for (Boid boid : proximity) {
          double d = boidToUpdate.position.distanceTo(boid.position);
          if (d > 0 && d < configuration.avoidanceRadius) {
            Vector2D diff = (boidToUpdate.position - boid.position).normalized() / d;
            steer = steer + diff;
            count += 1;
          }
        }
        if (count > 0) {
          steer = steer / count;
        }
        if (steer.magnitude() > 0) {
          steer = steer.normalized();
          steer = steer * configuration.speedLimit;
          steer = steer - boidToUpdate.velocity;
          return steer.limit(configuration.forceLimit) * configuration.separationWeight;
        }
        return steer;
      };
};

class AlignmentRule: public Rule {
public:
    virtual Vector2D operator()(Boid                      boidToUpdate,
                                std::vector<Boid>         proximity,
                                FlockSimulationParameters configuration) {
      Vector2D sum(0, 0);
      int count = 0;
      for (Boid boid: proximity) {
        sum = sum + boid.velocity;
        count++;
      }
      if (count > 0) {
        sum = sum / count;
        sum = sum.normalized() * configuration.speedLimit;
        Vector2D steer = sum - boidToUpdate.velocity;
        return steer.limit(configuration.forceLimit) * configuration.alignmentWeight;
      }
      return sum;
    }
};

class CohesionRule: public Rule {
public:
    virtual Vector2D operator()(Boid                      boidToUpdate,
                                std::vector<Boid>         proximity,
                                FlockSimulationParameters configuration) {
      Vector2D sum(0, 0);
      int count = 0;
      for (Boid boid : proximity) {
        sum = sum + boid.position;
        count++;
      }
      // Steer towards average position
      if (count > 0) {
        Vector2D target = sum / count;
        Vector2D desired = target - boidToUpdate.position;
        desired = desired.normalized() * configuration.speedLimit;
        Vector2D steer = desired - boidToUpdate.velocity;
        return steer.limit(configuration.forceLimit) * configuration.cohesionWeight;
      }
      return sum;
    }
};

static SeparationRule separationRule;
static AlignmentRule alignmentRule;
static CohesionRule cohesionRule;

static std::vector<Rule*> defaultRules { &separationRule, &alignmentRule, &cohesionRule };
