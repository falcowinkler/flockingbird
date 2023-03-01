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

class CollisionRule : public Rule {
    virtual Vector3D operator()(flockingbird::Boid                      boidToUpdate,
                                std::vector<flockingbird::Boid>         proximity,
                                flockingbird::FlockSimulationParameters configuration) {

        // std::cout << "Lets check for collisions! :)" << std::endl;
        std::vector<Vector3D> viewRays     = BoidHelper();
        int                   count        = 0;
        Vector3D              steer        = Vector3D{0.0, 0.0, 0.0};
        float                 t            = 0;
        float                 furthestDist = 0;
        bool hit = detectCollision(boidToUpdate.position, boidToUpdate.velocity, t, configuration);
        if (hit && t < 5) {
           // std::cout << "velocity t " << t << std::endl;
            for (Vector3D viewRay : viewRays) {
                bool hit = detectCollision(boidToUpdate.position, viewRay, t, configuration);
                
              //  std::cout << "viewRay t " << t << std::endl;

                if (t > furthestDist) {
                    steer        = viewRay;
                    furthestDist = t;
                }
                if(!hit && furthestDist >= 100) break;
            }
        }

        if (steer.magnitude() > 0) {
            steer = steer.normalized() * configuration.speedLimit - boidToUpdate.velocity;

            return steer.limit(configuration.forceLimit) * configuration.avoidanceWeight;
        }
        return steer;
    };

    bool detectCollision(Vector3D                                startPos,
                         Vector3D                                dir,
                         float&                                  t,
                         flockingbird::FlockSimulationParameters configuration) {
        // r.dir is unit direction vector of ray
        dir = dir.normalized();
        //  startPos = startPos.normalized();
        float tmin, tmax, tymin, tymax, tzmin, tzmax;

        int sign[3];

        Vector3D invdir = dir;
        Vector3D bounds[2]
            = {Vector3D{configuration.maxX, configuration.maxY, configuration.maxZ} * -1,
               Vector3D{configuration.maxX, configuration.maxY, configuration.maxZ}};

        invdir.x = 1 / dir.x;
        invdir.y = 1 / dir.y;
        invdir.z = 1 / dir.z;

        sign[0] = (invdir.x < 0);
        sign[1] = (invdir.y < 0);
        sign[2] = (invdir.z < 0);

        tmin  = (bounds[sign[0]].x - startPos.x) * invdir.x;
        tmax  = (bounds[1 - sign[0]].x - startPos.x) * invdir.x;
        tymin = (bounds[sign[1]].y - startPos.y) * invdir.y;
        tymax = (bounds[1 - sign[1]].y - startPos.y) * invdir.y;

        // if ((tmin > tymax) || (tymin > tmax))
        //  return false;

        if (tymin > tmin)
            tmin = tymin;
        if (tymax < tmax)
            tmax = tymax;

        tzmin = (bounds[sign[2]].z - startPos.z) * invdir.z;
        tzmax = (bounds[1 - sign[2]].z - startPos.z) * invdir.z;

        // if ((tmin > tzmax) || (tzmin > tmax))
        //    return false;

        if (tzmin > tmin)
            tmin = tzmin;
        if (tzmax < tmax)
            tmax = tzmax;

        t = tmin;

        if (t < 0) {
            t = tmax;
            if (t < 0)
                return false;
        }

        return true;
    };
    // changeDirFunction
    std::vector<Vector3D> BoidHelper() {
        std::vector<Vector3D> calcDirs(300);

        float goldenRatio    = (1 + sqrt(5)) / 2;
        float angleIncrement = 3.14159265359 * 2 * goldenRatio;

        for (int i = 0; i < 300; i++) {
            float t           = (float)i / 300;
            float inclination = acos(1 - 2 * t);
            float azimuth     = angleIncrement * i;

            float x     = sin(inclination) * cos(azimuth);
            float y     = sin(inclination) * sin(azimuth);
            float z     = cos(inclination);
            calcDirs[i] = Vector3D{x, y, z};
        }

        return calcDirs;
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
static CollisionRule  collisionRule;

static std::vector<Rule*> defaultRules{&separationRule,
                                       &alignmentRule,
                                       &cohesionRule,
                                       &collisionRule};
