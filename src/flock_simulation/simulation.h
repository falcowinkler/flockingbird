#include "flock_simulation/rules.h"
#include "nearest_neighbors/visible_proximity.h"

using namespace FlockSimulation;
using namespace VectorOperations;

const double SPEED_LIMIT = 500;
const double POSITION_INCREMENT_SCALING_FACTOR = 1.0 / 100;
// ranges in squared euclidean distance
const double AVOIDANCE_RADIUS                  = 625;
const double VISION_RANGE                      = 1000000.0;
inline Flock step(Flock flock) {
    Flock            result(flock);

    VisibleProximity visibleProximity(flock);
    // std::cout << "number of boids: " << flock.boids.size() << std::endl;
    for (int i = 0; i < flock.boids.size(); i++) {
        Boid boidToUpdate = flock.boids[i];
        // std::cout << "boid to update: " << boidToUpdate << std::endl;

        // TODO: include boid at index i from result;
        std::vector<Boid> proximity      = visibleProximity.of(i, VISION_RANGE);
        std::vector<Boid> closeProximity      = visibleProximity.of(i, AVOIDANCE_RADIUS);
        //std::cout << "found proximity: " << std::endl;
        //for (auto it = proximity.begin(); it < proximity.end(); it++) {
        //  std::cout << "        " << *it << std::endl;
        //}
        //std::cout << "------------------\n";
        Vector2D          cohesion       = Rules::cohesion(boidToUpdate, proximity);
        Vector2D          seperation     = Rules::seperation(boidToUpdate, closeProximity);
        Vector2D          alignment      = Rules::alignment(boidToUpdate, proximity);

        /* std::cout << "cohesion: " << cohesion << std::endl; */
        /* std::cout << "separation: " << seperation << std::endl; */
        /* std::cout << "alignment: " << alignment << std::endl; */

        Vector2D          velocityCorrection
            = vecSum(boidToUpdate.velocity, vecSum(vecSum(cohesion, seperation), alignment));
        // std::cout << "velocity correctionL: " << velocityCorrection << std::endl;
        if (magnitude(velocityCorrection) > SPEED_LIMIT) {
            velocityCorrection
                = vecMulScalar(velocityCorrection, SPEED_LIMIT / magnitude(velocityCorrection));
        }
        result.boids[i].velocity = velocityCorrection;
        Vector2D scaledVelocity = vecMulScalar(velocityCorrection, POSITION_INCREMENT_SCALING_FACTOR);
        result.boids[i].position = vecSum(result.boids[i].position, scaledVelocity);
    }
    return result;
}
