#include "flock_simulation/rules.h"
#include "nearest_neighbors/visible_proximity.h"

using namespace FlockSimulation;
using namespace VectorOperations;

inline Flock step(Flock flock) {
    Flock            result(flock);
    const double     visionRange       = 50.0; // squared euclidean distance
    VisibleProximity visibleProximity(flock);
    std::cout << "number of boids: " << flock.boids.size() << std::endl;
    for (int i = 0; i < flock.boids.size(); i++) {
        Boid boidToUpdate = flock.boids[i];
        std::cout << "boid to update: " << boidToUpdate << std::endl;

        // TODO: include boid at index i from result;
        std::vector<Boid> proximity      = visibleProximity.of(i, visionRange);
        std::cout << "found proximity: " << std::endl;
        for (auto it = proximity.begin(); it < proximity.end(); it++) {
          std::cout << "        " << *it << std::endl;
        }
        std::cout << "------------------\n";
        Vector2D          cohesion       = Rules::cohesion(boidToUpdate, proximity);
        Vector2D          seperation     = Rules::seperation(boidToUpdate, proximity);
        Vector2D          alignment      = Rules::alignment(boidToUpdate, proximity);

        std::cout << "cohesion: " << cohesion << std::endl;
        std::cout << "separation: " << seperation << std::endl;
        std::cout << "alignment: " << alignment << std::endl;

        Vector2D          velocityCorrection
            = vecSum(boidToUpdate.velocity, vecSum(vecSum(cohesion, seperation), alignment));
        std::cout << "velocity correctionL: " << velocityCorrection << std::endl;
        result.boids[i].velocity = velocityCorrection;
        result.boids[i].position = vecSum(result.boids[i].position, velocityCorrection);
    }
    return result;
}
