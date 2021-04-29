#include "flock_simulation/flock.h"
#include "nanoflann.h"
#include <iostream>
#include <ostream>
#include <vector>
using namespace nanoflann;
using namespace FlockSimulation;

inline double L2_Reference(Vector2D a, Vector2D b) { return pow(a.x - b.x, 2) + pow(a.y - b.y, 2); }

const int dim     = 2;
const int maxLeaf = 10;

typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, Flock>, Flock, dim> kd_tree_t;

class VisibleProximity {
private:
    kd_tree_t kdTree;
    double    visionRange;
    Flock     flock;

public:
    VisibleProximity(FlockSimulation::Flock flockToQuery)
      : flock(flockToQuery)
      , kdTree(dim, flockToQuery, KDTreeSingleIndexAdaptorParams(maxLeaf)) {
        kdTree.buildIndex();
    }

     std::vector<Boid> of(int index, double visionRange) {

        nanoflann::SearchParams params;
        params.sorted = false;
        std::vector<std::pair<size_t, double>> ret_matches;

        Vector2D boidPosition = flock.boids[index].position;
        const double query_pt[2] = {boidPosition.x, boidPosition.y};
        const size_t nMatches
            = kdTree.radiusSearch(query_pt, visionRange, ret_matches, params);

        // TODO: Maybe block vision in the backwards direction of the bird?
        // TODO: Exclude the boid itself?
        std::vector<Boid> result;
        for (int i = 0; i < nMatches; i++) {
          double nano = ret_matches[i].second; // Second item of the tuple is the distance
          double me   = L2_Reference(Vector2D(query_pt[0], query_pt[1]),
                                     flock.boids[ret_matches[i].first].position);
            result.push_back(flock.boids[ret_matches[i].first]);
            if (nano != me) { // Should never happen
                //  assert(abs(ret_matches[i].second - L2_Reference(flock.boids[index].position,
                //  flock.boids[ret_matches[i].first].position)) < 0.1);
                std::cout << "Distance result: nanoflann says: " << nano << ", i say:" << me
                          << std::endl;
                //  std::cout << "searchRadius: " << visionRange << std::endl;
                std::cout << "found: " << flock.boids[ret_matches[i].first].position << " in radius of source: " << boidPosition << ", index: " << ret_matches[i].first << std::endl;
            }
        }
        return result;
    }
};
