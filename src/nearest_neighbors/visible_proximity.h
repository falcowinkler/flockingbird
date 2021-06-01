#include "../flock_simulation/flock.h"
#include "nanoflann.h"
#include <iostream>
#include <ostream>
#include <vector>
using namespace nanoflann;

const int dim     = 2;
const int maxLeaf = 10;

typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, Flock>, Flock, dim> kd_tree_t;

class VisibleProximity {
private:
    double    visionRange;
    Flock     flock;
  kd_tree_t kdTree;

public:
    VisibleProximity(Flock flockToQuery)
        : flock(flockToQuery)
        , kdTree(dim, flock, KDTreeSingleIndexAdaptorParams(maxLeaf)) {
          kdTree.buildIndex();
    }
    std::vector<Boid> of(int index, double visionRange) {
        nanoflann::SearchParams params;
        params.sorted = false;
        std::vector<std::pair<size_t, double>> ret_matches;

        Vector2D     boidPosition = flock.boids[index].position;
        const double query_pt[2]  = {boidPosition.x, boidPosition.y};
        const size_t nMatches     = kdTree.radiusSearch(query_pt, visionRange, ret_matches, params);

        // TODO: Maybe block vision in the backwards direction of the bird?
        std::vector<Boid> result;
        for (int i = 0; i < nMatches; i++) {
          double distance = ret_matches[i].second;
          if (distance > 0) {
            result.push_back(flock.boids[ret_matches[i].first]);
          }
        }
        return result;
    }
};
