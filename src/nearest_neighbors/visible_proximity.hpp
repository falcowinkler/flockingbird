#include "../flock_simulation/flock.hpp"
#include "nanoflann.hpp"
#include <iostream>
#include <ostream>
#include <vector>
using namespace nanoflann;

const int dim     = 2;
const int maxLeaf = 10;

typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<float, flockingbird::Flock>, flockingbird::Flock, dim> kd_tree_t;

class VisibleProximity {
private:
    flockingbird::Flock flock;
    kd_tree_t           kdTree;

public:
    VisibleProximity(flockingbird::Flock flockToQuery)
        : flock(flockToQuery)
        , kdTree(dim, flock, KDTreeSingleIndexAdaptorParams(maxLeaf)) {
        kdTree.buildIndex();
    }
    std::vector<flockingbird::Boid> of(int index, float visionRange) {
        nanoflann::SearchParams params;
        params.sorted = false;
        std::vector<std::pair<size_t, float>> ret_matches;

        Vector2D     boidPosition = flock.boids[index].position;
        const float query_pt[2]  = {boidPosition.x, boidPosition.y};
        kdTree.radiusSearch(query_pt, visionRange, ret_matches, params);

        // TODO: Maybe block vision in the backwards direction of the bird?
        std::vector<flockingbird::Boid> result;
        for (auto match: ret_matches) {
            float distance = match.second;
            if (distance > 0) {
                result.push_back(flock.boids[match.first]);
            }
        }
        return result;
    }
};
