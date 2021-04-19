#include "nanoflann.h"
#include "flock_simulation/flock.h"
#include <vector>
using namespace nanoflann;
using namespace FlockSimulation;


const int dim     = 2;
const int maxLeaf = 10;

typedef KDTreeSingleIndexAdaptor<L1_Adaptor<double, Flock>, Flock, dim> kd_tree_t;

class VisibleProximity {
 private:
  kd_tree_t kdTree;
  double visionRange;
  Flock flock;
 public:

  VisibleProximity(FlockSimulation::Flock flockToQuery, double boidVisionRange) :
    kdTree(dim, flockToQuery, KDTreeSingleIndexAdaptorParams(maxLeaf)),
    visionRange(boidVisionRange),
    flock(flockToQuery) {
    kdTree.buildIndex();
  }

  std::vector<Boid> of(int index) {

      nanoflann::SearchParams params;

      const double                           search_radius = static_cast<double>(visionRange);
      std::vector<std::pair<size_t, double>> ret_matches;

      Vector2D boidPosition = flock.boids[index].position;

      const double query_pt[2] = {boidPosition.x, boidPosition.y};
      const size_t nMatches = kdTree.radiusSearch(&query_pt[0], search_radius, ret_matches, params);

      // Maybe block vision in the backwards direction of the bird?

      std::vector<Boid> result;
      for (int i = 0; i < nMatches; i++) {
        result.push_back(flock.boids[ret_matches[i].first]);
      }
      return result;
  }
};
