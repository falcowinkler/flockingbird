#include "gtest/gtest.h"
#include "../src/flockingbird.h"
#define GTEST_COUT std::cerr << "[          ] [ INFO ]"
using namespace nanoflann;
using namespace std;
using namespace FlockSimulation;

class NanoflannTest: public ::testing::Test {
public:

protected:
  NanoflannTest() : flock(Flock(0)) {
        flock.boids.resize(2);

        Point point1;
        point1.x = 1;
        point1.y = 2;

        Boid boid1;
        boid1.bearing  = 0.1;
        boid1.position = point1;

        Point point2;
        point2.x = 1;
        point2.y = 2;

        Boid boid2;
        boid2.bearing  = 0.1;
        boid2.position = point2;

        flock.boids[0] = boid1;
        flock.boids[1] = boid2;
    };

    Flock        flock;
    virtual void TearDown(){
    };
};

TEST_F(NanoflannTest, FindsNearestNeighbors) {
  const int dim = 2;
  // construct a kd-tree index:
  typedef KDTreeSingleIndexAdaptor<L1_Adaptor<double, Flock>,
                                   Flock,
                                   dim /* dim */
                                   > my_kd_tree_t;
  my_kd_tree_t index(dim /*dim*/, flock, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
  index.buildIndex();

  nanoflann::SearchParams params;

  const double search_radius = static_cast<double>(0.1);
  std::vector<std::pair<size_t, double>> ret_matches;

  const double query_pt[2] = {1, 2.09};
  const size_t nMatches = index.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
  GTEST_COUT << "Hello World" << std::endl;
  EXPECT_EQ(nMatches, 2) << "Found wrong neighbors: " << nMatches;
}
