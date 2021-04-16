#include "gtest/gtest.h"
#include "nearest_neighbors/nanoflann.h"
#include "flock_simulation/flock.h"

using namespace nanoflann;
using namespace std;
using namespace FlockSimulation;

const int dim = 2;
const int maxLeaf = 10;

typedef KDTreeSingleIndexAdaptor<L1_Adaptor<double, Flock>,
                                 Flock,
                                 dim
                                 > my_kd_tree_t;

class NanoflannTest: public ::testing::Test {
public:

protected:
  NanoflannTest() : flock(Flock()),
                    kdTree(dim, flock, KDTreeSingleIndexAdaptorParams(maxLeaf)) {
        flock.boids.resize(3);
        Point point1 = Point(1, 2);
        Boid boid1 = Boid(point1, 0.1, 0);

        Point point2 = Point(1, 2);
        Boid  boid2  = Boid(point2, 0.1, 0);

        Point point3 = Point(5, 4);
        Boid  boid3  = Boid(point3, 360, 0.5);

        flock.boids[0] = boid1;
        flock.boids[1] = boid2;
        flock.boids[2] = boid3;

        const int dim = 2;
        const int maxLeaf = 10;

        kdTree.buildIndex();
    };

    Flock        flock;
    my_kd_tree_t kdTree;
    virtual void TearDown(){
    };
};


TEST_F(NanoflannTest, FindsNeighborsNextToQueryPoint) {
  nanoflann::SearchParams params;

  const double search_radius = static_cast<double>(0.1);
  std::vector<std::pair<size_t, double>> ret_matches;

  const double query_pt[2] = {1, 2.09};
  const size_t nMatches = kdTree.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
  EXPECT_EQ(nMatches, 2) << "Found wrong neighbors: " << nMatches;
  EXPECT_EQ(ret_matches[0].first, 0);
  EXPECT_EQ(ret_matches[1].first, 1);
}

TEST_F(NanoflannTest, ExcludesNeighborsIfNotInRadius) {
    nanoflann::SearchParams params;

    const double                           search_radius = static_cast<double>(0.5);
    std::vector<std::pair<size_t, double>> ret_matches;

    const double query_pt[2] = {5.2, 4.1};
    const size_t nMatches = kdTree.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
    EXPECT_EQ(nMatches, 1) << "Found wrong neighbors: " << nMatches;
    EXPECT_EQ(ret_matches[0].first, 2);
}
