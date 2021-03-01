#include "gtest/gtest.h"
#include "../src/flockingbird.h"

using namespace nanoflann;
using namespace std;
using namespace FlockSimulation;

class NanoflannTest: public ::testing::Test {

protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(NanoflannTest, FindsNearestNeighbors) {

  Flock flock;

  flock.points.resize(2);
  Point point1;
  point1.x = 1;
  point1.y = 2;
  Point point2;
  point2.x = 1;
  point2.y = 2;

  flock.points[0] = point1;
  flock.points[1] = point2;

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
  EXPECT_EQ(nMatches, 2);
}

TEST_F(NanoflannTest, DoesNotReturnPointsOutOfSearchRadius) {
    Flock flock;
    flock.points.resize(2);
    Point point1;
    point1.x = 1;
    point1.y = 2;
    Point point2;
    point2.x = 1;
    point2.y = 2;

    flock.points[0] = point1;
    flock.points[1] = point2;

    const int dim = 2;

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<L1_Adaptor<double, Flock>, Flock, dim /* dim */
                                     >
                 my_kd_tree_t;
    my_kd_tree_t index(dim /*dim*/, flock, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
    index.buildIndex();

    nanoflann::SearchParams params;

    const double                           search_radius = static_cast<double>(0.1);
    std::vector<std::pair<size_t, double>> ret_matches;

    const double query_pt[2] = {1, 2.19};
    const size_t nMatches    = index.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
    EXPECT_EQ(nMatches, 0);
}
