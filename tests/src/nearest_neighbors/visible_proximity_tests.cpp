#include "nearest_neighbors/nanoflann.hpp"
#include "nearest_neighbors/visible_proximity.hpp"
#include "utility/random_numbers.hpp"
#include "gtest/gtest.h"
#include <cmath>
#include <cstdlib>
#include <vector>

using namespace nanoflann;
using namespace flockingbird;

class VisibleProximityTest : public ::testing::Test {
public:
protected:
    VisibleProximityTest()
      : flock(Flock(0, 10, 10, 0)) {

      Vector3D dummyDirection = Vector3D(1.0, 1.0, 0);

        Boid boid1 = Boid(Vector3D(1.01, 2.12, 0), dummyDirection);
        Boid boid2 = Boid(Vector3D(1.5, 2.5, 0), dummyDirection);
        Boid boid3 = Boid(Vector3D(5, 4, 0), dummyDirection);
        Boid boid4 = Boid(Vector3D(5.01, 3.99, 0), dummyDirection);
        flock.boids.push_back(boid1);
        flock.boids.push_back(boid2);
        flock.boids.push_back(boid3);
        flock.boids.push_back(boid4);
    };

    Flock        flock;
    virtual void TearDown(){};
};

TEST_F(VisibleProximityTest, FindNearestNeighborsInCloseProximity) {
  VisibleProximity  visibleProximity(flock);
  const float visionRange = 1;
  std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ 0, visionRange);
    EXPECT_EQ(visibleBoids.size(), 1);

    Boid firstBoid  = visibleBoids.front();

    EXPECT_EQ(firstBoid.position.x, 1.5);
    EXPECT_EQ(firstBoid.position.y, 2.5);
}

TEST_F(VisibleProximityTest, FindsPointsOnEdgeOfSearchRadius) {
    VisibleProximity  visibleProximity(flock);
    const float      visionRange  = 0.3845 + 1E-5; // ((1.5 - 1.01) ^ 2 + (2.5-2.12) ^ 2 == 0.3845)
    std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ 0, visionRange);
    EXPECT_EQ(visibleBoids.size(), 1);

    Boid firstBoid  = visibleBoids.front();

    EXPECT_EQ(firstBoid.position.x, 1.5);
    EXPECT_EQ(firstBoid.position.y, 2.5);
}

TEST_F(VisibleProximityTest, ExcludesPointsOnEdgeOfSearchRadius) {
    VisibleProximity  visibleProximity(flock);
    const float      visionRange  = 0.3844;  // ((1.5 - 1.01) ^ 2 + (2.5-2.12) ^ 2 == 0.3845)
    std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ 0, visionRange);
    EXPECT_EQ(visibleBoids.size(), 0);
}

TEST_F(VisibleProximityTest, MultipleCallsDontVaryTheResult) {
    VisibleProximity  visibleProximity(flock);
    const float      visionRange  = 0.3844;  // ((1.5 - 1.01) ^ 2 + (2.5-2.12) ^ 2 == 0.3845)

    visibleProximity.of(/*boid at index*/ 0, visionRange);
    visibleProximity.of(/*boid at index*/ 2, visionRange);
    visibleProximity.of(/*boid at index*/ 1, visionRange);
    std::vector<Boid> visibleBoids = visibleProximity.of(0, visionRange);
    EXPECT_EQ(visibleBoids.size(), 0);
}

TEST_F(VisibleProximityTest, FindTheWholeFlockWithASufficientVisionRange) {
    VisibleProximity  visibleProximity(flock);
    const float visionRange = 50;
    std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ 2, visionRange);
    EXPECT_EQ(visibleBoids.size(), flock.boids.size() - 1);
}

TEST_F(VisibleProximityTest, FindsNeigborWithVeryNarrowVision) {
    VisibleProximity  visibleProximity(flock);
    const float visionRange = 0.2;
    std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ 2, visionRange);
    EXPECT_EQ(visibleBoids.size(), 1);
}

/*
 * Random tests
 *
 */
inline float L2_Reference(Vector3D a, Vector3D b) { return pow(a.x - b.x, 2) + pow(a.y - b.y, 2); }

TEST_F(VisibleProximityTest, RandomTests) {
  int N = 30; // if test runs in under 1 sec, we can reach this fps

    for (int testRun = 0; testRun < N; testRun++) {
        int               numBoids = 500;
        std::vector<Boid> boids;
        for (int boid = 0; boid < numBoids; boid++) {
            boids.push_back(
                            Boid(Vector3D(randomInBounds(0, 10), randomInBounds(0, 10), 0), Vector3D(0, 0, 0)));
        }
        float visionRange = randomInBounds(0, 10);
        Flock flock = Flock();
        flock.boids = boids;

        Flock refFlock(flock);
        VisibleProximity visibleProximity(flock);
        for (int i = 0; i < numBoids; i++) {
          std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ i, visionRange);
          for (auto boidIt = visibleBoids.begin(); boidIt != visibleBoids.end(); boidIt++) {
             EXPECT_LE(L2_Reference(boidIt->position, refFlock.boids[i].position), visionRange);
          }
        }
    }
}
