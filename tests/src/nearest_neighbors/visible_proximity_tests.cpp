#include "gtest/gtest.h"
#include "nearest_neighbors/visible_proximity.h"

using namespace nanoflann;
using namespace std;
using namespace FlockSimulation;

class VisibleProximityTest : public ::testing::Test {
public:
protected:
    VisibleProximityTest()
        : flock(Flock(0, 10, 10)) {

        Vector2D dummyDirection = Vector2D(1.0, 1.0);

        Boid  boid1  = Boid(Vector2D(1.01, 2.12), dummyDirection);
        Boid  boid2  = Boid(Vector2D(1.5, 2.5), dummyDirection);
        Boid  boid3  = Boid(Vector2D(5, 4), dummyDirection);
        Boid  boid4  = Boid(Vector2D(5.01, 3.99), dummyDirection);
        flock.boids.push_back(boid1);
        flock.boids.push_back(boid2);
        flock.boids.push_back(boid3);
        flock.boids.push_back(boid4);
    };

    Flock        flock;
    virtual void TearDown(){};
};

TEST_F(VisibleProximityTest, FindNearestNeighborsInCloseProximity) {
  VisibleProximity visibleProximity(flock, 1);
  std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/0);
  EXPECT_EQ(visibleBoids.size(), 2);

  Boid firstBoid = visibleBoids.front();
  Boid secondBoid = visibleBoids.back();

  EXPECT_EQ(firstBoid.position.x, 1.01);
  EXPECT_EQ(firstBoid.position.y, 2.12);

  EXPECT_EQ(secondBoid.position.x, 1.5);
  EXPECT_EQ(secondBoid.position.y, 2.5);
}

TEST_F(VisibleProximityTest, FindTheWholeFlockWithASufficientVisionRange) {
    VisibleProximity  visibleProximity(flock, 10);
    std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ 2);
    EXPECT_EQ(visibleBoids.size(), flock.boids.size());
}

TEST_F(VisibleProximityTest, FindsNeigborWithVeryNarrowVision) {
    VisibleProximity  visibleProximity(flock, 0.03);
    std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ 2);
    EXPECT_EQ(visibleBoids.size(), 2);
}
