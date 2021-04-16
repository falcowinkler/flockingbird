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
        flock.boids.resize(2);

        Point point1;
        point1.x = 1.01;
        point1.y = 2.12;

        Boid boid1;
        boid1.bearing  = 0.1;
        boid1.position = point1;

        Point point2;
        point2.x = 1.5;
        point2.y = 2.5;

        Boid boid2;
        boid2.bearing  = 0.254;
        boid2.position = point2;

        flock.boids[0] = boid1;
        flock.boids[1] = boid2;
    };

    Flock        flock;
    virtual void TearDown(){};
};

TEST_F(VisibleProximityTest, FindNearestNeighbors) {
  VisibleProximity visibleProximity(flock, 1);
  std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/0);
  EXPECT_EQ(visibleBoids.size(), 2);

  Boid firstBoid = visibleBoids.front();
  Boid secondBoid = visibleBoids.back();

  EXPECT_EQ(firstBoid.bearing, 0.1);
  EXPECT_EQ(firstBoid.position.x, 1.01);
  EXPECT_EQ(firstBoid.position.y, 2.12);

  EXPECT_EQ(secondBoid.bearing, 0.254);
  EXPECT_EQ(secondBoid.position.x, 1.5);
  EXPECT_EQ(secondBoid.position.y, 2.5);

}
