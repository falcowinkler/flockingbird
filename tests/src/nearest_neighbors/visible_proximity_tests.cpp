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
        flock.boids.resize(3);

        vector<double> dummyDirection = vector<double>{1.0, 1.0};

        Point point1 = Point(1.01, 2.12);
        Boid  boid1  = Boid(point1, dummyDirection);

        Point point2 = Point(1.5, 2.5);
        Boid  boid2  = Boid(point2, dummyDirection);

        Point point3 = Point(5, 4);
        Boid  boid3  = Boid(point3, dummyDirection);

        flock.boids[0] = boid1;
        flock.boids[1] = boid2;
        flock.boids[2] = boid3;
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

  EXPECT_EQ(firstBoid.position.x, 1.01);
  EXPECT_EQ(firstBoid.position.y, 2.12);

  EXPECT_EQ(secondBoid.position.x, 1.5);
  EXPECT_EQ(secondBoid.position.y, 2.5);

}
