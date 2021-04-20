#include "nearest_neighbors/nanoflann.h"
#include "nearest_neighbors/visible_proximity.h"
#include "utility/random_numbers.h"
#include "gtest/gtest.h"
#include <cmath>
#include <cstdlib>
#include <vector>

using namespace nanoflann;
using namespace std;
using namespace FlockSimulation;

class VisibleProximityTest : public ::testing::Test {
public:
protected:
    VisibleProximityTest()
        : flock(Flock(0, 10, 10)) {

        Vector2D dummyDirection = Vector2D(1.0, 1.0);

        Boid boid1 = Boid(Vector2D(1.01, 2.12), dummyDirection);
        Boid boid2 = Boid(Vector2D(1.5, 2.5), dummyDirection);
        Boid boid3 = Boid(Vector2D(5, 4), dummyDirection);
        Boid boid4 = Boid(Vector2D(5.01, 3.99), dummyDirection);
        flock.boids.push_back(boid1);
        flock.boids.push_back(boid2);
        flock.boids.push_back(boid3);
        flock.boids.push_back(boid4);
    };

    Flock        flock;
    virtual void TearDown(){};
};

TEST_F(VisibleProximityTest, FindNearestNeighborsInCloseProximity) {
  VisibleProximity  visibleProximity(flock, 1);
    std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ 0);
    EXPECT_EQ(visibleBoids.size(), 2);

    Boid firstBoid  = visibleBoids.front();
    Boid secondBoid = visibleBoids.back();

    EXPECT_EQ(firstBoid.position.x, 1.01);
    EXPECT_EQ(firstBoid.position.y, 2.12);

    EXPECT_EQ(secondBoid.position.x, 1.5);
    EXPECT_EQ(secondBoid.position.y, 2.5);
}

TEST_F(VisibleProximityTest, FindTheWholeFlockWithASufficientVisionRange) {
    VisibleProximity  visibleProximity(flock, 50);
    std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ 2);
    EXPECT_EQ(visibleBoids.size(), flock.boids.size());
}

TEST_F(VisibleProximityTest, FindsNeigborWithVeryNarrowVision) {
    VisibleProximity  visibleProximity(flock, 0.03);
    std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ 2);
    EXPECT_EQ(visibleBoids.size(), 2);
}

/*
 * Random tests
 */
/*
inline double L2_Reference(Vector2D a, Vector2D b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
*/
TEST_F(VisibleProximityTest, RandomTests) {
    int N = 1;

    for (int testRun = 0; testRun < N; testRun++) {
        int               numBoids = rand() % 100;
        std::vector<Boid> boids;
        for (int boid = 0; boid < numBoids; boid++) {
            boids.push_back(
                Boid(Vector2D(randomInBounds(0, 10), randomInBounds(0, 10)), Vector2D(0, 0)));
        }
        double visionRange = randomInBounds(0, 10);
        Flock flock = Flock();
        flock.boids = boids;

        Flock refFlock(flock);
        VisibleProximity visibleProximity(flock, visionRange);
        for (int i = 0; i < numBoids; i++) {
          std::vector<Boid> visibleBoids = visibleProximity.of(/*boid at index*/ i);
          for (auto boidIt = visibleBoids.begin(); boidIt != visibleBoids.end(); boidIt++) {
             EXPECT_LE(L2_Reference(boidIt->position, refFlock.boids[i].position), visionRange);
          }
        }
    }
}
