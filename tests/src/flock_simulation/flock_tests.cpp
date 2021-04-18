#include "flock_simulation/flock.h"
#include "gtest/gtest.h"

using namespace std;

class FlockingbirdTests : public ::testing::Test {

protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(FlockingbirdTests, RandomInit) {
    FlockSimulation::Flock flock(10, 10, 10);
    EXPECT_EQ(flock.boids.size(), 10);
    for (int i = 0; i < 10; i++) {
        FlockSimulation::Boid boid = flock.boids[i];
        EXPECT_GE(boid.velocity[0], 0);
        EXPECT_LE(boid.velocity[1], 1);
        EXPECT_GE(boid.position.x, 0);
        EXPECT_LE(boid.position.x, 10);
        EXPECT_GE(boid.position.y, 0);
        EXPECT_LE(boid.position.y, 10);
    }
}
