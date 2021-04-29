#include "gtest/gtest.h"
#include "utility/vector_operations.h"

using namespace std;
using namespace FlockSimulation;
using namespace VectorOperations;

class FlockingbirdTest : public ::testing::Test {

protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(FlockingbirdTest, RandomInit) {
    Flock flock(10, 10, 10);
    EXPECT_EQ(flock.boids.size(), 10);
    for (int i = 0; i < 10; i++) {
        Boid boid = flock.boids[i];
        EXPECT_GE(boid.velocity.x, 0);
        EXPECT_LE(boid.velocity.x, 1);
        EXPECT_GE(boid.velocity.y, 0);
        EXPECT_LE(boid.velocity.y, 1);
        EXPECT_GE(boid.position.x, 0);
        EXPECT_LE(boid.position.x, 10);
        EXPECT_GE(boid.position.y, 0);
        EXPECT_LE(boid.position.y, 10);
    }
}

TEST_F(FlockingbirdTest, CopyConstructors) {
    const int N        = 1000;
    const int numTests = 10;
    for (int testRun = 0; testRun < numTests; testRun++) {
        Flock flock(N, 100, 100);
        Flock copy(flock);
        for (int i = 0; i < N; i++) {
            EXPECT_EQ(flock.boids[i].position.x, copy.boids[i].position.x);
            EXPECT_EQ(flock.boids[i].position.y, copy.boids[i].position.y);
            EXPECT_EQ(flock.boids[i].velocity.x, copy.boids[i].velocity.x);
            EXPECT_EQ(flock.boids[i].velocity.y, copy.boids[i].velocity.y);
        }
    }
}
