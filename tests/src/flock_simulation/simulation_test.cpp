#include "utility/vector_operations.h"
#include "gtest/gtest.h"
#include <iostream>
#include "flock_simulation/simulation.h"

using namespace std;
using namespace FlockSimulation;
using namespace VectorOperations;

class SimulationTest : public ::testing::Test {
public:
protected:
    SimulationTest()
        : testedBoidOriginal(Vector2D(0, 0), Vector2D(1.0, 1.0)) {
        Boid boid1 = Boid(Vector2D(1.0, 2), Vector2D(1.0, 1.0));
        Boid boid2 = Boid(Vector2D(2, 3), Vector2D(2, 1.0));
        Boid boid3 = Boid(Vector2D(3, 4), Vector2D(1.0, 3.5));
        Boid boid4 = Boid(Vector2D(0, 0), Vector2D(1.0, 1.0));
        Boid boid5 = Boid(Vector2D(1024, 1024),
                          Vector2D(1.0, 1.0));  // out of range boid, should not affect calculations
        proximity.push_back(boid1);
        proximity.push_back(boid2);
        proximity.push_back(boid3);
        proximity.push_back(boid4);
        proximity.push_back(boid5);
    };
    std::vector<Boid> proximity;
    Boid              testedBoidOriginal;
    virtual void      TearDown(){};
};


TEST_F(SimulationTest, Step) {
    Vector2D cohesion = Vector2D(2.0 / 100, 3.0 / 100);
    Vector2D separation = Vector2D(6, 9);
    Vector2D alignment = Vector2D(1.0 / 3 / 8, 0.833333333333333 / 8);
    Vector2D expectedCorrection = vecSum(vecSum(alignment, separation), cohesion);

    Flock sut;
    sut.boids = proximity;

    std::cout << "initiating simulation step with flock:" << std::endl;

    for (auto it = sut.boids.begin(); it != sut.boids.end(); it++) {
        std::cout << *it << std::endl;
    }

    Flock result = step(sut);

    std::cout << "simulation result:"<< std::endl;

    for (auto it = result.boids.begin(); it!=result.boids.end();it++) {
      std::cout << *it << std::endl;
    }


    Boid     testedBoid       = result.boids[3];
    Vector2D newVelocity      = testedBoid.velocity;
    Vector2D newPosition      = testedBoid.position;
    Vector2D expectedVelocity = vecSum(testedBoidOriginal.velocity, expectedCorrection);
    Vector2D scaledVelocity = vecMulScalar(expectedVelocity, POSITION_INCREMENT_SCALING_FACTOR);
    Vector2D expectedPosition = vecSum(testedBoidOriginal.position, scaledVelocity);

    double accuracy = 1E-5;

    EXPECT_NEAR(testedBoid.velocity.x, expectedVelocity.x, accuracy);
    EXPECT_NEAR(testedBoid.velocity.y, expectedVelocity.y, accuracy);
    EXPECT_NEAR(testedBoid.position.x, expectedPosition.x, accuracy);
    EXPECT_NEAR(testedBoid.position.y, expectedPosition.y, accuracy);
}


TEST_F(SimulationTest, LimitsSpeed) {
    Flock sut;
    Boid singleBoid(Vector2D(0, 0), Vector2D(1000, 1000));
    sut.boids = std::vector<Boid> {singleBoid};
    Flock result = step(sut);
    Vector2D resultingVelocity = result.boids.front().velocity;
    // 1000 * (SPEED_LIMIT / sqrt(1000 ^ 2 + 1000 ^ 2)), where speed_limit = 500;
    EXPECT_NEAR(resultingVelocity.x, 353.55339059327372, 1E-5);
    EXPECT_NEAR(resultingVelocity.y, 353.55339059327372, 1E-5);
}
