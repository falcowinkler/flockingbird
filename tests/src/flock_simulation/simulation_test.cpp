#include "utility/vector_operations.hpp"
#include "gtest/gtest.h"
#include <iostream>
#include "flock_simulation/simulation.hpp"
#include "gmock/gmock.h"

using ::testing::_;
using ::testing::Return;
using namespace flockingbird;


class SimulationTest : public ::testing::Test {
public:
protected:
  SimulationTest() {
        Boid boid1 = Boid(Vector3D(1.0, 2, 0), Vector3D(1.0, 1.0, 0));
        Boid boid2 = Boid(Vector3D(2, 3, 0), Vector3D(2, 1.0, 0));
        Boid boid3 = Boid(Vector3D(3, 4, 0), Vector3D(1.0, 3.5, 0));
        Boid boid4 = Boid(Vector3D(0, 0, 0), Vector3D(1.0, 1.0, 0));
        Boid boid5 = Boid(Vector3D(1024, 1024, 0),
                          Vector3D(1.0, 1.0, 0));  // out of range boid, should not affect calculations
        boids.push_back(boid1);
        boids.push_back(boid2);
        boids.push_back(boid3);
        boids.push_back(boid4);
        boids.push_back(boid5);
        flock.boids = boids;
    };
    std::vector<Boid> boids;
    Flock flock;
    virtual void      TearDown(){};
};


class MockRule : public Rule {
public:
  MockRule(): callReturnValue(Vector3D(1, 1, 0)) {
  }
  Vector3D callReturnValue;
  MOCK_METHOD(Vector3D, Apply, (Boid, std::vector<Boid>, FlockSimulationParameters));
  Vector3D operator()(Boid boidToUpdate, std::vector<Boid> proximity, FlockSimulationParameters configuration) override {
    return Apply(boidToUpdate, proximity, configuration);
  }
};


TEST_F(SimulationTest, TestStepAppliesRulesToSingleOutlierBoid) {
  // Arrange
  FlockSimulationParameters testParameters;
  MockRule                 dummyRule;
  testParameters.speedLimit = 500;
  testParameters.twoD = true;
  testParameters.maxX = 2048;
  testParameters.maxY       = 2048;
  testParameters.maxZ       = 2048;
  EXPECT_CALL(dummyRule, Apply(_, _, _))
    .WillRepeatedly(Return(Vector3D(1, 1, 0)));

  std::vector<Rule*>        rules;
  rules.push_back(&dummyRule);
  FlockSimulation simulation(testParameters, flock, rules);
  // Act
  simulation.step();

  Boid outlierBoid = flock.boids[4];
  Vector3D expectedPosition = Vector3D(1026, 1026, 0);

  // Assert
  EXPECT_EQ(outlierBoid.position, expectedPosition);
  EXPECT_NEAR(outlierBoid.velocity.x, 2, 1E-5);
  EXPECT_NEAR(outlierBoid.velocity.y, 2, 1E-5);
  EXPECT_NEAR(outlierBoid.velocity.z, 0, 1E-5);
}


TEST_F(SimulationTest, TestSteppAppliesRulesForAllNeighbors) {
  // Arrange
  FlockSimulationParameters testParameters;
  testParameters.speedLimit = 500;
  testParameters.twoD = true;
  testParameters.maxX       = 2048;
  testParameters.maxY       = 2048;
  testParameters.maxZ       = 2048;
  MockRule                  dummyRule;
  std::vector<Rule*>        rules;
  rules.push_back(&dummyRule);
  FlockSimulation simulation(testParameters, flock, rules);
  EXPECT_CALL(dummyRule, Apply(_, _, _)).Times(5).WillRepeatedly(Return(Vector3D(1, 1, 0)));

  // Act
  simulation.step();
}
