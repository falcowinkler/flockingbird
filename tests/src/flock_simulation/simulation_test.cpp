#include "utility/vector_operations.h"
#include "gtest/gtest.h"
#include <iostream>
#include "flock_simulation/simulation.h"
#include "gmock/gmock.h"

using namespace FlockSimulation;
using namespace VectorOperations;
using ::testing::_;
using ::testing::Return;

class SimulationTest : public ::testing::Test {
public:
protected:
  SimulationTest() {
        Boid boid1 = Boid(Vector2D(1.0, 2), Vector2D(1.0, 1.0));
        Boid boid2 = Boid(Vector2D(2, 3), Vector2D(2, 1.0));
        Boid boid3 = Boid(Vector2D(3, 4), Vector2D(1.0, 3.5));
        Boid boid4 = Boid(Vector2D(0, 0), Vector2D(1.0, 1.0));
        Boid boid5 = Boid(Vector2D(1024, 1024),
                          Vector2D(1.0, 1.0));  // out of range boid, should not affect calculations
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
  MockRule(): callReturnValue(Vector2D(1, 1)) {
  }
  Vector2D callReturnValue;
  MOCK_METHOD(Vector2D, Apply, (Boid, std::vector<Boid>, FlockSimulationParameters));
  Vector2D operator()(Boid boidToUpdate, std::vector<Boid> proximity, FlockSimulationParameters configuration) override {
    return Apply(boidToUpdate, proximity, configuration);
  }
};


TEST_F(SimulationTest, TestStepAppliesRulesToSingleOutlierBoid) {
  // Arrange
  FlockSimulationParameters testParameters = FlockSimulationParameters(500, 1, 1, 2);
  MockRule                 dummyRule;
  EXPECT_CALL(dummyRule, Apply(_, _, _))
    .WillRepeatedly(Return(Vector2D(1, 1)));

  std::vector<Rule*>        rules;
  rules.push_back(&dummyRule);
  Simulation simulation(testParameters, flock, rules);
  // Act
  simulation.step();

  Boid outlierBoid = flock.boids[4];
  Vector2D expectedPosition = Vector2D(1025, 1025);

  // Assert
  EXPECT_EQ(outlierBoid.position, expectedPosition);
  EXPECT_EQ(outlierBoid.velocity, Vector2D(1, 1));
}


TEST_F(SimulationTest, TestSteppAppliesRulesForAllNeighbors) {
  // Arrange
  FlockSimulationParameters testParameters = FlockSimulationParameters(500, 1, 1, 2);
  MockRule                  dummyRule;
  std::vector<Rule*>        rules;
  rules.push_back(&dummyRule);
  Simulation simulation(testParameters, flock, rules);
  EXPECT_CALL(dummyRule, Apply(_, _, _)).Times(5).WillRepeatedly(Return(Vector2D(1, 1)));

  // Act
  simulation.step();
}
