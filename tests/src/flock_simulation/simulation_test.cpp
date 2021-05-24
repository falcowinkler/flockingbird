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


class DummyRule : public Rule {
public:
  DummyRule(): callReturnValue(Vector2D(1, 1)) {
  }
  Vector2D callReturnValue;
  Vector2D operator()(Boid boidToUpdate, std::vector<Boid> proximity) override {
    return callReturnValue;
  }
};


TEST_F(SimulationTest, TestStepAppliesRulesToSingleOutlierBoid) {
  FlockSimulationParameters testParameters = FlockSimulationParameters(500, 1, 1, 2);
  DummyRule                 dummyRule;
  std::vector<Rule*>        rules;
  rules.push_back(&dummyRule);
  Simulation simulation(testParameters, flock, rules);
  simulation.step();

  Boid outlierBoid = flock.boids[4];
  Vector2D expectedPosition = Vector2D(1025, 1025);
  EXPECT_EQ(outlierBoid.position, expectedPosition);
}


TEST_F(SimulationTest, TestSteppAppliesRulesForAllNeighbors) {
  
}
