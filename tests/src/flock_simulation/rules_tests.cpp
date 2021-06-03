#include "flock_simulation/rules.hpp"
#include "gtest/gtest.h"
#include <vector>
using namespace flockingbird;

class RulesTest : public ::testing::Test {
public:
protected:
    RulesTest()
        : boidToUpdate(Boid(Vector2D(0, 0), Vector2D(1.0, 1.0))) {
        Boid boid1 = Boid(Vector2D(1.0, 2), Vector2D(1.0, 1.0));
        Boid boid2 = Boid(Vector2D(2, 3), Vector2D(2, 1.0));
        Boid boid3 = Boid(Vector2D(3, 4), Vector2D(1.0, 3.5));
        proximity.push_back(boid1);
        proximity.push_back(boid2);
        proximity.push_back(boid3);
    };
    std::vector<Boid> proximity;
    Boid              boidToUpdate;
    virtual void      TearDown(){};
};


TEST_F(RulesTest, SeparationSimplestTest) {
  // Arrange Simple test with just one neighbor, and unit parameters
  FlockSimulationParameters parameters;
  parameters.speedLimit = 1;
  parameters.forceLimit = 500;
  parameters.avoidanceRadius = 25;
  parameters.separationWeight                    = 1;
  Boid boidToUpdate                 = Boid(Vector2D(2.0, 2.0), Vector2D(1.0, 1.0));
  Boid boid2                 = Boid(Vector2D(1.0, 1.0), Vector2D(1.0, 1.0));
  std::vector<Boid> proximity { boid2 };
  SeparationRule rule;

  // Difference between two boids is 1, 1
  // This is normalized and devided by the distance (sqrt 2)
  // Averaged (which has no effect)
  Vector2D expectedSteer = Vector2D(1, 1).normalized() * (1.0/sqrt(2));

    // After that we normalize the result
    // multiply with max speed (no effect)
    // subtract the velocity (1, 1)
    // and limit it by max force (no effect)
  Vector2D expectedResult = expectedSteer.normalized() - Vector2D(1, 1);

  // Act
  Vector2D actualResult = rule(boidToUpdate, proximity, parameters);

  // Assert
  EXPECT_NEAR(expectedResult.x, actualResult.x, 1E-5);
  EXPECT_NEAR(expectedResult.y, actualResult.y, 1E-5);
}

TEST_F(RulesTest, SeparationCancellingForcesTest) {
  // Arrange
    FlockSimulationParameters parameters;
    parameters.speedLimit          = 3;
    parameters.forceLimit          = 500;
    parameters.avoidanceRadius     = 25;
    parameters.separationWeight    = 1;
    Boid              boidToUpdate = Boid(Vector2D(2.0, 2.0), Vector2D(2.5, 1.5));
    Boid              boid2        = Boid(Vector2D(1.0, 1.0), Vector2D(1.0, 1.0));
    Boid              boid3        = Boid(Vector2D(3.0, 3.0), Vector2D(1.0, 1.0));
    std::vector<Boid> proximity{boid2, boid3};
    SeparationRule    rule;

    // Act
    Vector2D actualResult = rule(boidToUpdate, proximity, parameters);
    // Assert
    EXPECT_NEAR(0, actualResult.x, 1E-5);
    EXPECT_NEAR(0, actualResult.y, 1E-5);
}

TEST_F(RulesTest, SeparationComplexTest) {
  // Arrange Simple test with just one neighbor, and unit parameters
  FlockSimulationParameters parameters;
  parameters.speedLimit = 3;
  parameters.forceLimit = 0.1;
  parameters.avoidanceRadius = 25;
  parameters.separationWeight = 2;

  Boid              boidToUpdate = Boid(Vector2D(2.0, 2.0), Vector2D(2.5, 1.5));
  Boid              boid2        = Boid(Vector2D(1.5, 1.5), Vector2D(1.0, 1.0));
  Boid              boid3        = Boid(Vector2D(0.5, 0.5), Vector2D(1.0, 1.0));
  std::vector<Boid> proximity { boid2, boid3 };
  SeparationRule rule;

  float dist1 = Vector2D(2.0, 2.0).distanceTo(Vector2D(1.5, 1.5));
  float   dist2 = Vector2D(2.0, 2.0).distanceTo(Vector2D(0.5, 0.5));
  Vector2D diff1         = Vector2D(0.5, 0.5).normalized() / dist1;
  Vector2D diff2         = Vector2D(1.5, 1.5).normalized() / dist2;
  Vector2D expectedSteer = (diff1 + diff2) / 2;

  Vector2D expectedResult = ((expectedSteer.normalized() * 3) - Vector2D(2.5, 1.5)).limit(0.1) * 2;

  // Act
  Vector2D actualResult = rule(boidToUpdate, proximity, parameters);

  // Assert
  EXPECT_NEAR(expectedResult.x, actualResult.x, 1E-5);
  EXPECT_NEAR(expectedResult.y, actualResult.y, 1E-5);
}

TEST_F(RulesTest, AligmentSimpleTest) {
    // Arrange Simple test with just one neighbor, and unit parameters
  FlockSimulationParameters parameters;
  parameters.speedLimit = 3;
  parameters.forceLimit = 0.1;
  parameters.alignmentWeight = 1;
  Boid              boidToUpdate = Boid(Vector2D(1.0, 1.0), Vector2D(3.0, 3.0));
  Boid              boid2        = Boid(Vector2D(0.0, 0.0), Vector2D(1.0, 1.0));
  std::vector<Boid> proximity { boid2 };
  AlignmentRule rule;

  Vector2D expectedResult = ((Vector2D(1.0, 1.0).normalized() * 3) - Vector2D(3.0, 3.0)).limit(0.1);

  // Act
  Vector2D actualResult = rule(boidToUpdate, proximity, parameters);

  // Assert
  EXPECT_NEAR(expectedResult.x, actualResult.x, 1E-5);
  EXPECT_NEAR(expectedResult.y, actualResult.y, 1E-5);
}

TEST_F(RulesTest, AlignComplexTest) {
    FlockSimulationParameters parameters;
    parameters.speedLimit          = 3;
    parameters.forceLimit          = 0.1;
    parameters.alignmentWeight     = 2;
    Boid              boidToUpdate = Boid(Vector2D(2.0, 2.0), Vector2D(2.5, 1.5));
    Boid              boid2        = Boid(Vector2D(1.5, 1.5), Vector2D(1.0, 2.1));
    Boid              boid3        = Boid(Vector2D(0.5, 0.5), Vector2D(3.5, 1.0));
    std::vector<Boid> proximity{ boid2, boid3 };
    AlignmentRule     rule;

    Vector2D expectedResult
      = (((Vector2D(4.5, 3.1) / 2).normalized() * 3) - Vector2D(2.5, 1.5)).limit(0.1) * 2;

    // Act
    Vector2D actualResult = rule(boidToUpdate, proximity, parameters);

    // Assert
    EXPECT_NEAR(expectedResult.x, actualResult.x, 1E-5);
    EXPECT_NEAR(expectedResult.y, actualResult.y, 1E-5);
}


TEST_F(RulesTest, CohesionSimpleTest) {
    // Arrange Simple test with just one neighbor, and unit parameters
    FlockSimulationParameters parameters;
    parameters.speedLimit          = 3;
    parameters.forceLimit          = 0.1;
    parameters.cohesionWeight     = 1;
    Boid              boidToUpdate = Boid(Vector2D(1.0, 1.0), Vector2D(3.0, 3.0));
    Boid              boid2        = Boid(Vector2D(0.0, 0.0), Vector2D(1.0, 1.0));
    std::vector<Boid> proximity{boid2};
    CohesionRule rule;


    // Averaged position of other boids is 0, 0
    Vector2D expectedResult
      = ((Vector2D(-1, -1).normalized() * 3) - Vector2D(3, 3)).limit(0.1);

    // Act
    Vector2D actualResult = rule(boidToUpdate, proximity, parameters);

    // Assert
    EXPECT_NEAR(expectedResult.x, actualResult.x, 1E-5);
    EXPECT_NEAR(expectedResult.y, actualResult.y, 1E-5);
}

TEST_F(RulesTest, CohesionComplexTest) {
    // Arrange Simple test with just one neighbor, and unit parameters
    FlockSimulationParameters parameters;
    parameters.speedLimit          = 3;
    parameters.forceLimit          = 0.1;
    parameters.cohesionWeight     = 2;
    Boid              boidToUpdate = Boid(Vector2D(1.0, 1.0), Vector2D(3.0, 3.0));
    Boid              boid2        = Boid(Vector2D(2.0, 1.0), Vector2D(1.0, 1.0));
    Boid              boid3        = Boid(Vector2D(3.0, 3.5), Vector2D(1.0, 1.0));
    std::vector<Boid> proximity{ boid2, boid3 };
    CohesionRule      rule;

    // Averaged position of other boids is 0, 0
    Vector2D expectedResult = (((Vector2D(1.5, 1.25)).normalized() * 3) - Vector2D(3, 3)).limit(0.1) * 2;

    // Act
    Vector2D actualResult = rule(boidToUpdate, proximity, parameters);

    // Assert
    EXPECT_NEAR(expectedResult.x, actualResult.x, 1E-5);
    EXPECT_NEAR(expectedResult.y, actualResult.y, 1E-5);
}
