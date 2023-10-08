#include "flock_simulation/rules.hpp"
#include "gtest/gtest.h"
#include <vector>
using namespace flockingbird;

class RulesTest : public ::testing::Test {
public:
protected:
    RulesTest()
      : boidToUpdate(Boid(Vector3D(0, 0, 0), Vector3D(1.0, 1.0, 0))) {
        Boid boid1 = Boid(Vector3D(1.0, 2, 0), Vector3D(1.0, 1.0, 0));
        Boid boid2 = Boid(Vector3D(2, 3, 0), Vector3D(2, 1.0, 0));
        Boid boid3 = Boid(Vector3D(3, 4, 0), Vector3D(1.0, 3.5, 0));
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
  Boid boidToUpdate                 = Boid(Vector3D(2.0, 2.0, 0), Vector3D(1.0, 1.0, 0));
  Boid boid2                 = Boid(Vector3D(1.0, 1.0, 0), Vector3D(1.0, 1.0, 0));
  std::vector<Boid> proximity { boid2 };
  SeparationRule rule;

  // Difference between two boids is 1, 1
  // This is normalized and devided by the distance (sqrt 2)
  // Averaged (which has no effect)
  Vector3D expectedSteer = Vector3D(1, 1, 0).normalized() / sqrt(2);

    // After that we normalize the result
    // multiply with max speed (no effect)
    // subtract the velocity (1, 1)
    // and limit it by max force (no effect)
  Vector3D expectedResult = expectedSteer.normalized() - Vector3D(1, 1, 0);

  // Act
  Vector3D actualResult = rule(boidToUpdate, proximity, parameters);

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
    Boid              boidToUpdate = Boid(Vector3D(2.0, 2.0, 0), Vector3D(2.5, 1.5, 0));
    Boid              boid2        = Boid(Vector3D(1.0, 1.0, 0), Vector3D(1.0, 1.0, 0));
    Boid              boid3        = Boid(Vector3D(3.0, 3.0, 0), Vector3D(1.0, 1.0, 0));
    std::vector<Boid> proximity{boid2, boid3};
    SeparationRule    rule;

    // Act
    Vector3D actualResult = rule(boidToUpdate, proximity, parameters);
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

  Boid              boidToUpdate = Boid(Vector3D(2.0, 2.0, 0), Vector3D(2.5, 1.5, 0));
  Boid              boid2        = Boid(Vector3D(1.5, 1.5, 0), Vector3D(1.0, 1.0, 0));
  Boid              boid3        = Boid(Vector3D(0.5, 0.5, 0), Vector3D(1.0, 1.0, 0));
  std::vector<Boid> proximity { boid2, boid3 };
  SeparationRule rule;

  float dist1 = Vector3D(2.0, 2.0, 0).distanceTo(Vector3D(1.5, 1.5, 0));
  float   dist2 = Vector3D(2.0, 2.0, 0).distanceTo(Vector3D(0.5, 0.5, 0));
  Vector3D diff1         = Vector3D(0.5, 0.5, 0).normalized() / dist1;
  Vector3D diff2         = Vector3D(1.5, 1.5, 0).normalized() / dist2;
  Vector3D expectedSteer = (diff1 + diff2) / 2;

  Vector3D expectedResult = ((expectedSteer.normalized() * 3) - Vector3D(2.5, 1.5, 0)).limit(0.1) * 2;

  // Act
  Vector3D actualResult = rule(boidToUpdate, proximity, parameters);

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
  Boid              boidToUpdate = Boid(Vector3D(1.0, 1.0, 0), Vector3D(3.0, 3.0, 0));
  Boid              boid2        = Boid(Vector3D(0.0, 0.0, 0), Vector3D(1.0, 1.0, 0));
  std::vector<Boid> proximity { boid2 };
  AlignmentRule rule;

  Vector3D expectedResult = ((Vector3D(1.0, 1.0, 0).normalized() * 3) - Vector3D(3.0, 3.0, 0)).limit(0.1);

  // Act
  Vector3D actualResult = rule(boidToUpdate, proximity, parameters);

  // Assert
  EXPECT_NEAR(expectedResult.x, actualResult.x, 1E-5);
  EXPECT_NEAR(expectedResult.y, actualResult.y, 1E-5);
}

TEST_F(RulesTest, AlignComplexTest) {
    FlockSimulationParameters parameters;
    parameters.speedLimit          = 3;
    parameters.forceLimit          = 0.1;
    parameters.alignmentWeight     = 2;
    Boid              boidToUpdate = Boid(Vector3D(2.0, 2.0, 0), Vector3D(2.5, 1.5, 0));
    Boid              boid2        = Boid(Vector3D(1.5, 1.5, 0), Vector3D(1.0, 2.1, 0));
    Boid              boid3        = Boid(Vector3D(0.5, 0.5, 0), Vector3D(3.5, 1.0, 0));
    std::vector<Boid> proximity{ boid2, boid3 };
    AlignmentRule     rule;

    Vector3D proximitySum = Vector3D(1.0, 2.1, 0) + Vector3D(3.5, 1.0, 0);
    Vector3D expectedResult
        = (((proximitySum / 2).normalized() * 3) - Vector3D(2.5, 1.5, 0)).limit(0.1) * 2;

    // Act
    Vector3D actualResult = rule(boidToUpdate, proximity, parameters);

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
    Boid              boidToUpdate = Boid(Vector3D(1.0, 1.0, 0), Vector3D(3.0, 3.0, 0));
    Boid              boid2        = Boid(Vector3D(0.0, 0.0, 0), Vector3D(1.0, 1.0, 0));
    std::vector<Boid> proximity{boid2};
    CohesionRule rule;


    // Averaged position of other boids is 0, 0
    Vector3D expectedResult
      = ((Vector3D(-1, -1, 0).normalized() * 3) - Vector3D(3, 3, 0)).limit(0.1);

    // Act
    Vector3D actualResult = rule(boidToUpdate, proximity, parameters);

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
    Boid              boidToUpdate = Boid(Vector3D(1.0, 1.0, 0), Vector3D(3.0, 3.0, 0));
    Boid              boid2        = Boid(Vector3D(2.0, 1.0, 0), Vector3D(1.0, 1.0, 0));
    Boid              boid3        = Boid(Vector3D(3.0, 3.5, 0), Vector3D(1.0, 1.0, 0));
    std::vector<Boid> proximity{ boid2, boid3 };
    CohesionRule      rule;

    // Averaged position of other boids is 0, 0
    Vector3D expectedResult = (((Vector3D(1.5, 1.25, 0)).normalized() * 3) - Vector3D(3, 3, 0)).limit(0.1) * 2;

    // Act
    Vector3D actualResult = rule(boidToUpdate, proximity, parameters);

    // Assert
    EXPECT_NEAR(expectedResult.x, actualResult.x, 1E-5);
    EXPECT_NEAR(expectedResult.y, actualResult.y, 1E-5);
}
