#include "utility/vector_operations.h"
#include "gtest/gtest.h"
#include <vector>


class VectorOperationsTest : public ::testing::Test {

protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(VectorOperationsTest, VectorEquality) {
  EXPECT_EQ(Vector2D(1, 2), Vector2D(1, 2));
  EXPECT_FALSE(Vector2D(1.00001, 2) == Vector2D(1, 2));
}

TEST_F(VectorOperationsTest, VectorSum) {
  Vector2D result = Vector2D(1, 2) + Vector2D(3, 4);
  EXPECT_EQ(result, Vector2D(4, 6));
}

TEST_F(VectorOperationsTest, VectorDiff) {
    Vector2D result = Vector2D(1, 2) - Vector2D(3, 4);
    EXPECT_EQ(result, Vector2D(-2, -2));
}

TEST_F(VectorOperationsTest, VectorMultiplyScalar) {
  Vector2D result = Vector2D(3, 4) * 0.5;
  EXPECT_EQ(result, Vector2D(1.5, 2));
}

TEST_F(VectorOperationsTest, Magnitude) {
  double magnitude = Vector2D(1, 3).magnitude();
  EXPECT_NEAR(magnitude, sqrt(10), 1E-10);
  magnitude = Vector2D(1, 50).magnitude();
  EXPECT_NEAR(magnitude, sqrt(2501), 1E-10);
}

TEST_F(VectorOperationsTest, Normalize) {
    Vector2D normalized = Vector2D(1, 3).normalized();
    EXPECT_NEAR(normalized.x, 1.0 / sqrt(10), 1E-10);
    EXPECT_NEAR(normalized.y, 3.0 / sqrt(10), 1E-10);
    EXPECT_NEAR(normalized.magnitude(), 1, 1E-10);
}

TEST_F(VectorOperationsTest, TestSteer) {
  Vector2D a = Vector2D(0, 1);
  Vector2D result = VectorOperations::steer(Vector2D(1, 0), Vector2D(0, 1), 50);
  EXPECT_EQ(result.x, 1);
  EXPECT_EQ(result.y, -1);
}
