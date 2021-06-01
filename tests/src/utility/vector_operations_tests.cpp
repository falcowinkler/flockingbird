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

TEST_F(VectorOperationsTest, NormalizeNegativeNumbers) {
    Vector2D normalized = Vector2D(-1, -1).normalized();
    EXPECT_NEAR(normalized.x, -1.0 / sqrt(2), 1E-10);
    EXPECT_NEAR(normalized.y, -1.0 / sqrt(2), 1E-10);
    EXPECT_NEAR(normalized.magnitude(), 1, 1E-10);
}

TEST_F(VectorOperationsTest, TestDistance) {
  Vector2D a = Vector2D(3, 2);
  Vector2D b = Vector2D(9, 7);
  EXPECT_NEAR(a.distanceTo(b), sqrt(61),1E-10);
}

TEST_F(VectorOperationsTest, TestLimitMagnitude) {
  Vector2D a = Vector2D(3, 2);
  Vector2D limited = a.limit(300);
  EXPECT_EQ(a, limited);

  limited = a.limit(0.1);
  EXPECT_NEAR(limited.magnitude(), 0.1, 1E-10);
}
