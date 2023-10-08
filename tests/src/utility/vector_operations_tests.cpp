#include "utility/vector_operations.hpp"
#include "gtest/gtest.h"
#include <vector>

class VectorOperationsTest : public ::testing::Test {

protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(VectorOperationsTest, VectorEquality) {
  EXPECT_EQ(Vector3D(1, 2, 0), Vector3D(1, 2, 0));
  EXPECT_FALSE(Vector3D(1.00001, 2, 0) == Vector3D(1, 2, 0));
}

TEST_F(VectorOperationsTest, VectorSum) {
  Vector3D result = Vector3D(1, 2, 0) + Vector3D(3, 4, 0);
  EXPECT_EQ(result, Vector3D(4, 6, 0));
}

TEST_F(VectorOperationsTest, VectorDiff) {
  Vector3D result = Vector3D(1, 2, 0) - Vector3D(3, 4, 0);
  EXPECT_EQ(result, Vector3D(-2, -2, 0));
}

TEST_F(VectorOperationsTest, VectorMultiplyScalar) {
  Vector3D result = Vector3D(3, 4, 0) * 0.5;
  EXPECT_EQ(result, Vector3D(1.5, 2, 0));
}

TEST_F(VectorOperationsTest, Magnitude) {
  float magnitude = Vector3D(1, 3, 0).magnitude();
  EXPECT_NEAR(magnitude, sqrt(10), 1E-5);
  magnitude = Vector3D(1, 50, 0).magnitude();
  EXPECT_NEAR(magnitude, sqrt(2501), 1E-5);
}

TEST_F(VectorOperationsTest, Normalize) {
  Vector3D normalized = Vector3D(1, 3, 0).normalized();
    EXPECT_NEAR(normalized.x, 1.0 / sqrt(10), 1E-2);
    EXPECT_NEAR(normalized.y, 3.0 / sqrt(10), 1E-2);
    EXPECT_NEAR(normalized.magnitude(), 1, 1E-2);

    Vector3D normalized2 = Vector3D(1, 1, 0).normalized();
    EXPECT_NEAR(normalized2.x, 1.0 / sqrt(2), 1E-2);
    EXPECT_NEAR(normalized2.y, 1.0 / sqrt(2), 1E-2);
    EXPECT_NEAR(normalized2.magnitude(), 1, 1E-2);
}

TEST_F(VectorOperationsTest, NormalizeNegativeNumbers) {
  Vector3D normalized = Vector3D(-1, -1, 0).normalized();
    EXPECT_NEAR(normalized.x, -1.0 / sqrt(2), 1E-3);
    EXPECT_NEAR(normalized.y, -1.0 / sqrt(2), 1E-3);
    EXPECT_NEAR(normalized.magnitude(), 1, 1E-3);
}

TEST_F(VectorOperationsTest, TestDistance) {
  Vector3D a = Vector3D(3, 2, 0);
  Vector3D b = Vector3D(9, 7, 0);
  EXPECT_NEAR(a.distanceTo(b), sqrt(61),1E-5);
}

TEST_F(VectorOperationsTest, TestLimitMagnitude) {
  Vector3D a = Vector3D(3, 2, 0);
  Vector3D limited = a.limit(300);
  EXPECT_EQ(a, limited);

  limited = a.limit(0.1);
  EXPECT_NEAR(limited.magnitude(), 0.1, 1E-3);
}
