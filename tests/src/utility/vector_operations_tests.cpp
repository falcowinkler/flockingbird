#include "utility/vector_operations.h"
#include "gtest/gtest.h"
#include <vector>

using namespace std;
using namespace FlockSimulation;

class VectorOperationsTest : public ::testing::Test {

protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(VectorOperationsTest, VectorSum) {
  Vector2D result = VectorOperations::vecSum(Vector2D(1, 2), Vector2D(3, 4));
  EXPECT_EQ(result.x, 4);
  EXPECT_EQ(result.y, 6);
}

TEST_F(VectorOperationsTest, VectorMultiplyScalar) {
  Vector2D result = VectorOperations::vecMulScalar(Vector2D(3, 4), 0.5);
  EXPECT_EQ(result.x, 1.5);
  EXPECT_EQ(result.y, 2);
}
