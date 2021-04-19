#include "utility/vector_operations.h"
#include "gtest/gtest.h"
#include <vector>

using namespace std;

class VectorOperationsTest : public ::testing::Test {

protected:
    virtual void SetUp(){};

    virtual void TearDown(){};
};

TEST_F(VectorOperationsTest, VectorSum) {
  std::vector<double> a = std::vector<double>{4, 3, 2};
  std::vector<double> b = std::vector<double>{1.0, 3.2, 5};
  std::vector<double> sum = VectorOperations::vecSum(a, b);
  EXPECT_EQ(sum, (std::vector<double>{5, 6.2, 7}));
}

TEST_F(VectorOperationsTest, MultiplyScalar) {
    std::vector<double> a   = std::vector<double>{1.0, 3.2, 5};
    std::vector<double> sum = VectorOperations::multiplyScalar(a, 0.5);
    EXPECT_EQ(sum, (std::vector<double>{0.5, 1.6, 2.5}));
}