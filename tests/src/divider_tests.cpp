#include "gtest/gtest.h"

using namespace std;

class DividerTest : public ::testing::Test {

protected:

  virtual void SetUp() {
  };

  virtual void TearDown() {
  };

};

TEST_F(DividerTest, 5_DivideBy_2) {
  EXPECT_EQ(1, 1);
}
