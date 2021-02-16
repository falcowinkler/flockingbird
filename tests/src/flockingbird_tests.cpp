#include "gtest/gtest.h"
#include "flockingbird.h"

using namespace std;

class DividerTest : public ::testing::Test {

protected:

  virtual void SetUp() {
  };

  virtual void TearDown() {
  };

};

TEST_F(DividerTest, 5_DivideBy_2) {
  FlockSimulation::Flock flock = FlockSimulation::Flock();
  EXPECT_EQ(flock.do_something(), 42);
}
