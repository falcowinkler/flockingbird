#include "flock_simulation/rules.h"
#include "gtest/gtest.h"
#include <vector>

using namespace std;
using namespace FlockSimulation;

class RulesTest : public ::testing::Test {
public:
protected:
    RulesTest()
      : boidToUpdate(Boid(Vector2D(0, 0), Vector2D(1.0, 1.0))) {
        Boid boid1 = Boid(Vector2D(1.0, 2), Vector2D(1.0, 1.0));
        Boid boid2 = Boid(Vector2D(2, 3), Vector2D(1.0, 1.0));
        Boid  boid3  = Boid(Vector2D(3, 4), Vector2D(1.0, 1.0));
        proximity.push_back(boid1);
        proximity.push_back(boid2);
        proximity.push_back(boid3);
    };
    std::vector<Boid> proximity;
    Boid boidToUpdate;
    virtual void TearDown(){};
};

TEST_F(RulesTest, TestAlingsVelocityToCenterOfMassOfTheNeighbors) {
    Vector2D velocityCorrection = Rules::cohesion(boidToUpdate, proximity);
    // Average position of proximity:
    // (1 + 2 + 3) / 3 = 2
    // (2 + 3 + 4) / 3 = 3
    // pcj = (2, 3)
    // correction vector: (pcj - boidToUpdate.pos) / 100 = (2/3) / 100 = (2.0/100, 3.0/100)
    ASSERT_DOUBLE_EQ(velocityCorrection.x, 2.0/100);
    ASSERT_DOUBLE_EQ(velocityCorrection.y, 3.0/100);
}
