#include "flock_simulation/rules.h"
#include "gtest/gtest.h"
#include <vector>

using namespace std;
using namespace FlockSimulation;

class RulesTest : public ::testing::Test {
public:
protected:
    RulesTest()
        : boidToUpdate(Boid(Point(0.0, 0.0), vector<double>{1.0, 1.0})) {

        Point point1 = Point(1.0, 2);
        Boid  boid1  = Boid(point1, vector<double>{1.0, 1.0});

        Point point2 = Point(2, 3);
        Boid  boid2  = Boid(point2, vector<double>{1.0, 1.0});

        Point point3 = Point(3, 4);
        Boid  boid3  = Boid(point3, vector<double>{1.0, 1.0});

        proximity.push_back(boid1);
        proximity.push_back(boid2);
        proximity.push_back(boid3);
    };
    std::vector<Boid> proximity;
    Boid boidToUpdate;
    virtual void TearDown(){};
};

TEST_F(RulesTest, TestAlingsVelocityToCenterOfMassOfTheNeighbors) {
  std::vector<double> velocityCorrection = Rules::cohesion(boidToUpdate, proximity);
}
