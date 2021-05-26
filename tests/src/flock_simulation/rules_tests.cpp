#include "flock_simulation/rules.h"
#include "gtest/gtest.h"
#include <vector>

using namespace std;

class RulesTest : public ::testing::Test {
public:
protected:
    RulesTest()
        : boidToUpdate(Boid(Vector2D(0, 0), Vector2D(1.0, 1.0))) {
        Boid boid1 = Boid(Vector2D(1.0, 2), Vector2D(1.0, 1.0));
        Boid boid2 = Boid(Vector2D(2, 3), Vector2D(2, 1.0));
        Boid boid3 = Boid(Vector2D(3, 4), Vector2D(1.0, 3.5));
        proximity.push_back(boid1);
        proximity.push_back(boid2);
        proximity.push_back(boid3);
    };
    std::vector<Boid> proximity;
    Boid              boidToUpdate;
    virtual void      TearDown(){};
};
