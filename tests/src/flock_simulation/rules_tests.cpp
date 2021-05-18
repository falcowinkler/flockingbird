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

TEST_F(RulesTest, TestAlingsVelocityToCenterOfMassOfTheNeighbors1) {
    Vector2D velocityCorrection = Rules::cohesion(boidToUpdate, proximity);
    // Average position of proximity:
    // (1 + 2 + 3) / 3 = 2
    // (2 + 3 + 4) / 3 = 3
    // pcj = (2, 3)
    // correction vector: (pcj - boidToUpdate.pos) / 100 = (2/3) / 100 = (2.0/100, 3.0/100)
    ASSERT_DOUBLE_EQ(velocityCorrection.x, 2.0 / 100);
    ASSERT_DOUBLE_EQ(velocityCorrection.y, 3.0 / 100);
}

TEST_F(RulesTest, TestCohesionNoNeigborsReturnsZeroVector) {
    Vector2D velocityCorrection = Rules::cohesion(boidToUpdate, std::vector<Boid>());
    ASSERT_DOUBLE_EQ(velocityCorrection.x, 0);
    ASSERT_DOUBLE_EQ(velocityCorrection.y, 0);
}

TEST_F(RulesTest, TestAlingsVelocityToCenterOfMassOfTheNeighbors2) {
    Boid     newBoid            = Boid(Vector2D(2, 4), Vector2D(0, 0));
    Vector2D velocityCorrection = Rules::cohesion(newBoid, proximity);
    /* Average position of proximity:
    (1 + 2 + 3) / 3 = 2
    (2 + 3 + 4) / 3 = 3
    pcj = (2, 3)
    correction vector: (pcj - boidToUpdate.pos) / 100 = (0/-1) / 100 = (0.0/100, -1/100) */
    ASSERT_DOUBLE_EQ(velocityCorrection.x, 0);
    ASSERT_DOUBLE_EQ(velocityCorrection.y, -1.0 / 100);
}

TEST_F(RulesTest, TestSeperationRepelsBoids1) {
    Vector2D velocityCorrection = Rules::seperation(boidToUpdate, proximity);
    /* Subtract each difference between boidToUpdate and a neighbor from zero vector.
     * d1 = (-1, -2)
     * d2 = (-2, -3)
     * d3 = (-3, -4)
     * (0, 0) - (-1, -2) - (-2, -3) - (-3, -4) = (6, 9)
     */
    ASSERT_DOUBLE_EQ(velocityCorrection.x, 6);
    ASSERT_DOUBLE_EQ(velocityCorrection.y, 9);
}

TEST_F(RulesTest, TestSeparationNoNeigborsReturnsZeroVector) {
    Vector2D velocityCorrection = Rules::seperation(boidToUpdate, std::vector<Boid>());
    ASSERT_DOUBLE_EQ(velocityCorrection.x, 0);
    ASSERT_DOUBLE_EQ(velocityCorrection.y, 0);
}

TEST_F(RulesTest, TestAlignmentAlignsBoidVelocityToNeighborhood) {
    Vector2D velocityCorrection = Rules::alignment(boidToUpdate, proximity);
    /* Average the velocity of other boids, and scale by factor 8
     * (4, 5.5) / 3 = (4/3, 1.8333)
     * ((4/3, 1.8333)-(1, 1)) / 8 = (0.333/8, .833/8)
     */
    ASSERT_DOUBLE_EQ(velocityCorrection.x, 1.0 / 3 / 8);
    ASSERT_DOUBLE_EQ(velocityCorrection.y, 0.833333333333333 / 8);
}

TEST_F(RulesTest, TestAlignmentNoNeigborsReturnsZeroVector) {
    Vector2D velocityCorrection = Rules::alignment(boidToUpdate, std::vector<Boid>());
    ASSERT_DOUBLE_EQ(velocityCorrection.x, 0);
    ASSERT_DOUBLE_EQ(velocityCorrection.y, 0);
}
