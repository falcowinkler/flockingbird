#pragma once

#include "../utility/vector_operations.hpp"
#include <ostream>


namespace flockingbird {

class Boid {
public:
    Boid(Vector2D positionIn, Vector2D velocity)
        : position(positionIn)
        , velocity(velocity) {}
    Vector2D position;
    Vector2D velocity;

    friend std::ostream& operator<<(std::ostream& outputStream, const Boid& p);
};

inline std::ostream& operator<<(std::ostream& outputStream, const Boid& p) {
    outputStream << "(pos" << p.position << ") (dir: " << p.velocity << ")";
    return outputStream;
}

}
