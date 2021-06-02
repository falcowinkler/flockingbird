#pragma once

#include "../utility/vector_operations.hpp"
#include <ostream>


namespace flockingbird {

class Boid {
public:
    Boid(const Boid& other)
        : velocity(Vector2D(other.velocity))
        , position(Vector2D(other.position)) {}
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
