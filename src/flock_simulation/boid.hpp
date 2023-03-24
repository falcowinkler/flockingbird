#pragma once

#include "../utility/vector_operations.hpp"
#include <ostream>

namespace flockingbird {

class Boid {
public:
    Boid(Vector3D positionIn, Vector3D velocity)
        : position(positionIn)
        , velocity(velocity) {}
    Vector3D position;
    Vector3D velocity;

    friend std::ostream& operator<<(std::ostream& outputStream, const Boid& p);

private:

};

inline std::ostream& operator<<(std::ostream& outputStream, const Boid& p) {
    outputStream << "(pos" << p.position << ") (dir: " << p.velocity << ")";
    return outputStream;
}

}  // namespace flockingbird
