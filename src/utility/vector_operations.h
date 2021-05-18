#pragma once
#include <iostream>
#include <ostream>
#include <math.h>

class Vector2D {
public:
    Vector2D(const Vector2D& other)
        : x(other.x)
        , y(other.y) {}
    Vector2D(double xIn, double yIn)
        : x(xIn)
        , y(yIn) {}
    double x, y;

    friend std::ostream& operator<<(std::ostream& outputStream, const Vector2D& p);
};

inline std::ostream& operator<<(std::ostream& outputStream, const Vector2D& p) {
    outputStream << "[" << p.x << ", " << p.y << "]";
    return outputStream;
}

namespace VectorOperations {

  inline Vector2D vecSum(Vector2D a, Vector2D b) {
    return Vector2D(a.x + b.x, a.y+b.y);
  }

  inline Vector2D vecDiff(Vector2D a, Vector2D b) { return Vector2D(a.x - b.x, a.y - b.y); }

  inline Vector2D vecMulScalar(Vector2D a, double scalar) {
    return Vector2D(a.x * scalar, a.y * scalar);
  }

  inline double magnitude(Vector2D a) {
    return sqrt(pow(a.x, 2) + pow(a.y, 2));
  }

  inline Vector2D normalize(Vector2D a) {
    double mag = magnitude(a);
    return Vector2D(a.x / mag, a.y / mag);
  }

  inline Vector2D limitMagnitude(Vector2D steer, double maxForce) {
      if (magnitude(steer) > maxForce * maxForce) {
         steer = vecMulScalar(normalize(steer), maxForce);
      }
      return steer;
  }

  inline Vector2D steer(Vector2D desired, Vector2D velocity, double maxForce) {
      Vector2D steer = vecDiff(desired,  velocity);
      steer = limitMagnitude(steer, maxForce);
      return steer;
  }
}
