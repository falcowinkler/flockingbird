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

    double magnitude() { return sqrt(pow(x, 2) + pow(y, 2)); }

    Vector2D normalized() {
        double mag = magnitude();
        return Vector2D(x / mag, y / mag);
    }

  double distanceTo(Vector2D other) {
    double a = abs(x-other.x);
    double b = abs(y-other.y);
    return Vector2D(a, b).magnitude();
  }

  Vector2D limit(double maxForce) {
      if (magnitude() > maxForce * maxForce) {
          Vector2D norm =  normalized();
          return Vector2D(norm.x * maxForce, norm.y * maxForce);
      }
      return *this;
  }

    friend std::ostream& operator<<(std::ostream& outputStream, const Vector2D& p);
};

inline std::ostream& operator<<(std::ostream& outputStream, const Vector2D& p) {
    outputStream << "[" << p.x << ", " << p.y << "]";
    return outputStream;
}

inline bool operator==(Vector2D a, Vector2D b) { return a.x == b.x && a.y == b.y; }

inline Vector2D operator+(Vector2D a, Vector2D b) { return Vector2D(a.x + b.x, a.y + b.y); }

inline Vector2D operator-(Vector2D a, Vector2D b) {  return Vector2D(a.x - b.x, a.y - b.y);  }

inline Vector2D operator*(Vector2D a, double x) { return Vector2D(a.x * x, a.y * x); }
inline Vector2D operator/(Vector2D a, double x) { return Vector2D(a.x / x, a.y / x); }

namespace VectorOperations {

  inline Vector2D limitMagnitude(Vector2D steer, double maxForce) {
    if (steer.magnitude() > maxForce * maxForce) {
         steer = steer.normalized() * maxForce;
      }
      return steer;
  }

  inline Vector2D steer(Vector2D desired, Vector2D velocity, double maxForce) {
      Vector2D steer = desired - velocity;
      steer = limitMagnitude(steer, maxForce);
      return steer;
  }
}
