#pragma once
#include <iostream>
#include <math.h>
#include <ostream>

#include <bit>
#include <cstdint>
#include <limits>

class Vector2D {
public:
    Vector2D(float xIn, float yIn)
        : x(xIn)
        , y(yIn) {}
    float x, y;

    float magnitude() { return sqrt(pow(x, 2) + pow(y, 2)); }

    Vector2D normalized() {
        float n      = pow(x, 2) + pow(y, 2);
        float length = sqrt(n);
        return Vector2D(x / length, y / length);
    }

    float distanceTo(Vector2D other) {
        float a = abs(x - other.x);
        float b = abs(y - other.y);
        return Vector2D(a, b).magnitude();
    }

    Vector2D limit(float maxForce) {
        if (magnitude() > maxForce * maxForce) {
            Vector2D norm = normalized();
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

inline Vector2D operator-(Vector2D a, Vector2D b) { return Vector2D(a.x - b.x, a.y - b.y); }

inline Vector2D operator*(Vector2D a, float x) { return Vector2D(a.x * x, a.y * x); }
inline Vector2D operator/(Vector2D a, float x) { return Vector2D(a.x / x, a.y / x); }
