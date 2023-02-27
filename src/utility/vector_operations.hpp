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

// 2D overloads
inline bool operator==(Vector2D a, Vector2D b) { return a.x == b.x && a.y == b.y; }

inline Vector2D operator+(Vector2D a, Vector2D b) { return Vector2D(a.x + b.x, a.y + b.y); }

inline Vector2D operator-(Vector2D a, Vector2D b) { return Vector2D(a.x - b.x, a.y - b.y); }

inline Vector2D operator*(Vector2D a, float x) { return Vector2D(a.x * x, a.y * x); }
inline Vector2D operator/(Vector2D a, float x) { return Vector2D(a.x / x, a.y / x); }

class Vector3D {
public:
    Vector3D(float xIn, float yIn, float zIn)
        : x(xIn)
        , y(yIn)
        , z(zIn) {}

    Vector3D()
        : x(0)
        , y(0)
        , z(0) {}
    float x, y, z;

    float magnitude() { return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)); }

    Vector3D normalized() {
        float n      = pow(x, 2) + pow(y, 2) + pow(z, 2);
        float length = sqrt(n);
        return Vector3D(x / length, y / length, z / length);
    }

    float distanceTo(Vector3D other) {
        float a = abs(x - other.x);
        float b = abs(y - other.y);
        float c = abs(z - other.z);
        return Vector3D(a, b, c).magnitude();
    }

    Vector3D limit(float maxForce) {
        if (magnitude() > maxForce * maxForce) {
            Vector3D norm = normalized();
            return Vector3D(norm.x * maxForce, norm.y * maxForce, norm.z * maxForce);
        }
        return *this;
    }

    friend std::ostream& operator<<(std::ostream& outputStream, const Vector3D& p);
};

inline std::ostream& operator<<(std::ostream& outputStream, const Vector3D& p) {
    outputStream << "[" << p.x << ", " << p.y << ", " << p.z << "]";
    return outputStream;
}

// 3d overloads
inline bool operator==(Vector3D a, Vector3D b) { return a.x == b.x && a.y == b.z && a.y == b.z; }

inline Vector3D operator+(Vector3D a, Vector3D b) {
    return Vector3D(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Vector3D operator-(Vector3D a, Vector3D b) {
    return Vector3D(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Vector3D operator*(Vector3D a, float x) { return Vector3D(a.x * x, a.y * x, a.z * x); }
inline Vector3D operator/(Vector3D a, float x) { return Vector3D(a.x / x, a.y / x, a.z / x); }
