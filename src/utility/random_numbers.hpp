#pragma once
#include <random>

inline float randomInBounds(float fMin, float fMax) {
    float f = (float)rand() / static_cast<float>(RAND_MAX);
    return fMin + f * (fMax - fMin);
}
