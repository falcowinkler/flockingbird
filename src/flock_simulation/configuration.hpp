#pragma once

namespace flockingbird {
struct FlockSimulationParameters {
    float speedLimit;
    float forceLimit;
    float positionIncrementScalingFactor;
    float avoidanceRadius;
    float visionRange;
    float separationWeight;
    float alignmentWeight;
    float cohesionWeight;
    float avoidanceWeight;
    float maxX = -1;
    float maxY = -1;
    float maxZ = -1;

    FlockSimulationParameters() {}
    FlockSimulationParameters(float speedLimit,
                              float forceLimit,
                              float positionIncrementScalingFactor,
                              float avoidanceRadius,
                              float visionRange,
                              float separationWeight,
                              float alignmentWeight,
                              float cohesionWeight,
                              float avoidanceWeight,
                              float maxX,
                              float maxY,
                              float maxZ)
        : speedLimit(speedLimit)
        , forceLimit(forceLimit)
        , positionIncrementScalingFactor(positionIncrementScalingFactor)
        , avoidanceRadius(avoidanceRadius)
        , visionRange(visionRange)
        , separationWeight(separationWeight)
        , alignmentWeight(alignmentWeight)
        , cohesionWeight(cohesionWeight)
        , avoidanceWeight(avoidanceWeight)
        , maxX(maxX)
        , maxY(maxY)
        , maxZ(maxZ) {}
};
}  // namespace flockingbird
