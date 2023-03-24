#pragma once

namespace flockingbird {
struct FlockSimulationParameters {
    float    speedLimit;
    float    forceLimit;
    float    positionIncrementScalingFactor;
    float    avoidanceRadius;
    float    visionRange;
    float    separationWeight;
    float    alignmentWeight;
    float    cohesionWeight;
    float    avoidanceWeight;
    float    directionWeight;
    bool     twoD = false;
    float    maxX = -1;
    float    maxY = -1;
    float    maxZ = -1;
    Vector3D targetPosition;

    FlockSimulationParameters() {}
    FlockSimulationParameters(float    speedLimit,
                              float    forceLimit,
                              float    positionIncrementScalingFactor,
                              float    avoidanceRadius,
                              float    visionRange,
                              float    separationWeight,
                              float    alignmentWeight,
                              float    cohesionWeight,
                              float    avoidanceWeight,
                              float    dirWeight,
                              bool     twoDimensions,
                              float    maxX,
                              float    maxY,
                              float    maxZ,
                              Vector3D TargetPos)
        : speedLimit(speedLimit)
        , forceLimit(forceLimit)
        , positionIncrementScalingFactor(positionIncrementScalingFactor)
        , avoidanceRadius(avoidanceRadius)
        , visionRange(visionRange)
        , separationWeight(separationWeight)
        , alignmentWeight(alignmentWeight)
        , cohesionWeight(cohesionWeight)
        , avoidanceWeight(avoidanceWeight)
        , directionWeight(dirWeight)
        , twoD(twoDimensions)
        , maxX(maxX)
        , maxY(maxY)
        , maxZ(maxZ)
        , targetPosition(TargetPos) {}

};
}  // namespace flockingbird
