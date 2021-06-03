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
    float maxX = -1;
    float maxY = -1;

    FlockSimulationParameters() {}
    FlockSimulationParameters(float speedLimit,
                              float forceLimit,
                              float positionIncrementScalingFactor,
                              float avoidanceRadius,
                              float visionRange,
                              float separationWeight,
                              float alignmentWeight,
                              float cohesionWeight,
                              float maxX,
                              float maxY)
        : speedLimit(speedLimit)
        , forceLimit(forceLimit)
        , positionIncrementScalingFactor(positionIncrementScalingFactor)
        , avoidanceRadius(avoidanceRadius)
        , visionRange(visionRange)
        , separationWeight(separationWeight)
        , alignmentWeight(alignmentWeight)
      , cohesionWeight(cohesionWeight),
      maxX(maxX),
      maxY(maxY){}
};
}  // namespace flockingbird
