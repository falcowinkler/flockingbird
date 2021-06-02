#pragma once

namespace flockingbird {
struct FlockSimulationParameters {
    double speedLimit;
    double forceLimit;
    double positionIncrementScalingFactor;
    double avoidanceRadius;
    double visionRange;
    double separationWeight;
    double alignmentWeight;
    double cohesionWeight;
    double maxX = -1;
    double maxY = -1;

    FlockSimulationParameters() {}
    FlockSimulationParameters(double speedLimit,
                              double forceLimit,
                              double positionIncrementScalingFactor,
                              double avoidanceRadius,
                              double visionRange,
                              double separationWeight,
                              double alignmentWeight,
                              double cohesionWeight,
                              double maxX,
                              double maxY)
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
