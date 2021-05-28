#pragma once

struct FlockSimulationParameters {
    double speedLimit;
    double forceLimit;
    double positionIncrementScalingFactor;
    double avoidanceRadius;
    double visionRange;
    double separationWeight;
    double alignmentWeight;
    double cohesionWeight;
    double maxX;
    double maxY;

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
