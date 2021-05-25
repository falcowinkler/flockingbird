#pragma once

struct FlockSimulationParameters {
    double speedLimit;
    double positionIncrementScalingFactor;
    double avoidanceRadius;
    double visionRange;
    FlockSimulationParameters(double speedLimitIn,
                              double positionIncrementScalingFactorIn,
                              double avoidanceRadiusIn,
                              double visionRangeIn)
        : speedLimit(speedLimitIn)
        , positionIncrementScalingFactor(positionIncrementScalingFactorIn)
        , avoidanceRadius(avoidanceRadiusIn)
        , visionRange(visionRangeIn) {}
};
