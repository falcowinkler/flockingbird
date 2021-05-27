#pragma once

struct FlockSimulationParameters {
    double speedLimit;
    double forceLimit;
    double positionIncrementScalingFactor;
    double avoidanceRadius;
    double visionRange;
    FlockSimulationParameters() {}
    FlockSimulationParameters(double speedLimitIn,
                              double forceLimitIn,
                              double positionIncrementScalingFactorIn,
                              double avoidanceRadiusIn,
                              double visionRangeIn)
        : speedLimit(speedLimitIn)
        , forceLimit(forceLimitIn)
        , positionIncrementScalingFactor(positionIncrementScalingFactorIn)
        , avoidanceRadius(avoidanceRadiusIn)
        , visionRange(visionRangeIn) {}
};
