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
    FlockSimulationParameters() {}
    FlockSimulationParameters(double speedLimitIn,
                              double forceLimitIn,
                              double positionIncrementScalingFactorIn,
                              double avoidanceRadiusIn,
                              double visionRangeIn,
                              double separationWeightIn,
                              double alignmentWeightIn,
                              double cohesionWeightIn)
        : speedLimit(speedLimitIn)
        , forceLimit(forceLimitIn)
        , positionIncrementScalingFactor(positionIncrementScalingFactorIn)
        , avoidanceRadius(avoidanceRadiusIn)
        , visionRange(visionRangeIn)
        , separationWeight(separationWeightIn)
        , alignmentWeight(alignmentWeightIn)
        , cohesionWeight(cohesionWeightIn){}
};
