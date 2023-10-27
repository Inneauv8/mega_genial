// @file
// Created by Mathieu Durand on 2023-10-27.
//

#include "mathX.h"

/**
 * @brief Calculate the squared distance between two points (x1, y1) and (x2, y2).
 *
 * This function computes the squared Euclidean distance between two points in 2D space.
 *
 * @param x1 The x-coordinate of the first point.
 * @param y1 The y-coordinate of the first point.
 * @param x2 The x-coordinate of the second point.
 * @param y2 The y-coordinate of the second point.
 * @return The squared distance between the two points.
 */
double sqDist(double x1, double y1, double x2, double y2) {
    return sqDist(x2 - x1, y2 - y1);
}

/**
 * @brief Calculate the squared distance from the origin to a point (x, y).
 *
 * This function computes the squared Euclidean distance from the origin to a point in 2D space.
 *
 * @param x The x-coordinate of the point.
 * @param y The y-coordinate of the point.
 * @return The squared distance from the origin to the point.
 */
double sqDist(double x, double y) {
    return x * x + y * y;
}

/**
 * @brief Calculate the Euclidean distance between two points (x1, y1) and (x2, y2).
 *
 * This function computes the Euclidean distance between two points in 2D space.
 *
 * @param x1 The x-coordinate of the first point.
 * @param y1 The y-coordinate of the first point.
 * @param x2 The x-coordinate of the second point.
 * @param y2 The y-coordinate of the second point.
 * @return The Euclidean distance between the two points.
 */
double dist(double x1, double y1, double x2, double y2) {
    return sqrt(sqDist(x1, y1, x2, y2));
}

/**
 * @brief Calculate the Euclidean distance from the origin to a point (x, y).
 *
 * This function computes the Euclidean distance from the origin to a point in 2D space.
 *
 * @param x The x-coordinate of the point.
 * @param y The y-coordinate of the point.
 * @return The Euclidean distance from the origin to the point.
 */
double dist(double x, double y) {
    return sqrt(sqDist(x, y));
}

/**
 * @brief Map a value from one range to another.
 *
 * This function maps a value from one range to another using linear interpolation.
 *
 * @param value The value to be mapped.
 * @param minValue The minimum value of the original range.
 * @param maxValue The maximum value of the original range.
 * @param minResult The minimum value of the target range.
 * @param maxResult The maximum value of the target range.
 * @return The mapped value within the target range.
 */
double map(double value, double minValue, double maxValue, double minResult, double maxResult) {
    return ((value - min(minValue, maxValue)) * abs(maxResult - minResult)) / abs(maxValue - minValue) + min(minResult, maxResult);
}

/**
 * @brief Map a value from the range [0, maxValue] to another range [0, maxResult].
 *
 * This function maps a value from the range [0, maxValue] to another range [0, maxResult] using linear interpolation.
 *
 * @param value The value to be mapped.
 * @param maxValue The maximum value of the original range.
 * @param maxResult The maximum value of the target range.
 * @return The mapped value within the target range.
 */
double map(double value, double maxValue, double maxResult) {
    return map(value, 0, maxValue, 0, maxResult);
}

/**
 * @brief Clamp a value within a specified range.
 *
 * This function clamps a value to be within the specified range.
 *
 * @param value The value to be clamped.
 * @param minValue The minimum allowed value.
 * @param maxValue The maximum allowed value.
 * @return The clamped value within the specified range.
 */
double clamp(double value, double minValue, double maxValue) {
    return min(max(value, minValue), maxValue);
}

/**
 * @brief Wrap a value within a specified range.
 *
 * This function wraps a value to be within the specified range, creating a cyclic behavior.
 *
 * @param value The value to be wrapped.
 * @param minValue The minimum allowed value.
 * @param maxValue The maximum allowed value.
 * @return The wrapped value within the specified range.
 */
double wrap(double value, double minValue, double maxValue) {
    double trueMin = min(minValue, maxValue);
    double trueMax = max(minValue, maxValue);

    float interval = trueMax - trueMin;

    while (value >= trueMax) {
        value -= interval;
    }

    while(value < trueMin) {
        value += interval;
    }

    return value;
}

/**
 * Calculate the smallest difference in a wrapped range.
 *
 * This function calculates the smallest difference between two values in a
 * wrapped range, where the range is defined by minValue and maxValue. If the
 * absolute difference is greater than half the interval, it adjusts the
 * difference to ensure it's the smallest possible.
 *
 * @param value The first value.
 * @param targetValue The second value.
 * @param minValue The minimum value of the range.
 * @param maxValue The maximum value of the range.
 * @return The smallest difference between the two values.
 */
double smallestDifferenceInWrap(double value, double targetValue, int minValue, int maxValue) {
    double trueMin = min(minValue, maxValue);
    double trueMax = max(minValue, maxValue);

    float interval = trueMax - trueMin;

    value = wrap(value, trueMin, trueMax);
    targetValue = wrap(targetValue, trueMin, trueMax);

    float difference = targetValue - value;
    const float absoluteDifferrence = abs(difference);

    if (absoluteDifferrence > interval / 2.0) {
        difference += difference < 0 ? interval : - interval;
    }

    return difference;
}

/**
 * Calculate the smallest angle difference in a wrapped range.
 *
 * This function calculates the smallest angle difference between two angles
 * in a wrapped range from -π to π.
 *
 * @param angle The first angle.
 * @param targetAngle The second angle.
 * @return The smallest angle difference between the two angles.
 */
double smallestAngleDifference(double angle, double targetAngle) {
    return smallestDifferenceInWrap(angle, targetAngle, -M_PI, M_PI);
}