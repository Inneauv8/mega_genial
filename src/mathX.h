// @file
// Created by Mathieu Durand on 2023-10-27.
//

#include <math.h>

#ifndef MATHX_H
#define MATHX_H

double sqDist(double x1, double y1, double x2, double y2);
double sqDist(double x, double y);

double dist(double x1, double y1, double x2, double y2);
double dist(double x, double y);

double map(double value, double minValue, double maxValue, double minResult, double maxResult);
double map(double value, double maxValue, double maxResult);

double clamp(double value, double minValue, double maxValue);

double wrap(double value, double minValue, double maxValue);

double min(double x, double y);

double max(double x, double y);

double abs(double value);

#endif //MATHX_H
