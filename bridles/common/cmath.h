/*
 * File name: cmath.h
 * Date:      2005/10/17 19:16
 * Author:    
 */

#ifndef __CMATH_H__
#define __CMATH_H__
#include <math.h>
#include <cmath>
#include <random>

double max(double a, double b);
double min(double a, double b);
int log2(int value);
int exp2(int value);
int signof(int a);
double euclideanDistance(double* position1, double* position2);
float euclideanDistancef(float* position1, float* position2);
double normalizeAngle(double angle);
double normalizeAngleDiff(double diff);
double randFromTO(double min,double max);

#endif

/* end of cmath.h */
