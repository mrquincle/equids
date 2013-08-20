#include "cmath.h"
//#define M_PI = 3.14159265;

double max(double a, double b)
{
	if (a < b) return b;
	return a;
}

double min(double a, double b)
{
	if (b < a) return b;
	return a;
}

int log2(int value)
{
	int r=0;
	while (value > 0){
		value = value/2;
		r++;
	}
	return r-1;
}									 

int exp2(int value)
{
	int r=1;
	for (int i = 0;i<value;i++){
		r=r*2;
	}
	return r;
}

int signof(int a)
{
	return (a == 0) ? 0 : (a<0 ? -1 : 1);
}

double normalizeAngle(double angle){
	double ret = fmod(angle,2*M_PI);
	if(ret> M_PI){
		ret=ret-2* M_PI;
	}else{
		if(ret<- M_PI){
			ret=ret+2* M_PI;
		}
	}
	return ret;
}

double euclideanDistance(double* position1, double* position2){
return sqrt( pow(position1[0]-position2[0],2) + pow(position1[1]-position2[1],2));
}

float euclideanDistancef(float* position1, float* position2) {
	return sqrt(
			pow(position1[0] - position2[0], 2)
					+ pow(position1[1] - position2[1], 2));
}

double normalizeAngleDiff(double diff){
	double ret = fabs(diff);
	if(ret>M_PI){
		ret=fabs(ret-2*M_PI);
	}
	return ret;
}

double randFromTO(double min,double max){
	return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
}
