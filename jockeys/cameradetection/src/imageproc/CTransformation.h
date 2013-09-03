/*
 * File name: CTransformation.h
 * Date:      2005/11/07 18:10
 * Author:    
 */

#ifndef __CTRANSFORMATION_H__
#define __CTRANSFORMATION_H__

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "CCircleDetect.h"

typedef enum {
	TRANSFORM_NONE, TRANSFORM_2D, TRANSFORM_3D, TRANSFORM_NUMBER
} ETransformType;

typedef struct {
	float x, y, z, d;
	float pitch, roll, yaw;
	float roundness;
	float bwratio;
	float error;
} STrackedObject;

class CTransformation {
public:
	CTransformation(int widthi, int heighti, float diam, bool fullUnbarreli =
			false);
	~CTransformation();

	float barrelX(float x, float y);
	float barrelY(float x, float y);
	float unbarrelX(float x, float y);
	float unbarrelY(float x, float y);
	float transformX(float x, float y);
	float transformY(float x, float y);
	void transformXY(float *ix, float *iy);
	void setTrackedObjectDiameter(float diam);

	void unbarrel(unsigned char* src, unsigned char* dst);
	STrackedObject transform(SSegment segment, bool unbarrel);
	STrackedObject eigen(double data[]);
	int calibrate3D(STrackedObject *o, float gridDimX, float gridDimY);
	int calibrate2D(STrackedObject *o, float gridDimX, float gridDimY);
	ETransformType transformType;
	void saveCalibration(const char *str);
	void loadCalibration(const char *str);

private:
	STrackedObject normalize(STrackedObject o);
	float establishError(STrackedObject o);
	STrackedObject transform3D(STrackedObject o);
	STrackedObject transform2D(STrackedObject o);
	float *xArray;
	float *yArray;
	float *gArrayX;
	float *gArrayY;
	int *pArray;

	float to3D[3][3];
	float hom[9];
	STrackedObject orig3D;

	int width, height;
	bool fullUnbarrel;
	bool unbarrelInitialized;
	float trackedObjectDiameter;
	float kc[6];
	float fc[2];
	float cc[2];
};

#endif
/* end of CTransformation.h */
