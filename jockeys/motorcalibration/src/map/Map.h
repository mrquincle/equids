/*
 * Mark.h
 *
 *  Created on: 15.11.2012
 *      Author: robert
 */
#include <IRobot.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_linalg.h>
#include <cmath.h>
#include <float.h>

#ifndef MARK_H_
#define MARK_H_

using namespace std;
typedef struct MapData {
		gsl_matrix * map;
		gsl_matrix * covariance;
	} MapData;

class Map {
public:
	Map(double*,RobotBase::RobotType robot_type);
	virtual ~Map();
	gsl_matrix *state;
	gsl_matrix *predstate;
	int velikostmapy;
	static double PI;
	void filter(double*,double*);
	void odometryChange(double*);
	MapData readFromFile(const char* filename);
	int writeToFile(const char* filename,MapData data);
	double* getLMposition(int ithLM);
	int nearestLMnumber(double* position);
	gsl_matrix *P;
private:
//	double **actualMeasured;
	//chyba odometrie robota
	//gsl_matrix *R;
	RobotBase::RobotType robot_type;
	gsl_matrix *predP;
	//chyba měření kamerou
	gsl_matrix *Q;
	double *odometry;
	int time;
	// sign is of {-1,1}
	int sign;
	double normalizeAngle(double );
	double normalizeAngleDiff(double);
	int minIndex(double* ,int );
	void printMatrix(gsl_matrix*);
	void printArray(double *,int );
	void convertCameraMeasurementS(double *);
	void convertCameraMeasurementKB(double *);
	void convertCameraMeasurement(double *,double hinge);
	void convertCameraMeasurementAW(double *,double);
	void calculateOdometryCovariance(double changedx,double changedy,double changedphi,double changedDL,double changedDR);
	void calculateOdometryCovarianceAW(double dX,double dY,double dPHI);
	void calculateOdometryCovarianceKB(double dX,double dY,double dPHI);
	void calculateOdometryCovarianceS(double dL,double dR,double changedphi);

};

#endif /* MARK_H_ */
