/*
 * Mark.h
 *
 *  Created on: 15.11.2012
 *      Author: robert
 */
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_linalg.h>

#ifndef MARK_H_
#define MARK_H_

using namespace std;
typedef struct MapData {
		gsl_matrix * map;
		gsl_matrix * covariance;
	} MapData;

class Map {
public:
	Map(double*,int);
	virtual ~Map();
	gsl_matrix *state;
	gsl_matrix *predstate;
	int velikostmapy;
	static double PI;
	void filter(double*,double*,int);
	MapData readFromFile(const char* filename);
	int writeToFile(const char* filename,MapData data);
private:
//	double **actualMeasured;
	//chyba odometrie robota
	gsl_matrix *R;
	gsl_matrix *P;
	gsl_matrix *predP;
	gsl_matrix *secondAngle;
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

};

#endif /* MARK_H_ */
