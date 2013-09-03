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
#include <vector>
#include <messageDataType.h>
#include <CMessage.h>

#ifndef MARK_H_
#define MARK_H_

using namespace std;
typedef struct MapData {
	gsl_matrix * map;
	gsl_matrix * covariance;
} MapData;

typedef struct OtherRobotMap {
	int robotID;
	std::vector<MappedObjectPosition> mappedObjects;
} OtherRobotMap;

class Map {
public:
	Map(double* robpos, RobotBase::RobotType robot_type);
	virtual ~Map();
	gsl_matrix *state;
	std::vector<MapObjectType> mappedObjectTypes;
	gsl_matrix *predstate;
	int mapSize;
	static double PI;
	void filter(double*, float*);
	void odometryChange(double*);
	MapData readFromFile(const char* filename);
	int writeToFile(const char* filename, MapData data);
	MappedObjectPosition* getMappedPosition(int ithLM);
	int nearestTypeID(double* position, MapObjectType type);
	int saveObjectToMap(MappedObjectPosition* position,
			MappedObjectCovariance* covariance);
	void addOtherRobotsObjects(CMessage message);
	gsl_matrix *P;
	bool newDetected;
	bool seeAfterLongTime;
	static void convertCameraMeasurementS(float *);
	static void convertCameraMeasurementKB(float *);
	static void convertCameraMeasurement(float *, float hinge,
			RobotBase::RobotType type);
	static void convertCameraMeasurementAW(float *, float);
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
	int lastSeen;
	double normalizeAngle(double);
	double normalizeAngleDiff(double);
	int minIndex(double*, int);
	void printMatrix(gsl_matrix*);
	void printArray(double *, int);

	void calculateOdometryCovariance(double changedx, double changedy,
			double changedphi, double changedDL, double changedDR);
	void calculateOdometryCovarianceAW(double dX, double dY, double dPHI);
	void calculateOdometryCovarianceKB(double dX, double dY, double dPHI);
	void calculateOdometryCovarianceS(double dL, double dR, double changedphi);
	std::vector<OtherRobotMap> otherMapData;
};

#endif /* MARK_H_ */
