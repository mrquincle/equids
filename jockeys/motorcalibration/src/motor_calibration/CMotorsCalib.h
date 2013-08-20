/*
 * OdCalib.h
 *
 *  Created on: 22.5.2013
 *      Author: Robert
 */

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_linalg.h>
#include <CMotors.h>
#include <cmath>

#ifndef ODCALIB_H_
#define ODCALIB_H_

using namespace std;
using namespace std;
enum RobotType
  {
      UNKNOWN,
      KABOT,
      ACTIVEWHEEL,
      SCOUTBOT
  };


class CMotorsCalib {
public:
	CMotorsCalib(RobotBase *robot_base, RobotBase::RobotType robot_type, CMotors* motor);
	virtual ~CMotorsCalib();
	void calibrate(DetectedBlob* detectedBlob);
	bool successful;
	bool sawLM;
	char calibstate;
	int filteriteration;
	bool readCalibResult();
private:
	int calibspeed;
	void calibrateAW(DetectedBlob* detectedBlob);
	void calibrateScout(DetectedBlob* detectedBlob);
	void calibrateKB(DetectedBlob* detectedBlob);
	void evaluateCalibrationS();
	void evaluateCalibrationKB();
	void evaluateCalibrationAW();
	void convertCameraMeasurementS(float*);
    void convertCameraMeasurementAW(float*,float);
    void convertCameraMeasurementKB(float*);
    void filter(float*,float*);
    void initializeCovariance();
    void turn();
    bool driveToCalibrationPosition();
    double euclideanDistance(double* position1, double* position2);
    float euclideanDistancef(float* position1, float* position2);
    float euclideanDistancefd(float* position1, double* position2);
	float firstmeasured[4];
	double firstodometry[5];
	float secondmeasured[4];
	double secondodometry[5];
	float thirdmeasured[4];
	double thirdodometry[5];
	float fourthmeasured[4];
	double fourthodometry[5];
	float fifthmeasured[4];
	double fifthodometry[5];
	double calibParam1;
	double calibParam2;
	CMotors* motor;
	double* odometry;
	RobotBase::RobotType robotype;
	RobotBase* robot;
	bool saveCalibResult(double calibresalt1,double calibresalt2,double calibresult3);
	gsl_matrix *Q;
	gsl_matrix *K;
	gsl_matrix *invers;
	gsl_matrix *P;
	gsl_matrix *Ppom;
	gsl_permutation *permut;
	bool already_see;
	int notsee;
	int stai_in_motion;
	int wasmoving;
};

#endif /* ODCALIB_H_ */
