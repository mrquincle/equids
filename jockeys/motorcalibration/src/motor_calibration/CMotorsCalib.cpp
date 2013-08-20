/*
 * CMotorsCalib.cpp
 *
 *  Created on: 22.5.2013
 *      Author: robert
 */

#include "CMotorsCalib.h"
#define FILTER_CAUNT 20
#define MIN_DISTANCE 0.2
#define PI 3.141592654
//#define DEBUGODOCALIB
#define MAX_NOT_SEE 5
#define TURNING_COUNT 6
#define WAIT_STOPPED 20
#define DEBUGODOCALIB_RESULTS

CMotorsCalib::CMotorsCalib(RobotBase *robot_base,
		RobotBase::RobotType robot_type, CMotors* motor) {
	this->successful = false;
	this->robotype = robot_type;
	this->robot = robot_base;
	this->motor = motor;
	this->wasmoving = WAIT_STOPPED;
	this->calibstate = 0;
	this->filteriteration = 0;
	this->notsee = MAX_NOT_SEE;
	this->stai_in_motion = TURNING_COUNT;
	printf("inside calibration\n");
	K = gsl_matrix_calloc(4, 4);
	invers = gsl_matrix_calloc(4, 4);
	P = gsl_matrix_calloc(4, 4);
	Ppom = gsl_matrix_calloc(4, 4);
	Q = gsl_matrix_calloc(4, 4);
	permut = gsl_permutation_alloc(4);
	gsl_matrix_set(Q, 0, 0, 0.0002);
	gsl_matrix_set(Q, 1, 1, 0.015);
	//gsl_matrix_set(Q, 2, 2, 0.02);
	gsl_matrix_set(Q, 2, 2, 0.04);
	gsl_matrix_set(Q, 3, 3, 0.0005);

	switch (robotype) {
	case KABOT:
		calibspeed = 40;
		break;
	case SCOUTBOT:
		calibspeed = 40;
		break;
	case ACTIVEWHEEL:
		calibspeed = 60;
		break;
	default:
		break;
	}

}

CMotorsCalib::~CMotorsCalib() {
	gsl_matrix_free(K);
	gsl_matrix_free(invers);
	gsl_matrix_free(P);
	gsl_matrix_free(Q);
	gsl_permutation_free(permut);
}

void CMotorsCalib::calibrate(DetectedBlob* detectedBlob) {
	//printf("incalibrating\n");
	switch (robotype) {
	case KABOT:
		//this->selectedCalibrate
		calibrateKB(detectedBlob);
		break;
	case SCOUTBOT:
		calibrateScout(detectedBlob);
		break;
	case ACTIVEWHEEL:
		//	printf("AW\n");
		calibrateAW(detectedBlob);
		break;
	default:
		break;
	}
}

void CMotorsCalib::calibrateAW(DetectedBlob* detectedBlob) {
	//renew image
#if defined(DEBUGODOCALIB)
	//printf("calibrate Active Wheel\n");
#endif
	if (detectedBlob != NULL) {

		notsee = MAX_NOT_SEE;
#if defined(DEBUGODOCALIB)
		printf("valid segment\n");
#endif

		float measured[4] = { detectedBlob->x, detectedBlob->y,
				detectedBlob->phi, detectedBlob->z };
		CMotorsCalib::convertCameraMeasurementAW(measured,
				motor->getPosition()[5]);
#if defined(DEBUGODOCALIB)
		printf("measured=[measured [ %f ; %f ; %f ; %f ]] \n", measured[0],
				measured[1], measured[2], measured[3]);
#endif
		switch (calibstate) {
		case 0: { //camera see blob for first time
			//jed na pozici pred blobem
			//this->driveToCalibrationPosition();
			calibstate = 1;
			motor->setMotorSpeedsAW(0, 0, 0);
		}
			;
			break;
		case 1: { //see first landmark
				  //wait until enough measurements
			motor->setMotorSpeedsAW(0, 0, 0);
			if (filteriteration < FILTER_CAUNT) {
				if (filteriteration > 0) {
					CMotorsCalib::filter(firstmeasured, measured);
				} else {
					memcpy(firstmeasured, measured, 4 * sizeof(float));
					initializeCovariance();
					already_see = true;
#if defined(DEBUGODOCALIB)
					printf("first measured: [ %f ; %f ; %f ; %f ]] \n",
							measured[0], measured[1], measured[2], measured[3]);
#endif
				}
				filteriteration += 1;
			} else {
				CMotorsCalib::filter(firstmeasured, measured);
				calibstate = 2;
				already_see = false;
				filteriteration = 0;
				memcpy(firstodometry, motor->getPosition(), 5 * sizeof(double));
			}

		}
			;
			break;
		case 2: {											    //drive forward
			if (this->euclideanDistance(motor->getPosition(),
					firstodometry) > MIN_DISTANCE) {
				//enought distance to resolve or lost in less distance
				calibstate = 3;
				motor->setMotorSpeedsAW(0, 0, 0);
			} else {
				//if see landmark and do not travel enough distance -> continue
				motor->setSpeeds(calibspeed, 0);

			}

		}
			;
			break;
		case 3: {								//see first landmark second time
			motor->setMotorSpeedsAW(0, 0, 0);
			if (filteriteration < FILTER_CAUNT) {
				if (filteriteration > 0) {
					CMotorsCalib::filter(secondmeasured, measured);
				} else {
					memcpy(secondmeasured, measured, 4 * sizeof(float));
					initializeCovariance();
					already_see = true;
#if defined(DEBUGODOCALIB)
					printf("second: [ %f ; %f ; %f ; %f ]] \n", measured[0],
							measured[1], measured[2], measured[3]);
#endif
				}
				filteriteration += 1;
			} else {
				CMotorsCalib::filter(secondmeasured, measured);
				calibstate = 4;
				already_see = false;
				filteriteration = 0;
				memcpy(secondodometry, motor->getPosition(),
						5 * sizeof(double));
			}

		}
			;
			break;
		case 4: {											    //back
			if (this->euclideanDistance(motor->getPosition(),
					secondodometry) > MIN_DISTANCE) {
				//enought distance to resolve or lost in less distance
				motor->setMotorSpeedsAW(0, 0, 0);
				calibstate = 5;
			} else {
				//if see second landmark continue
				motor->setSpeeds(-calibspeed, 0);
//				motor->setMotorSpeedsAW(calibspeed/2, calibspeed/2,-calibspeed);
			}

		}
			;
			break;
		case 5: {								//see first landmark third time

			motor->setMotorSpeedsAW(0, 0, 0);
			if (filteriteration < FILTER_CAUNT) {
				if (filteriteration > 0) {
					CMotorsCalib::filter(thirdmeasured, measured);
				} else {
					memcpy(thirdmeasured, measured, 4 * sizeof(float));
					initializeCovariance();
					already_see = true;
#if defined(DEBUGODOCALIB)
					printf("third: [ %f ; %f ; %f ; %f ]] \n", measured[0],
							measured[1], measured[2], measured[3]);
#endif
				}

				filteriteration += 1;
			} else {
				CMotorsCalib::filter(thirdmeasured, measured);
				calibstate = 6;
				already_see = false;
				filteriteration = 0;
				memcpy(thirdodometry, motor->getPosition(), 5 * sizeof(double));
			}

		}
			;
			break;
		case 6: {		//turning

			if ((motor->getPosition()[2] - secondodometry[2] < 4.71238898)
					|| (std::abs(
							euclideanDistancefd(thirdmeasured,
									motor->getPosition())
									- euclideanDistancefd(measured,
											motor->getPosition())) > 0.3) ) {
				//if do not travel enough or measured landmark is not the same
				if ((motor->isMoving() || this->wasmoving > 0)
						&& stai_in_motion < 1) {	//wait two times
					motor->setSpeeds(0, 0);
					this->wasmoving -= 1;
					if (this->wasmoving < 1) {
						this->stai_in_motion = TURNING_COUNT;
					}
				} else {
					motor->setSpeeds(0, calibspeed);
					this->wasmoving = WAIT_STOPPED;
					this->stai_in_motion -= 1;
				}
			} else {
				//detected same landmark again
				motor->setSpeeds(0, 0);
				calibstate = 7;
			}

		}
			;
			break;
		case 7: {								//see first landmark third time
			motor->setSpeeds(0, 0);

			if (filteriteration < FILTER_CAUNT) {
				if (filteriteration > 0) {	//first measurement after stop
					CMotorsCalib::filter(fourthmeasured, measured);
				} else {
					memcpy(fourthmeasured, measured, 4 * sizeof(float));
					already_see = true;
					initializeCovariance();
#if defined(DEBUGODOCALIB)
					printf("fourth: [ %f ; %f ; %f ; %f ]] \n", measured[0],
							measured[1], measured[2], measured[3]);
#endif
				}
				filteriteration += 1;
			} else {
				CMotorsCalib::filter(fourthmeasured, measured);
				successful = true;
				already_see = false;
				filteriteration = 0;
				memcpy(fourthodometry, motor->getPosition(),
						5 * sizeof(double));
				this->evaluateCalibrationAW();
			}

		}
			;
			break;
		}
	} else {
		notsee -= 1;
		switch (calibstate) {
		case 0: {						//before calibration start
			// otacej();				//implement random walk
			// motor->randomSpeeds();
			//insert here random walk if after turning full circle no detection!!

		}
			;
			break;
		case 1: {							//misdetection

		}
			;
			break;
		case 2: {							//lost first landmark - return back
			if (notsee > 0) {
				//nothink to be done
			} else if (already_see && wasmoving > 0) {
				motor->setSpeeds(-calibspeed, 0);
				wasmoving = wasmoving + 1;
			} else {
				wasmoving = 0;
				motor->setSpeeds(0, 0);
			}

		}
			;
			break;
		case 3: {							//see first landmark second time
											//missdetection
		}
			;
			break;
		case 4: {						//lost first landmark - return forward
			if (notsee > 0) {
				//nothink to be done
			} else if (already_see && wasmoving > 0) {
				motor->setSpeeds(calibspeed, 0);
				wasmoving = wasmoving + 1;
			} else {
				wasmoving = 0;
				motor->setSpeeds(0, 0);
			}

		}
			;
			break;
		case 5: {							    //see first landmark third time
												//missdetection
		}
			;
			break;
		case 6: {
#if defined(DEBUGODOCALIB)
			printf("rozdil %f already_seen:%s was moving:%d , stai_in_motion:%d\n",
					motor->getPosition()[2] - thirdodometry[2],
					(already_see) ? "true" : "false", this->wasmoving,this->stai_in_motion);
#endif

			if (!already_see) {
				//before see same landmark
				if ((motor->isMoving() || this->wasmoving > 0)
						&& stai_in_motion < 1) {	//wait two times
					motor->setSpeeds(0, 0);
					this->wasmoving -= 1;
					if (this->wasmoving < 1) {
						this->stai_in_motion = TURNING_COUNT;
					}
				} else {
					motor->setSpeeds(0, calibspeed);
					this->wasmoving = WAIT_STOPPED;
					this->stai_in_motion -= 1;
				}
			} else if (already_see) {
				//already see same landmark
				if ((motor->isMoving() || this->wasmoving < 10)
						&& stai_in_motion < 1) {	//wait two times
					motor->setSpeeds(0, 0);
					this->wasmoving = this->wasmoving + 1;
				} else {
					motor->setSpeeds(0, -calibspeed);
					this->wasmoving = 0;
				}
			} else {
				motor->setSpeeds(0, 0);
				printf("no moving\n");
			}
		}
			;
			break;
		case 7: {							    //see first landmark third time
												//missdetection
			motor->setSpeeds(0, 0);
		}
			;
			break;
		}
	}
}

void CMotorsCalib::calibrateKB(DetectedBlob* detectedBlob) {
	//renew image

	if (detectedBlob != NULL) {
		if (detectedBlob->x > -12 && detectedBlob->x < 12
				&& detectedBlob->y > -12 && detectedBlob->y < 12
				&& detectedBlob->phi > -2 && detectedBlob->phi < 2
				&& detectedBlob->z > -10 && detectedBlob->z < 10) {

			float measured[4] = { detectedBlob->x, detectedBlob->y,
					detectedBlob->phi, detectedBlob->z };
			CMotorsCalib::convertCameraMeasurementKB(measured);
			switch (calibstate) {
			case 0: {							//camera see blob for first time
				calibstate = 1;
				motor->setMotorSpeedsKB(0, 0);
				memcpy(firstmeasured, measured, 4 * sizeof(float));
				initializeCovariance();
			}
				;
				break;
			case 1: {								//see first landmark
													//wait until enough measurements

				if (filteriteration < FILTER_CAUNT) {
					filteriteration += 1;
				} else {
					calibstate = 2;
					filteriteration = 0;
					memcpy(firstodometry, motor->getPosition(),
							5 * sizeof(double));
				}
				CMotorsCalib::filter(firstmeasured, measured);
				motor->setMotorSpeedsKB(0, 0);
			}
				;
				break;
			case 2: {						//sliding left until 10cm distance
				if (motor->getPosition()[1] - firstodometry[1] > MIN_DISTANCE
						|| (motor->actualspeed1 == 50
								&& motor->actualspeed2 == -50)) {
					//enought distance to resolve or lost in less distance
					calibstate = 3;
					motor->setMotorSpeedsKB(0, 0);
					memcpy(secondmeasured, measured, 4 * sizeof(float));
					initializeCovariance();
				} else {
					//if see landmark and do not travel enough distance -> continue
					motor->setMotorSpeedsKB(-50, 50);
				}

			}
				;
				break;
			case 3: {							//see first landmark second time

				if (filteriteration < FILTER_CAUNT) {
					filteriteration += 1;
				} else {
					calibstate = 4;
					filteriteration = 0;
					memcpy(secondodometry, motor->getPosition(),
							5 * sizeof(double));
				}
				CMotorsCalib::filter(secondmeasured, measured);
				motor->setMotorSpeedsKB(0, 0);

			}
				;
				break;
			case 4: {						//sliding right until 10cm distance
				if (motor->getPosition()[1] - secondodometry[1] < -MIN_DISTANCE
						|| (motor->actualspeed1 == -50
								&& motor->actualspeed2 == 50)) {
					calibstate = 5;
					memcpy(thirdmeasured, measured, 4 * sizeof(float));
					initializeCovariance();
					motor->setMotorSpeedsKB(0, 0);
				} else {
					//if see second landmark continue
					motor->setMotorSpeedsKB(50, -50);
				}

			}
				;
				break;
			case 5: {							//see first landmark third time
				CMotorsCalib::filter(thirdmeasured, measured);
				motor->setMotorSpeedsKB(0, 0);
				if (filteriteration < FILTER_CAUNT) {
					filteriteration += 1;
				} else {
					calibstate = 6;
					filteriteration = 0;
					memcpy(thirdodometry, motor->getPosition(),
							5 * sizeof(double));

				}

			}
				;
				break;
			case 6: {						//sliding back until 10cm distance
				if (motor->getPosition()[0] - thirdodometry[0] > MIN_DISTANCE
						|| (motor->actualspeed1 == 50
								&& motor->actualspeed2 == 50)) {
					//enought distance to resolve or lost in less distance
					calibstate = 7;
					motor->setMotorSpeedsKB(0, 0);
					memcpy(fourthmeasured, measured, 4 * sizeof(float));
					initializeCovariance();
				} else {
					//if see landmark and do not travel enough distance -> continue
					motor->setMotorSpeedsKB(-50, -50);
				}

			}
				;
				break;
			case 7: {							//see first landmark second time

				if (filteriteration < FILTER_CAUNT) {
					filteriteration += 1;
				} else {
					calibstate = 8;
					filteriteration = 0;
					memcpy(fourthodometry, motor->getPosition(),
							5 * sizeof(double));
				}
				CMotorsCalib::filter(fourthmeasured, measured);
				motor->setMotorSpeedsKB(0, 0);

			}
				;
				break;
			case 8: {					//sliding forward until 10cm distance
				if (motor->getPosition()[0] - fourthodometry[0] > MIN_DISTANCE
						|| (motor->actualspeed1 == -50
								&& motor->actualspeed2 == -50)) {
					calibstate = 9;
					memcpy(fifthmeasured, measured, 4 * sizeof(float));
					initializeCovariance();
					motor->setMotorSpeedsKB(0, 0);
				} else {
					//if see second landmark continue
					motor->setMotorSpeedsKB(50, 50);
				}

			}
				;
				break;
			case 9: {							//see first landmark last time
				CMotorsCalib::filter(fifthmeasured, measured);
				if (filteriteration < FILTER_CAUNT) {
					filteriteration += 1;
				} else {
					successful = true;
					filteriteration = 0;
					memcpy(fifthodometry, motor->getPosition(),
							5 * sizeof(double));
					evaluateCalibrationKB();
				}
				motor->setMotorSpeedsKB(0, 0);

			}
				;
				break;
			}

		}
	} else {
		switch (calibstate) {
		case 0: {									  //before calibration start
													  //implement random walk
			motor->setMotorSpeedsKB(0, 0);

		}
			;
			break;
		case 1: {					    //misdetection

		}
			;
			break;
		case 2: {					   //lost first landmark - return back right
			motor->setMotorSpeedsKB(50, -50);

		}
			;
			break;
		case 3: {					    //see first landmark second time
										//missdetection
		}
			;
			break;
		case 4: {					   //lost first landmark - return back right
			motor->setMotorSpeedsKB(-50, 50);

		}
			;
			break;
		case 5: {					    //see first landmark third time
										//missdetection
		}
			;
			break;
		case 6: {					//lost first landmark - return back forward
			motor->setMotorSpeedsKB(50, 50);

		}
			;
			break;
		case 7: {						//see first landmark second time
										//missdetection
		}
			;
			break;
		case 8: {					    //lost first landmark - return back back
			motor->setMotorSpeedsKB(-50, -50);

		}
			;
			break;
		case 9: {					    //see first landmark third time
										//missdetection
		}
			;
			break;
		}
	}
}

void CMotorsCalib::calibrateScout(DetectedBlob* detectedBlob) {
	//renew image
#if defined(DEBUGODOCALIB)
	printf("calibrate scout\n");
#endif
	if (detectedBlob != NULL) {
		notsee = MAX_NOT_SEE;
#if defined(DEBUGODOCALIB)
		printf("valid segment\n");
#endif

		if (detectedBlob->x > -12 && detectedBlob->x < 12
				&& detectedBlob->y > -12 && detectedBlob->y < 12
				&& detectedBlob->phi > -2 && detectedBlob->phi < 2
				&& detectedBlob->z > -10 && detectedBlob->z < 10) {
			printf("inside\n");

			float measured[4] = { detectedBlob->x, detectedBlob->y,
					detectedBlob->phi, detectedBlob->z };
			CMotorsCalib::convertCameraMeasurementS(measured);
#if defined(DEBUGODOCALIB)
			printf("measured=[measured [ %f ; %f ; %f ; %f ]] \n", measured[0],
					measured[1], measured[2], measured[3]);
#endif
			switch (calibstate) {
			case 0: {						//camera see blob for first time
				//jed na pozici pred blobem
				//this->driveToCalibrationPosition();
				calibstate = 1;
				motor->setMotorSpeedsS(0, 0);

			}
				;
				break;
			case 1: {						//see first landmark
											//wait until enough measurements
				motor->setMotorSpeedsS(0, 0);
				if (filteriteration < FILTER_CAUNT) {
					if (filteriteration > 0) {
						CMotorsCalib::filter(firstmeasured, measured);
					} else {					//first measurement after stop
						memcpy(firstmeasured, measured, 4 * sizeof(float));
						initializeCovariance();
						already_see = true;
#if defined(DEBUGODOCALIB)
						printf("first measured: [ %f ; %f ; %f ; %f ]] \n",
								measured[0], measured[1], measured[2],
								measured[3]);
#endif
					}

					filteriteration += 1;
				} else {
					CMotorsCalib::filter(firstmeasured, measured);
					calibstate = 2;
					already_see = false;
					filteriteration = 0;
					memcpy(firstodometry, motor->getPosition(),
							5 * sizeof(double));
				}

			}
				;
				break;
			case 2: {								    //driving forvard
				if (this->euclideanDistance(motor->getPosition(),
						firstodometry) > MIN_DISTANCE) {
					calibstate = 3;
					motor->setMotorSpeedsS(0, 0);

				} else {
					//if see second landmark continue
					motor->setMotorSpeedsS(calibspeed, -calibspeed);

				}

			}
				;
				break;
			case 3: {							//see first landmark second time
				motor->setMotorSpeedsS(0, 0);
				if (filteriteration < FILTER_CAUNT) {
					if (filteriteration > 0) {
						CMotorsCalib::filter(secondmeasured, measured);
					} else {					//first measurement after stop
						memcpy(secondmeasured, measured, 4 * sizeof(float));
						initializeCovariance();
						already_see = true;
#if defined(DEBUGODOCALIB)
						printf("second: [ %f ; %f ; %f ; %f ]] \n", measured[0],
								measured[1], measured[2], measured[3]);
#endif
					}
					filteriteration += 1;
				} else {
					CMotorsCalib::filter(secondmeasured, measured);
					calibstate = 4;
					already_see = false;
					filteriteration = 0;
					memcpy(secondodometry, motor->getPosition(),
							5 * sizeof(double));
				}

			}
				;
				break;
			case 4: {								    //driving back
				if (this->euclideanDistance(motor->getPosition(),
						secondodometry) > MIN_DISTANCE) {
					motor->setMotorSpeedsS(0, 0);
					calibstate = 5;
				} else {
					//if see second landmark continue
					motor->setMotorSpeedsS(-calibspeed, calibspeed);
				}

			}
				;
				break;
			case 5: {							//see first landmark third time

				motor->setMotorSpeedsS(0, 0);
				if (filteriteration < FILTER_CAUNT) {
					if (filteriteration > 0) {
						CMotorsCalib::filter(thirdmeasured, measured);
					} else {					//first measurement after stop
						memcpy(thirdmeasured, measured, 4 * sizeof(float));
						already_see = true;
						initializeCovariance();
#if defined(DEBUGODOCALIB)
						printf("third: [ %f ; %f ; %f ; %f ]] \n", measured[0],
								measured[1], measured[2], measured[3]);
#endif
					}
					filteriteration += 1;
				} else {
					CMotorsCalib::filter(thirdmeasured, measured);
					calibstate = 6;
					already_see = false;
					filteriteration = 0;
					memcpy(thirdodometry, motor->getPosition(),
							5 * sizeof(double));
				}

			}
				;
				break;

			case 6: {								    //turn around

				if ((motor->getPosition()[2] - secondodometry[2] < 4.71238898)
						|| (std::abs(
								euclideanDistancefd(thirdmeasured,
										motor->getPosition())
										- euclideanDistancefd(measured,
												motor->getPosition())) > 0.3) ) {
					//if do not travel enough or measured landmark is not the same
					if ((motor->isMoving() || this->wasmoving > 0)
							&& stai_in_motion < 1) {	//wait two times
						motor->setSpeeds(0, 0);
						this->wasmoving -= 1;
						if (this->wasmoving < 1) {
							this->stai_in_motion = TURNING_COUNT;
						}
					} else {
						motor->setSpeeds(0, calibspeed);
						this->wasmoving = WAIT_STOPPED;
						this->stai_in_motion -= 1;
					}
				} else {
					//detected same landmark again
					motor->setSpeeds(0, 0);
					calibstate = 7;
				}

			}
				;
				break;
			case 7: {							//see first landmark third time

				motor->setMotorSpeedsS(0, 0);
				if (filteriteration < FILTER_CAUNT) {
					if (filteriteration > 0) {	//first measurement after stop
						CMotorsCalib::filter(fourthmeasured, measured);
					} else {
						memcpy(fourthmeasured, measured, 4 * sizeof(float));
						already_see = true;
						initializeCovariance();
#if defined(DEBUGODOCALIB)
						printf("fourth: [ %f ; %f ; %f ; %f ]] \n", measured[0],
								measured[1], measured[2], measured[3]);
#endif
					}
					filteriteration += 1;
				} else {
					CMotorsCalib::filter(fourthmeasured, measured);
					successful = true;
					already_see = false;
					filteriteration = 0;
					memcpy(fourthodometry, motor->getPosition(),
							5 * sizeof(double));
					this->evaluateCalibrationS();

				}

			}
				;
				break;
			}

		}
	} else {
		switch (calibstate) {
		case 0: {								    //before calibration start
			//	otacej();										//until no blob in camera vision turning slow left

			//	motor->randomSpeeds();
			//insert here random walk if after turning full circle no detection!!

			//

		}
			;
			break;
		case 1: {				//see first landmark
								//missdetection
		}
			;
			break;
		case 2: {			    	//must go back
			if (!already_see && wasmoving > 0) {
				motor->setMotorSpeedsS(-calibspeed, calibspeed);
				wasmoving = wasmoving + 1;
			} else {
				wasmoving = 0;
				motor->setMotorSpeedsS(0, 0);
			}
		}
			;
			break;
		case 3: {			    	//see first landmark second time
									//missdetection
		}
			;
			break;
		case 4: {				    //must go forward
			if (!already_see && wasmoving > 0) {
				motor->setMotorSpeedsS(calibspeed, -calibspeed);
				wasmoving = wasmoving + 1;
			} else {
				wasmoving = 0;
				motor->setMotorSpeedsS(0, 0);
			}
		}
			;
			break;
		case 5: {				    //see first landmark third time
									//missdetection
		}
			;
			break;
		case 6: {					//turn around
#if defined(DEBUGODOCALIB)
		printf("rozdil %f already_seen:%s was moving:%d",
				motor->getPosition()[2] - thirdodometry[2],
				(already_see) ? "true" : "false", this->wasmoving);
#endif
		if (!already_see) {
			//before see same landmark
			if ((motor->isMoving() || this->wasmoving > 0)
					&& stai_in_motion < 1) {	//wait two times
				motor->setSpeeds(0, 0);
				this->wasmoving -= 1;
				if (this->wasmoving < 1) {
					this->stai_in_motion = TURNING_COUNT;
				}
			} else {
				motor->setSpeeds(0, calibspeed);
				this->wasmoving = WAIT_STOPPED;
				this->stai_in_motion -= 1;
			}
		} else if (already_see) {
			//already see same landmark
			if ((motor->isMoving() || this->wasmoving < 10)
					&& stai_in_motion < 1) {	//wait two times
				motor->setSpeeds(0, 0);
				this->wasmoving = this->wasmoving + 1;
			} else {
				motor->setSpeeds(0, -calibspeed);
				this->wasmoving = 0;
			}
		} else {
			motor->setSpeeds(0, 0);
			printf("no moving\n");
		}

		}
			;
			break;
		case 7: {
			//missdetection
			motor->setMotorSpeedsS(0, 0);
		}
			;
			break;
		}
	}
}

void CMotorsCalib::evaluateCalibrationS() {
#if defined(DEBUGODOCALIB_RESULTS)
	//first observation after detection arbitrary blob
	printf("evaluating calibration\n");
	//second observation after motion left by right track movement
	printf("first odometry: %f %f %f\n", firstodometry[0], firstodometry[1],
			firstodometry[2]);
	printf("second odometry: %f %f %f\n", secondodometry[0], secondodometry[1],
			secondodometry[2]);
	printf("third odometry: %f %f %f\n", thirdodometry[0], thirdodometry[1],
			thirdodometry[2]);
	printf("fourth odometry: %f %f %f\n", fourthodometry[0], fourthodometry[1],
			fourthodometry[2]);
	printf("first mesured: %f %f %f\n", firstmeasured[0], firstmeasured[1],
			firstmeasured[2]);
	printf("second mesured: %f %f %f\n", secondmeasured[0], secondmeasured[1],
			secondmeasured[2]);
	printf("third measured: %f %f %f\n", thirdmeasured[0], thirdmeasured[1],
			thirdmeasured[2]);
	printf("fourth measured: %f %f %f\n", fourthmeasured[0], fourthmeasured[1],
			fourthmeasured[2]);
#endif
	double angle_error1 = secondmeasured[2] - firstmeasured[2];
	double angle_error2 = secondmeasured[2] - thirdmeasured[2];
#if defined(DEBUGODOCALIB_RESULTS)
	printf("angle error: %e \n", angle_error1);
	printf("angle error: %e \n", angle_error2);
#endif
	float null[3]={0,0,0};


	double alfa=atan2(firstmeasured[0],firstmeasured[1]);
	double beta=atan2(secondmeasured[0],secondmeasured[1]);
	float b=euclideanDistancef(firstmeasured,null);
	float c=euclideanDistancef(secondmeasured,null);
	double delta = abs(-beta+alfa+angle_error1);
	float a = sqrtf(b*b + c*c - 2*b*c*cos(delta));  //cosine theorem, a is distance between center of robot

	double r=0;		//circular path radius

	float traveled_LT1;
	float traveled_RT1;
	if(angle_error1>0){
		r = (a/2.0)/tan(abs(angle_error1)/2.0);
		traveled_LT1= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
		traveled_RT1= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
	}else if(angle_error1<0){
		r = (a/2.0)/tan(abs(angle_error1)/2.0);
		traveled_LT1= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
		traveled_RT1= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
	}else{
		traveled_LT1=traveled_RT1=a;
	}

#if defined(DEBUGODOCALIB_RESULTS)
	printf("alfa: %f \n", alfa);
	printf("beta: %f \n", beta);
	printf("b: %f \n", b);
	printf("c: %f \n", c);
	printf("delta: %f \n", delta);
	printf("a: %f \n", a);
	printf("r: %f \n", r);
	printf("traveled_LT: %f \n", traveled_LT1);
	printf("traveled_RT: %f \n", traveled_RT1);
#endif

//for traveling back:
	alfa=atan2(thirdmeasured[0],thirdmeasured[1]);
	beta=atan2(secondmeasured[0],secondmeasured[1]);
	b=euclideanDistancef(thirdmeasured,null);
	c=euclideanDistancef(secondmeasured,null);
	delta = abs(-beta+alfa+angle_error2);
	a = sqrtf(b*b + c*c - 2*b*c*cos(delta));

	float traveled_LT2;
	float traveled_RT2;
		if(angle_error1>0){
			r = (a/2.0)/tan(abs(angle_error2)/2.0);
			traveled_LT2= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
			traveled_RT2= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
		}else if(angle_error1<0){
			r = (a/2.0)/tan(abs(angle_error2)/2.0);
			traveled_LT2= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
			traveled_RT2= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
		}else{
			traveled_LT2=traveled_RT1=a;
		}

#if defined(DEBUGODOCALIB_RESULTS)
	printf("alfa2: %f \n", alfa);
	printf("beta2: %f \n", beta);
	printf("b2: %f \n", b);
	printf("c2: %f \n", c);
	printf("delta2: %f \n", delta);
	printf("a2: %f \n", a);
	printf("r2: %f \n", r);
	printf("traveled_LT2: %f \n", traveled_LT2);
	printf("traveled_RT2: %f \n", traveled_RT2);
#endif


	double odometryTraveled_dist1 = this->euclideanDistance(secondodometry,
			firstodometry);
	double odometry_angle_error1 =firstodometry[2]-secondodometry[2];
	double odometry_LT1;
	double odometry_RT1;
	if(odometry_angle_error1>0){
			r = (odometryTraveled_dist1/2.0)/tan(abs(odometry_angle_error1)/2.0);
			odometry_LT1= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error1;
			odometry_RT1= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error1;
		}else if(odometry_angle_error1<0){
			r = (odometryTraveled_dist1/2.0)/tan(abs(odometry_angle_error1)/2.0);
			odometry_LT1= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error1;
			odometry_RT1= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error1;
		}else{
			odometry_RT1=odometry_LT1=odometryTraveled_dist1;
		}

	double odometryTraveled_dist2 = this->euclideanDistance(secondodometry,
				thirdodometry);
	double odometry_angle_error2 =thirdodometry[2]-secondodometry[2];
	double odometry_LT2;
	double odometry_RT2;
	if(odometry_angle_error2>0){
				r = (odometryTraveled_dist2/2.0)/tan(abs(odometry_angle_error2)/2.0);
				odometry_LT2= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error2;
				odometry_RT2= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error2;
			}else if(odometry_angle_error2<0){
				r = (odometryTraveled_dist2/2.0)/tan(abs(odometry_angle_error2)/2.0);
				odometry_LT2= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error2;
				odometry_RT2= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error2;
			}else{
				odometry_RT2=odometry_LT1=odometryTraveled_dist2;
			}

#if defined(DEBUGODOCALIB_RESULTS)
	printf("odometryTraveled_dist1: %e \n", odometryTraveled_dist1);
	printf("odometry_angle_error1: %e \n", odometry_angle_error1);
	printf("odometry_LT1: %f \n", odometry_LT1);
	printf("odometry_RT1: %f \n", odometry_RT1);
	printf("odometryTraveled_dist2: %e \n", odometryTraveled_dist2);
	printf("odometry_angle_error2: %e \n", odometry_angle_error2);
	printf("odometry_LT2: %f \n", odometry_LT2);
	printf("odometry_RT2: %f \n", odometry_RT2);

#endif
#if defined(DEBUGODOCALIB_RESULTS)
	printf("old koeficients: %e , %e , %e\n", motor->odometry_koef1,
			motor->odometry_koef2, motor->odometry_koef3);
#endif

	double kalib_paramLT = (traveled_LT1 / odometry_LT1
			+ traveled_LT2 / odometry_LT2) / 2;
	double kalib_paramRT = (traveled_RT1 / odometry_RT1
			+ traveled_RT2 / odometry_RT2) / 2;

	double measured_angle_change = 2 * PI
			- (fourthmeasured[2] - thirdmeasured[2]);
	printf("ujetý úhel dokola: %f", measured_angle_change);

	double odo_angle_ch_ef_by_kalib_p = (fourthodometry[2] - thirdodometry[2])
			* (kalib_paramLT + kalib_paramRT) / 2; //d phi ktera by mela byt kdyz nepocitam se kalibraci rozchodu(odometry_koef3)
	double kalib_param_track = measured_angle_change
			/ odo_angle_ch_ef_by_kalib_p;

#if defined(DEBUGODOCALIB_RESULTS)
	printf("measured_angle_change: %f \n", measured_angle_change);
	printf("odo_angle_ch_ef_by_kalib_p: %f \n", odo_angle_ch_ef_by_kalib_p);
	printf("kalib_paramLT: %f \n", kalib_paramLT);
	printf("kalib_paramRT: %f \n", kalib_paramRT);
	printf("kalib_param_track: %f \n", kalib_param_track);
#endif
	//adjust mean values
	motor->odometry_koef1 = motor->odometry_koef1 * kalib_paramLT;	//left track
	motor->odometry_koef2 = motor->odometry_koef2 * kalib_paramRT;//right track
	motor->odometry_koef3 = motor->odometry_koef3 / kalib_param_track;//change track parameter

#if defined(DEBUGODOCALIB_RESULTS)
	printf("motor calibrated\n");
	printf("new koeficients: %e , %e , %e\n", motor->odometry_koef1,
			motor->odometry_koef2, motor->odometry_koef3);
#endif

	this->saveCalibResult(motor->odometry_koef1, motor->odometry_koef2,
			motor->odometry_koef3);
}

void CMotorsCalib::evaluateCalibrationKB() {

	//movement left
	double supTravAngle = this->euclideanDistance(firstodometry,
			secondodometry);
	double measuredTravAngle = -secondmeasured[1] + firstmeasured[1];
	this->calibParam1 = measuredTravAngle / supTravAngle;//kolikrát je reálná větší(tímto přenásobit konstantu)
	//movement right
	supTravAngle = this->euclideanDistance(thirdodometry, secondodometry);
	measuredTravAngle = -thirdmeasured[1] + secondmeasured[1];
	//konečná hodnota je průměrem pohybu doleva a doprava
	this->calibParam1 = (this->calibParam1 + measuredTravAngle / supTravAngle)
			/ 2;

	//movement back
	supTravAngle = this->euclideanDistance(fourthodometry, secondodometry);
	measuredTravAngle = fourthmeasured[0] - thirdmeasured[0];
	this->calibParam2 = measuredTravAngle / supTravAngle;
	//movement forward
	supTravAngle = this->euclideanDistance(fourthodometry, fifthodometry);
	measuredTravAngle = -fifthmeasured[0] + fourthmeasured[0];
	this->calibParam2 = (this->calibParam2 + measuredTravAngle / supTravAngle)
			/ 2;
	//param1 do stran
	//param2 dopredu
}

void CMotorsCalib::evaluateCalibrationAW() {
#if defined(DEBUGODOCALIB_RESULTS)
	//first observation after detection arbitrary blob
	printf("evaluating calibration\n");
	//second observation after motion left by right track movement
	printf("first odometry: %f %f %f\n", firstodometry[0], firstodometry[1],
			firstodometry[2]);
	printf("second odometry: %f %f %f\n", secondodometry[0], secondodometry[1],
			secondodometry[2]);
	printf("third odometry: %f %f %f\n", thirdodometry[0], thirdodometry[1],
			thirdodometry[2]);
	printf("fourth odometry: %f %f %f\n", fourthodometry[0], fourthodometry[1],
			fourthodometry[2]);
	printf("first mesured: %f %f %f\n", firstmeasured[0], firstmeasured[1],
			firstmeasured[2]);
	printf("second mesured: %f %f %f\n", secondmeasured[0], secondmeasured[1],
			secondmeasured[2]);
	printf("third measured: %f %f %f\n", thirdmeasured[0], thirdmeasured[1],
			thirdmeasured[2]);
	printf("fourth measured: %f %f %f\n", fourthmeasured[0], fourthmeasured[1],
			fourthmeasured[2]);

#endif
	//how robot changes angle during drive straight forward
	double angle_error1 = secondmeasured[2] - firstmeasured[2];
	//how robot changes angle during drive straight backward
	double angle_error2 = secondmeasured[2] - thirdmeasured[2];

#if defined(DEBUGODOCALIB_RESULTS)
	printf("angle error: %e \n", angle_error1);
	printf("angle error: %e \n", angle_error2);
#endif
	float null[3]={0,0,0};


	double alfa=atan2(firstmeasured[0],firstmeasured[1]);
	double beta=atan2(secondmeasured[0],secondmeasured[1]);
	float b=euclideanDistancef(firstmeasured,null);
	float c=euclideanDistancef(secondmeasured,null);
	double delta = abs(-beta+alfa+angle_error1);
	float a = sqrtf(b*b + c*c - 2*b*c*cos(delta));  //cosine theorem, a is distance between center of robot

	double r=0;		//circular path radius

	float traveled_LT1;
	float traveled_RT1;
	if(angle_error1>0){
		r = (a/2.0)/tan(abs(angle_error1)/2.0);
		traveled_LT1= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
		traveled_RT1= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
	}else if(angle_error1<0){
		r = (a/2.0)/tan(abs(angle_error1)/2.0);
		traveled_LT1= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
		traveled_RT1= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
	}else{
		traveled_LT1=traveled_RT1=a;
	}

#if defined(DEBUGODOCALIB_RESULTS)
	printf("alfa: %f \n", alfa);
	printf("beta: %f \n", beta);
	printf("b: %f \n", b);
	printf("c: %f \n", c);
	printf("delta: %f \n", delta);
	printf("a: %f \n", a);
	printf("r: %f \n", r);
	printf("traveled_LT: %f \n", traveled_LT1);
	printf("traveled_RT: %f \n", traveled_RT1);
#endif

//for traveling back:
	alfa=atan2(thirdmeasured[0],thirdmeasured[1]);
	beta=atan2(secondmeasured[0],secondmeasured[1]);
	b=euclideanDistancef(thirdmeasured,null);
	c=euclideanDistancef(secondmeasured,null);
	delta = abs(-beta+alfa+angle_error2);
	a = sqrtf(b*b + c*c - 2*b*c*cos(delta));

	float traveled_LT2;
	float traveled_RT2;
		if(angle_error1>0){
			r = (a/2.0)/tan(abs(angle_error2)/2.0);
			traveled_LT2= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
			traveled_RT2= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
		}else if(angle_error1<0){
			r = (a/2.0)/tan(abs(angle_error2)/2.0);
			traveled_LT2= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
			traveled_RT2= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*angle_error1;
		}else{
			traveled_LT2=traveled_RT1=a;
		}

#if defined(DEBUGODOCALIB_RESULTS)
	printf("alfa2: %f \n", alfa);
	printf("beta2: %f \n", beta);
	printf("b2: %f \n", b);
	printf("c2: %f \n", c);
	printf("delta2: %f \n", delta);
	printf("a2: %f \n", a);
	printf("r2: %f \n", r);
	printf("traveled_LT2: %f \n", traveled_LT2);
	printf("traveled_RT2: %f \n", traveled_RT2);
#endif


	double odometryTraveled_dist1 = this->euclideanDistance(secondodometry,
			firstodometry);
	double odometry_angle_error1 =firstodometry[2]-secondodometry[2];
	double odometry_LT1;
	double odometry_RT1;
	if(odometry_angle_error1>0){
			r = (odometryTraveled_dist1/2.0)/tan(abs(odometry_angle_error1)/2.0);
			odometry_LT1= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error1;
			odometry_RT1= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error1;
		}else if(odometry_angle_error1<0){
			r = (odometryTraveled_dist1/2.0)/tan(abs(odometry_angle_error1)/2.0);
			odometry_LT1= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error1;
			odometry_RT1= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error1;
		}else{
			odometry_RT1=odometry_LT1=odometryTraveled_dist1;
		}

	double odometryTraveled_dist2 = this->euclideanDistance(secondodometry,
				thirdodometry);
	double odometry_angle_error2 =thirdodometry[2]-secondodometry[2];
	double odometry_LT2;
	double odometry_RT2;
	if(odometry_angle_error2>0){
				r = (odometryTraveled_dist2/2.0)/tan(abs(odometry_angle_error2)/2.0);
				odometry_LT2= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error2;
				odometry_RT2= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error2;
			}else if(odometry_angle_error2<0){
				r = (odometryTraveled_dist2/2.0)/tan(abs(odometry_angle_error2)/2.0);
				odometry_LT2= (r-sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error2;
				odometry_RT2= (r+sin(0.5 * motor->getPosition()[5]) * 0.1375)*odometry_angle_error2;
			}else{
				odometry_RT2=odometry_LT1=odometryTraveled_dist2;
			}

#if defined(DEBUGODOCALIB_RESULTS)
	printf("odometryTraveled_dist1: %e \n", odometryTraveled_dist1);
	printf("odometry_angle_error1: %e \n", odometry_angle_error1);
	printf("odometry_LT1: %f \n", odometry_LT1);
	printf("odometry_RT1: %f \n", odometry_RT1);
	printf("odometryTraveled_dist2: %e \n", odometryTraveled_dist2);
	printf("odometry_angle_error2: %e \n", odometry_angle_error2);
	printf("odometry_LT2: %f \n", odometry_LT2);
	printf("odometry_RT2: %f \n", odometry_RT2);

#endif


#if defined(DEBUGODOCALIB_RESULTS)
	printf("old koeficients: %e , %e , %e\n", motor->odometry_koef1,
			motor->odometry_koef2, motor->odometry_koef3);
#endif

	double kalib_param_top = (traveled_LT1 / odometry_LT1
			+ traveled_LT2 / odometry_LT2) / 2;
	double kalib_param_down = (traveled_RT1 / odometry_RT1
			+ traveled_RT2 / odometry_RT2) / 2;

	double measured_angle_change = 2 * PI
			- (fourthmeasured[2] - thirdmeasured[2]);
	printf("ujetý úhel dokola: %f", measured_angle_change);

	double odo_angle_ch_ef_by_kalib_p = (fourthodometry[2] - thirdodometry[2])
			* (kalib_param_down + kalib_param_top) / 2;
	double kalib_param_track = measured_angle_change
			/ odo_angle_ch_ef_by_kalib_p;

#if defined(DEBUGODOCALIB_RESULTS)
	printf("measured_angle_change: %f \n", measured_angle_change);
	printf("odo_angle_ch_ef_by_kalib_p: %f \n", odo_angle_ch_ef_by_kalib_p);
	printf("kalib_param_down: %f \n", kalib_param_down);
	printf("kalib_param_top: %f \n", kalib_param_top);
	printf("kalib_param_track: %f \n", kalib_param_top);
#endif
	//adjust mean values
	motor->odometry_koef1 = motor->odometry_koef1 * kalib_param_down;//left down wheel , right down wheel
	motor->odometry_koef2 = motor->odometry_koef2 * kalib_param_top;//top wheel
	motor->odometry_koef3 = motor->odometry_koef3 / kalib_param_track;	//

#if defined(DEBUGODOCALIB_RESULTS)
	printf("motor calibrated\n");
	printf("new koeficients: %e , %e , %e\n", motor->odometry_koef1,
			motor->odometry_koef2, motor->odometry_koef3);
#endif

	this->saveCalibResult(motor->odometry_koef1, motor->odometry_koef2,
			motor->odometry_koef3);
}

void CMotorsCalib::convertCameraMeasurementS(float* measuredpos) {
	measuredpos[0] = measuredpos[0] + 0.049;
	measuredpos[1] = measuredpos[1] - 0.018;
}

void CMotorsCalib::convertCameraMeasurementAW(float* measuredpos, float hinge) {
	measuredpos[0] = measuredpos[0] + 0.04;
	measuredpos[1] = sin(hinge / 2) * 0.05
			+ (-cos((PI / 2) - (hinge / 2)) * measuredpos[1]
					+ sin((PI / 2) - (hinge / 2)) * measuredpos[3]);
	measuredpos[2] = -measuredpos[2];
}

void CMotorsCalib::convertCameraMeasurementKB(float* measuredpos) {
	//domer
	measuredpos[0] = measuredpos[0] + 0.04;
	measuredpos[1] = measuredpos[1] + 0.018;
}

void CMotorsCalib::filter(float* tofilter, float measuredpos[]) {
#if defined(DEBUGODOCALIB)
	printf("filter before %f , %f , %f , %f\n", tofilter[0], tofilter[1],
			tofilter[2], tofilter[3]);
#endif
	float z[4] = { tofilter[0] - measuredpos[0], tofilter[1] - measuredpos[1],
			tofilter[2] - measuredpos[2], tofilter[3] - measuredpos[3] };
	int s;
	gsl_matrix_set(this->Ppom, 0, 0,
			gsl_matrix_get(this->P, 0, 0) + gsl_matrix_get(this->Q, 0, 0));
	gsl_matrix_set(this->Ppom, 1, 1,
			gsl_matrix_get(this->P, 1, 1) + gsl_matrix_get(this->Q, 1, 1));
	gsl_matrix_set(this->Ppom, 2, 2,
			gsl_matrix_get(this->P, 2, 2) + gsl_matrix_get(this->Q, 2, 2));
	gsl_matrix_set(this->Ppom, 3, 3,
			gsl_matrix_get(this->P, 3, 3) + gsl_matrix_get(this->Q, 3, 3));
	gsl_linalg_LU_decomp(this->Ppom, permut, &s);
	gsl_linalg_LU_invert(this->Ppom, permut, invers);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, P, invers, 0.0, K);
	for (int var = 0; var < 4; ++var) {
		tofilter[var] = tofilter[var]
				- gsl_matrix_get(this->K, var, var) * z[var];
		gsl_matrix_set(this->P, var, var,
				(1 - gsl_matrix_get(this->K, var, var))
						* gsl_matrix_get(this->P, var, var));
	}
#if defined(DEBUGODOCALIB)
	printf("filtered %f , %f , %f , %f\n", tofilter[0], tofilter[1],
			tofilter[2], tofilter[3]);
#endif
}

void CMotorsCalib::initializeCovariance() {
	for (int var = 0; var < 4; ++var) {
		gsl_matrix_set(this->P, var, var, gsl_matrix_get(this->Q, var, var));
	}
}

bool CMotorsCalib::saveCalibResult(double calibresult1, double calibresult2,
		double calibresult3) {
	FILE * file;
	if (file = fopen("/flash/motorCALIB.dat", "wb")) {
		fprintf(file, "%e\n%e\n%e\n%d\n", calibresult1, calibresult2,
				calibresult3, this->calibspeed);
		fclose(file);
		return true;
	} else {
		printf("can not write calibresults to file \n");
		return false;
	}
}

bool CMotorsCalib::readCalibResult() {
	FILE * file;
	double calibresult1;
	double calibresult2;
	double calibresult3;
	int calibspeed;
	if (file = fopen("/flash/motorCALIB.dat", "rb")) {
		fscanf(file, "%le\n%le\n%le\n%d\n", &calibresult1, &calibresult2,
				&calibresult3, &calibspeed);
		fclose(file);
		printf("setting odometry coef 1 , 2 , 3: %e , %e , %e for speed %d\n",
				calibresult1, calibresult2, calibresult3, calibspeed);
		this->motor->odometry_koef1 = calibresult1;
		this->motor->odometry_koef2 = calibresult2;
		this->motor->odometry_koef3 = calibresult3;
		this->motor->calibratedSpeed = calibspeed;
		this->successful = true;
		return true;
	} else {
		printf("can not read calibresults from file \n");
		this->successful = false;
		return false;
	}
}

void CMotorsCalib::turn() {
	if (motor->isMoving() || this->wasmoving < 3) {	//wait two times
		motor->setSpeeds(0, 0);
		this->wasmoving = this->wasmoving + 1;

	} else {
		motor->setSpeeds(0, calibspeed);
		this->wasmoving = 0;
	}
}

bool CMotorsCalib::driveToCalibrationPosition() {
	// here some motion to be exactly before circle
	return true;
}

double CMotorsCalib::euclideanDistance(double* position1, double* position2) {
	return sqrt(
			pow(position1[0] - position2[0], 2)
					+ pow(position1[1] - position2[1], 2));
}
float CMotorsCalib::euclideanDistancef(float* position1, float* position2) {
	return sqrt(
			pow(position1[0] - position2[0], 2)
					+ pow(position1[1] - position2[1], 2));
}

float CMotorsCalib::euclideanDistancefd(float* position1, double* position2) {
	return sqrt(
			pow(position1[0] - position2[0], 2)
					+ pow(position1[1] - position2[1], 2));
}
/* namespace Mapping */
