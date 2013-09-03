/*
 * Mark.cpp
 *
 *  Created on: 15.11.2012
 *      Author: robert
 */

#include "Map.h"
#include <stdio.h>
#include <cmath>
#include <cstring>
#include "CTimer.h"
//13 32
using namespace std;
bool PRINT_MEASUREDPOS = true;
bool PRINT_MATRICES = false;
bool PRINT_LAND_MARKS = true;
bool PRINT_ROB_POS = true;
bool PRINT_MPTC = true;
double Map::PI = 3.141592653589793238462;
double ODOMETRY_XERROR = 0.00014;
double ODOMETRY_YERROR = 0.00003;
double ODOMETRY_PHIERROR = 0.0001;

double DOCK_MINIMAL_Z_POS = 0.18;
/*
 double MEASUREMENT_XERROR =  0.000126829009494;
 double MEASUREMENT_YERROR = 0.000007700015105;
 double MEASUREMENT_PHIERROR =  0.2311;
 */

double MEASUREMENT_XERROR = 0.000126829009494;
double MEASUREMENT_YERROR = 0.00003;
double MEASUREMENT_PHIERROR = 0.2311;
double MEASUREMENT_ZERROR = 0.2;

double kna2 = 2.3060e-05;
double ODOMETRY_DL_VARIANCE = 0;
double ODOMETRY_DR_VARIANCE = 0;
double ODOMETRY_B_VARIANCE = 0.01;
float TOLERANCE = 0.5;
double TOLERANCEPHI = 0.2;
double addP = 0.2;
double hL = 0.01;
double hR = 0.01;
double b = 0.12;
double *odometry_covariance = new double[3];
CTimer timermap;

Map::Map(double* odometr, RobotBase::RobotType robot_type) {
	//ODOMETRY_D_VARIANCE=ODOMETRY_D_VARIANCE*4;
	printf("Map initializing....\n");
	this->robot_type = robot_type;
	//odometry error
	kna2 = 100 * kna2;
	timermap.start();
	//MEASUREMENT_PHIERROR=2*MEASUREMENT_PHIERROR;
	this->odometry = new double[10];
	printf("map pos %f %f %f\n", odometr[0], odometr[1], odometr[2]);

	memcpy(this->odometry, odometr, 10 * sizeof(double));
	printf("map pos %f %f %f\n", odometry[0], odometry[1], odometry[2]);

	this->mapSize = 0;
	/*	this->R = gsl_matrix_calloc (3, 3);
	 gsl_matrix_set(this->R,0, 0, ODOMETRY_XERROR);
	 gsl_matrix_set(this->R,1,1, ODOMETRY_YERROR);
	 gsl_matrix_set(this->R,2,2, ODOMETRY_PHIERROR);*/
	//measurement error
	this->Q = gsl_matrix_calloc(4, 4);

	gsl_matrix_set(this->Q, 0, 0, MEASUREMENT_XERROR);
	gsl_matrix_set(this->Q, 1, 1, MEASUREMENT_YERROR);
	gsl_matrix_set(this->Q, 2, 2, MEASUREMENT_PHIERROR);
	gsl_matrix_set(this->Q, 3, 3, MEASUREMENT_ZERROR);

	//initiate robot state to odometry position
	this->state = gsl_matrix_calloc(3, 1);
	gsl_matrix_set(this->state, 0, 0, this->odometry[0]);
	gsl_matrix_set(this->state, 1, 0, this->odometry[1]);
	gsl_matrix_set(this->state, 2, 0, this->odometry[2]);

	mappedObjectTypes.push_back(ROBOT);

	this->predstate = gsl_matrix_calloc(3, 1);
	//initiate covariance P to null 3x3 matrix
	this->P = gsl_matrix_calloc(3, 3);
	//zmeneno
	gsl_matrix_set(this->P, 0, 0, 0);
	gsl_matrix_set(this->P, 1, 1, 0);
	gsl_matrix_set(this->P, 2, 2, 0);
	this->predP = gsl_matrix_calloc(3, 3);
	//zmeneno
	gsl_matrix_set(this->predP, 0, 0, 0);
	gsl_matrix_set(this->predP, 1, 1, 0);
	gsl_matrix_set(this->predP, 2, 2, 0);
	this->time = time;
	this->newDetected = false;
	printf("Map initialized\n");
	lastSeen = -1;
	seeAfterLongTime = false;
}

Map::~Map() {
	gsl_matrix_free(this->state);
	gsl_matrix_free(this->P);
	gsl_matrix_free(this->Q);

}

void Map::filter(double robpos[], float measuredpos[]) {

	int timechanged = this->time - time;
	//this->time=time;

	convertCameraMeasurement(measuredpos, robpos[5], robot_type);

	if (PRINT_MEASUREDPOS) {
		printf("measuredpos=[measuredpos , [%f ; %f ; %f ; %f]]\n",
				measuredpos[0], measuredpos[1], measuredpos[2], measuredpos[3]);
	}
	/*if(measuredpos[2]>0.2 ||measuredpos[2]<-0.2	){
	 gsl_matrix_set(this->Q,2, 2, MEASUREMENT_PHIERROR);
	 }else{
	 gsl_matrix_set(this->Q,2, 2, MEASUREMENT_PHIERROR2);
	 }*/

	//printf("uhel:%f\n",measuredpos[2]*180/this->PI);
	double changedx = robpos[0] - this->odometry[0];
	double changedy = robpos[1] - this->odometry[1];
	double changedphi = robpos[2] - this->odometry[2];
	double changedDL = robpos[3] - this->odometry[3];
	double changedDR = robpos[4] - this->odometry[4];

//	printf("%1.10f , %1.10f , %1.10f\n",changedx,changedy,changedphi);
	memcpy(this->odometry, robpos, 10 * sizeof(double));

	gsl_matrix_memcpy(this->predstate, this->state);

	int pocetprvku = this->mapSize * 4 + 3;
	gsl_matrix_set(this->predstate, 0, 0,
			gsl_matrix_get(this->state, 0, 0) + (changedx));
	gsl_matrix_set(this->predstate, 1, 0,
			gsl_matrix_get(this->state, 1, 0) + (changedy));
	gsl_matrix_set(this->predstate, 2, 0,
			gsl_matrix_get(this->state, 2, 0) + (changedphi));
	gsl_matrix_set(this->predstate, 2, 0,
			normalizeAngle(gsl_matrix_get(this->predstate, 2, 0)));

	//6    predP=G*P*G' + F'*R*F'

	gsl_matrix_memcpy(this->predP, this->P);

	//robpos[3] je dL
	//robpos[4] je dR

	calculateOdometryCovariance(changedx, changedy, changedphi, changedDL,
			changedDR);
	/*
	 double sigma2x = (0.5*pow(cos(gsl_matrix_get(this->predstate, 2, 0)),2)+2*pow(((changedDL+changedDR)/(2*b))*sin(gsl_matrix_get(this->predstate, 2, 0)),2))*ODOMETRY_D_VARIANCE;
	 double sigma2y = (0.5*pow(sin(gsl_matrix_get(this->predstate, 2, 0)),2)+2*pow(((changedDL+changedDR)/(2*b))*cos(gsl_matrix_get(this->predstate, 2, 0)),2))*ODOMETRY_DR_VARIANCE;
	 double sigma2phi = (2/(b*b))*ODOMETRY_DR_VARIANCE;
	 */

	gsl_matrix_set(this->predP, 0, 0,
			gsl_matrix_get(this->P, 0, 0) + odometry_covariance[0]);
	gsl_matrix_set(this->predP, 1, 1,
			gsl_matrix_get(this->P, 1, 1) + odometry_covariance[1]);
	gsl_matrix_set(this->predP, 2, 2,
			gsl_matrix_get(this->P, 2, 2) + odometry_covariance[2]);
	//	printMatrix(this->predP);

	/*
	 double sigmaL2=1.7936e-06;
	 double sigmaR2=1.7936e-06;
	 gsl_matrix_set(this->predP,0, 0, gsl_matrix_get(this->P, 0, 0)+0.25*cos(robpos[2])*cos(robpos[2])*(sigmaL2+sigmaR2));
	 gsl_matrix_set(this->predP,0, 0, gsl_matrix_get(this->P, 0, 1)+0.25*cos(robpos[2])*sin(robpos[2])*(sigmaL2+sigmaR2));
	 gsl_matrix_set(this->predP,0, 0, gsl_matrix_get(this->P, 0, 2)+(0.5/b)*cos(robpos[2])*(sigmaL2-sigmaR2));
	 gsl_matrix_set(this->predP,0, 0, gsl_matrix_get(this->P, 1, 0)+0.25*cos(robpos[2])*sin(robpos[2])*(sigmaL2+sigmaR2));
	 gsl_matrix_set(this->predP,0, 0, gsl_matrix_get(this->P, 1, 1)+0.25*sin(robpos[2])*sin(robpos[2])*(sigmaL2+sigmaR2));
	 gsl_matrix_set(this->predP,0, 0, gsl_matrix_get(this->P, 1, 2)+(0.5/b)*sin(robpos[2])*((sigmaL2+sigmaR2)/2));
	 gsl_matrix_set(this->predP,0, 0, gsl_matrix_get(this->P, 2, 0)+(0.5/b)*cos(robpos[2])*((sigmaL2+sigmaR2)/2));
	 gsl_matrix_set(this->predP,0, 0, gsl_matrix_get(this->P, 2, 1)+(0.5/b)*sin(robpos[2])*((sigmaL2+sigmaR2)/2));
	 gsl_matrix_set(this->predP,0, 0, gsl_matrix_get(this->P, 2, 2)+((sigmaL2-sigmaR2)/2)/(b*b));
	 */

	if (PRINT_MATRICES) {
		printf("P:\n");
		printMatrix(this->P);
		printf("predP:\n");
		printMatrix(this->predP);
		printf("predState:\n");
		printMatrix(this->predstate);

	}

	robpos[0] = gsl_matrix_get(this->predstate, 0, 0);
	robpos[1] = gsl_matrix_get(this->predstate, 1, 0);
	robpos[2] = gsl_matrix_get(this->predstate, 2, 0);
	//9 computing position of measurement in global space
	double measuredToCenterX = cos(robpos[2]) * measuredpos[0]
			- sin(robpos[2]) * measuredpos[1] + robpos[0];
	double measuredToCenterY = sin(robpos[2]) * measuredpos[0]
			+ cos(robpos[2]) * measuredpos[1] + robpos[1];
	double measuredToCenterPHI = normalizeAngle(robpos[2] + measuredpos[2]);
	double measuredToCenterZ = measuredpos[3];

	if (PRINT_MPTC) {
		printf("MPTC= [MPTC [%2.6f ; %2.6f ; %2.6f ; %2.6f ]]; \n",
				measuredToCenterX, measuredToCenterY, measuredToCenterPHI,
				measuredToCenterZ);
	}
	//printf("after measured to center\n");
	bool mapatoNEobsahuje = true;
	int pozicevmape = -1;
	int sign = -1;
	float vzdalenost;
	float vzdalenostold = 10;
	int var;
	for (var = 0; var < this->mapSize; ++var) {
		float vzdalenost = pow(
				pow(
						measuredToCenterX
								- gsl_matrix_get(this->predstate, var * 4 + 3,
										0), 2)
						+ pow(
								measuredToCenterY
										- gsl_matrix_get(this->predstate,
												var * 4 + 4, 0), 2), 0.5);
		if ((this->mappedObjectTypes[var+1]==NORMAL_CIRCLE || this->mappedObjectTypes[var+1]==DOCK_CIRCLE) && vzdalenost < TOLERANCE ) {
			//if(gsl_matrix_get(this->predstate, var*3+3, 0)-TOLERANCE<measuredToCenterX && gsl_matrix_get(this->predstate, var*3+3, 0)-TOLERANCE<measuredToCenterX){
			//	if(gsl_matrix_get(this->predstate, var*3+4, 0)-TOLERANCE<measuredToCenterY && gsl_matrix_get(this->predstate, var*3+4, 0)-TOLERANCE<measuredToCenterY){
			//je to ten objekt
			if (pozicevmape == -1 || vzdalenost < vzdalenostold) {
				if (this->lastSeen != var) {
					//see again after some time = need more filtering
					this->seeAfterLongTime = true;
				} else {
					this->seeAfterLongTime = false;
				}
				this->lastSeen = var;
				pozicevmape = var;
				//	printf("klasifikován LM%d \n",pozicevmape+1);
				mapatoNEobsahuje = false;
				this->newDetected = false;
				sign = (measuredToCenterPHI > 0) ? -1 : -1;
				vzdalenostold = vzdalenost;
			}
		}
	}
	//odometry phi just for shorter write
	double predictedPhi = gsl_matrix_get(this->predstate, 2, 0);

	if (mapatoNEobsahuje) {
		//	printf("new Landmark\n");
		this->newDetected = true;
		if (measuredToCenterZ > DOCK_MINIMAL_Z_POS) {
			mappedObjectTypes.push_back(NORMAL_CIRCLE);
		} else {
			mappedObjectTypes.push_back(DOCK_CIRCLE);
		}

		pozicevmape = this->mapSize; //pozice v mape je od 0-inf
		this->mapSize += 1;
		if (PRINT_MATRICES) {
			printf("velikost mapy %d \n", this->mapSize);

		}
		int oldsize = 4 * (this->mapSize - 1) + 3;
		int newsize = 4 * (this->mapSize) + 3;
		pocetprvku = newsize;
		gsl_matrix *newstate = gsl_matrix_alloc(newsize, 1);
		gsl_matrix *newpredstate = gsl_matrix_alloc(newsize, 1);
		gsl_matrix *newP = gsl_matrix_calloc(newsize, newsize);
		gsl_matrix *newpredP = gsl_matrix_calloc(newsize, newsize);
		int var, var2;
		for (var = 0; var < oldsize; ++var) {
			gsl_matrix_set(newstate, var, 0,
					gsl_matrix_get(this->state, var, 0));
			gsl_matrix_set(newpredstate, var, 0,
					gsl_matrix_get(this->predstate, var, 0));
			for (var2 = 0; var2 < oldsize; ++var2) {
				gsl_matrix_set(newP, var, var2,
						gsl_matrix_get(this->P, var, var2));
				gsl_matrix_set(newpredP, var, var2,
						gsl_matrix_get(this->predP, var, var2));
			}
		}

		//setting new state and predicted state values
		int i = 0;
		gsl_matrix_set(newstate, pozicevmape * 4 + 3, 0, measuredToCenterX);
		gsl_matrix_set(newpredstate, pozicevmape * 4 + 3, 0, measuredToCenterX);
		gsl_matrix_set(newstate, pozicevmape * 4 + 4, 0, measuredToCenterY);
		gsl_matrix_set(newpredstate, pozicevmape * 4 + 4, 0, measuredToCenterY);

		gsl_matrix_set(newstate, pozicevmape * 4 + 5, 0, measuredToCenterPHI);
		gsl_matrix_set(newpredstate, pozicevmape * 4 + 5, 0,
				measuredToCenterPHI);
		gsl_matrix_set(newstate, pozicevmape * 4 + 6, 0, measuredToCenterZ);
		gsl_matrix_set(newpredstate, pozicevmape * 4 + 6, 0, measuredToCenterZ);

		if (PRINT_MATRICES) {
			printf("new state:\n");
			printMatrix(newstate);
			printf("new predictedstate:\n");
			printMatrix(newpredstate);
		}
		i = 0;

		gsl_matrix *Pll = gsl_matrix_calloc(4, 4);
		gsl_matrix *Plx = gsl_matrix_calloc(4, oldsize);
		//jacoby matrix of measuredToCenter=g(robpos,measuredpos) by robpos
		gsl_matrix *Gr = gsl_matrix_calloc(4, 3);
		////jacoby matrix of measuredToCenter=g(robpos,measuredpos) by measuredpos
		gsl_matrix *Gw = gsl_matrix_calloc(4, 4);

		//setting parts of jacobian Gr which is derivation of relative position by robot
		gsl_matrix_set(Gr, 0, 0, 1);
		gsl_matrix_set(Gr, 1, 1, 1);
		gsl_matrix_set(Gr, 0, 2,
				-sin(predictedPhi) * measuredpos[0]
						- cos(predictedPhi) * measuredpos[1]);
		gsl_matrix_set(Gr, 1, 2,
				cos(predictedPhi) * measuredpos[0]
						- sin(predictedPhi) * measuredpos[1]);
		gsl_matrix_set(Gr, 2, 2, 1);
		//setting parts of jacobian Gw which is derivation of relative position by mapped position
		gsl_matrix_set(Gw, 0, 0, cos(predictedPhi));
		gsl_matrix_set(Gw, 0, 1, -sin(predictedPhi));
		gsl_matrix_set(Gw, 1, 0, sin(predictedPhi));
		gsl_matrix_set(Gw, 1, 1, cos(predictedPhi));
		gsl_matrix_set(Gw, 2, 2, 1);
		gsl_matrix_set(Gw, 3, 3, 1);

		if (PRINT_MATRICES) {
			printf("Q:\n");
			printMatrix(this->Q);
			printf("Gr:\n");
			printMatrix(Gr);
			printf("Gw:\n");
			printMatrix(Gw);

			printf("predictedP:\n");
			printMatrix(predP);
		}

		//Pll=Gr*predictedP(0:2,0:2)*Gr'+Gw*R*Gw'
		gsl_matrix *Gw_Q = gsl_matrix_calloc(4, 4);
		//Gw*Q

		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Gw, this->Q, 0.0, Gw_Q);
		gsl_matrix *Gw_Q_Gwt = gsl_matrix_calloc(4, 4);
		//Gw*Q*Gw'
		gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, Gw_Q, Gw, 0.0, Gw_Q_Gwt);
		gsl_matrix *Gr_predP = gsl_matrix_calloc(4, 3);
		//submatrix of P
		gsl_matrix_view P0_20_2 = gsl_matrix_submatrix(this->predP, 0, 0, 3, 3);
		//Gr*predictedP(0:2,0:2)
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Gr, &P0_20_2.matrix,
				0.0, Gr_predP);
		gsl_matrix *Gr_predP_Grt = gsl_matrix_calloc(4, 4);
		//Gw*R*Gw'

		gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, Gr_predP, Gr, 0.0,
				Gr_predP_Grt);
		//Pll=Gr*predictedP(0:2,0:2)*Gr'+Gw*Q*Gw'
		gsl_matrix_add(Pll, Gr_predP_Grt);
		gsl_matrix_add(Pll, Gw_Q_Gwt);

		gsl_matrix_view P0_20_end = gsl_matrix_submatrix(this->predP, 0, 0, 3,
				oldsize);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Gr, &P0_20_end.matrix,
				0.0, Plx);
		if (PRINT_MATRICES) {
			printf("Plx:\n");
			printMatrix(Plx);
			printf("Pll:\n");
			printMatrix(Pll);
		}

		//setting new covariance and predicted covariance values
		for (var = 0; var < 4; ++var) {

			for (var2 = 0; var2 < oldsize; ++var2) {
				gsl_matrix_set(newpredP, oldsize + var, var2,
						gsl_matrix_get(Plx, var, var2));
				gsl_matrix_set(newpredP, var2, var + oldsize,
						gsl_matrix_get(Plx, var, var2));
			}

			for (var2 = 0; var2 < 4; ++var2) {
				gsl_matrix_set(newpredP, var + oldsize, var2 + oldsize,
						gsl_matrix_get(Pll, var, var2));
			}

		}
		if (PRINT_MATRICES) {
			printf("newpredP:\n");
			printMatrix(newpredP);
			printf("newP:\n");
			printMatrix(newP);
		}

		//dealocationg old matrices
		gsl_matrix_free(this->state);
		gsl_matrix_free(this->predstate);
		gsl_matrix_free(this->P);
		gsl_matrix_free(this->predP);
		gsl_matrix_free(Gr);
		gsl_matrix_free(Gw);
		gsl_matrix_free(Gw_Q);
		gsl_matrix_free(Gw_Q_Gwt);
		gsl_matrix_free(Gr_predP);
		gsl_matrix_free(Gr_predP_Grt);
		gsl_matrix_free(Pll);
		gsl_matrix_free(Plx);
		//setting new matrices
		this->state = newstate;
		this->predstate = newpredstate;
		this->P = newP;
		this->predP = newpredP;

		gsl_matrix_memcpy(this->state, this->predstate);
		gsl_matrix_memcpy(this->P, this->predP);

		return;
	}

	//creating kalmans gain matrix
	gsl_matrix *K = gsl_matrix_calloc(pocetprvku, 4);
	//creating jacobi matrix enlarged to size of state
	gsl_matrix *H = gsl_matrix_calloc(4, pocetprvku);

	//creating jacobi matrix enlarged to size of state
	double rozdilx = gsl_matrix_get(this->predstate, pozicevmape * 4 + 3, 0)
			- gsl_matrix_get(this->predstate, 0, 0);
	double rozdily = gsl_matrix_get(this->predstate, pozicevmape * 4 + 4, 0)
			- gsl_matrix_get(this->predstate, 1, 0);
	if (PRINT_MATRICES) {
		printf("rozdilx:\n %2.6f \n", rozdilx);
		printf("rozdily:\n %2.6f \n", rozdily);
		printf("predictedPhi:\n %2.6f \n", predictedPhi);
	}

	gsl_matrix_set(H, 0, 0, -cos(-predictedPhi));
	gsl_matrix_set(H, 1, 0, -sin(-predictedPhi));
	gsl_matrix_set(H, 0, 1, sin(-predictedPhi));
	gsl_matrix_set(H, 1, 1, -cos(-predictedPhi));
	gsl_matrix_set(H, 0, 2,
			sin(-predictedPhi) * rozdilx + cos(-predictedPhi) * rozdily);
	gsl_matrix_set(H, 1, 2,
			-cos(-predictedPhi) * rozdilx + sin(-predictedPhi) * rozdily);
	gsl_matrix_set(H, 2, 2, -1);
	gsl_matrix_set(H, 0, 4 * pozicevmape + 3, cos(-predictedPhi));
	gsl_matrix_set(H, 1, 4 * pozicevmape + 3, sin(-predictedPhi));
	gsl_matrix_set(H, 0, 4 * pozicevmape + 4, -sin(-predictedPhi));
	gsl_matrix_set(H, 1, 4 * pozicevmape + 4, cos(-predictedPhi));
	gsl_matrix_set(H, 2, 4 * pozicevmape + 5, 1);
	gsl_matrix_set(H, 2, 4 * pozicevmape + 5, 1);
	gsl_matrix_set(H, 3, 4 * pozicevmape + 6, 1);
	if (PRINT_MATRICES) {
		printf("H:\n");
		printMatrix(H);
	}
	//K=(predictedP*(H'))/(H*predictedP*(H')+Q);
	gsl_matrix *predP_Ht = gsl_matrix_calloc(pocetprvku, 4);
	//predictedP*(H')
	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, this->predP, H, 0.0,
			predP_Ht);
	gsl_matrix *H_predP_Ht_Q = gsl_matrix_calloc(4, 4);
	//H*predictedP*(H')
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, H, predP_Ht, 0.0,
			H_predP_Ht_Q);
	//H*predictedP*(H')+Q
	gsl_matrix_add(H_predP_Ht_Q, this->Q);
	//(H*predictedP*(H')+Q)^-1
	if (PRINT_MATRICES) {
		printf("H_predP_Ht_Q:\n");
		printMatrix(H_predP_Ht_Q);
	}
	gsl_matrix *invers = gsl_matrix_calloc(4, 4);
	gsl_permutation * permut = gsl_permutation_alloc(4);
	int s;
	gsl_linalg_LU_decomp(H_predP_Ht_Q, permut, &s);
	gsl_linalg_LU_invert(H_predP_Ht_Q, permut, invers);
	if (PRINT_MATRICES) {
		printf("invers:\n");
		printMatrix(invers);
	}

	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, predP_Ht, invers, 0.0, K);
	if (PRINT_MATRICES) {
		printf("K:\n");
		printMatrix(K);
	}
	gsl_matrix *difference = gsl_matrix_alloc(4, 1);
	gsl_matrix_set(difference, 0, 0,
			measuredpos[0]
					- (cos(-predictedPhi) * (rozdilx)
							- sin(-predictedPhi) * (rozdily)));
	gsl_matrix_set(difference, 1, 0,
			measuredpos[1]
					- (sin(-predictedPhi) * (rozdilx)
							+ cos(-predictedPhi) * (rozdily)));
	gsl_matrix_set(difference, 2, 0,
			measuredpos[2]
					- (normalizeAngle(
							gsl_matrix_get(this->predstate, 4 * pozicevmape + 5,
									0) - predictedPhi)));
	gsl_matrix_set(difference, 3, 0,
			measuredpos[3]
					- gsl_matrix_get(this->predstate, 4 * pozicevmape + 6, 0));

	if (PRINT_MATRICES) {

		printf("difference:\n");
		printMatrix(difference);
	}

	gsl_matrix *newP = gsl_matrix_calloc(mapSize * 4 + 3, mapSize * 4 + 3);
	gsl_matrix_memcpy(this->state, this->predstate);
	for (var = 0; var < mapSize * 4 + 3; ++var) {
		//P=I
		gsl_matrix_set(newP, var, var, 1);
	}

	gsl_matrix *K_difference = gsl_matrix_calloc(mapSize * 4 + 3, 1);
	//K*difference
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, difference, 0.0,
			K_difference);
	//state=predictedstate+K*difference
	if (PRINT_MATRICES) {
		printf("predictedstate:\n");
		printMatrix(this->predstate);
		printf("K_difference:\n");
		printMatrix(K_difference);
	}
	gsl_matrix_add(this->state, K_difference);

	gsl_matrix *K_H = gsl_matrix_calloc(mapSize * 4 + 3, mapSize * 4 + 3);
	if (PRINT_MATRICES) {
		printf("newP");
		printMatrix(newP);
		printf("K_H:\n");
		printMatrix(K_H);
	}
	//K*H
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, K, H, 0.0, K_H);
	//P=I-K*H
	gsl_matrix_sub(newP, K_H);
	if (PRINT_MATRICES) {
		printf("newP");
		printMatrix(newP);

	}
	//P=(I-K*H)*predP
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, newP, this->predP, 0.0,
			this->P);

	if (PRINT_MATRICES) {
		printf("P:\n");
		printMatrix(this->P);
		printf("predP:\n");
		printMatrix(this->predP);
	}

	gsl_matrix_free(newP);
	gsl_matrix_free(K_difference);
	gsl_matrix_free(K);
	gsl_matrix_free(K_H);
	gsl_matrix_free(predP_Ht);
	gsl_matrix_free(difference);
	if (PRINT_ROB_POS) {
		printf("robot pos\n");
		printf("ROBPOS=[ROBPOS [%2.7f ; %2.7f ; %2.7f ; 1 ]]; \n",
				gsl_matrix_get(this->state, 0, 0),
				gsl_matrix_get(this->state, 1, 0),
				gsl_matrix_get(this->state, 2, 0));
		printf("ROBUNCERT=[ROBUNCERT [%2.7f ; %2.7f ; %2.7f]];\n",
				gsl_matrix_get(this->P, 0, 0), gsl_matrix_get(this->P, 1, 1),
				gsl_matrix_get(this->P, 2, 2));
	}
	if (PRINT_LAND_MARKS) {
		//printf("velikost mapy je:%d",this->mapSize);
		printf("statesize %d %d \n", this->state->size1, this->state->size2);
		printf("Psize %d %d \n", this->P->size1, this->P->size2);
		for (var = 0; var <= this->mapSize - 1; ++var) {
			printf("LM%d=[LM%d [%2.7f ; %2.7f ; %2.7f ; %2.7f]]; \n", var, var,
					gsl_matrix_get(this->state, var * 4 + 3, 0),
					gsl_matrix_get(this->state, var * 4 + 4, 0),
					gsl_matrix_get(this->state, var * 4 + 5, 0),
					gsl_matrix_get(this->state, var * 4 + 6, 0));
			printf(
					"LM%dUNCERT=[LM%dUNCERT [%2.7f ; %2.7f ; %2.7f ; %2.7f]]; \n",
					var, var, gsl_matrix_get(this->P, var * 4 + 3, var * 4 + 3),
					gsl_matrix_get(this->P, var * 4 + 4, var * 4 + 4),
					gsl_matrix_get(this->P, var * 4 + 5, var * 4 + 5),
					gsl_matrix_get(this->P, var * 4 + 6, var * 4 + 6));
		}

	}

}

void Map::odometryChange(double robpos[]) {
	//printf("odometry change\n");
	int var;
	double changedx = robpos[0] - this->odometry[0];
	double changedy = robpos[1] - this->odometry[1];
	double changedphi = robpos[2] - this->odometry[2];
	double changedDL = robpos[3] - this->odometry[3];
	double changedDR = robpos[4] - this->odometry[4];
	//printf("DL=%f\n",robpos[3]);
	//printf("DR=%f\n",robpos[4]);

	memcpy(this->odometry, robpos, 10 * sizeof(double));
	gsl_matrix_set(this->state, 0, 0,
			gsl_matrix_get(this->state, 0, 0) + (changedx));
	gsl_matrix_set(this->state, 1, 0,
			gsl_matrix_get(this->state, 1, 0) + (changedy));
	gsl_matrix_set(this->state, 2, 0,
			gsl_matrix_get(this->state, 2, 0) + (changedphi));
	gsl_matrix_set(this->state, 2, 0,
			normalizeAngle(gsl_matrix_get(this->state, 2, 0)));

	calculateOdometryCovariance(changedx, changedy, changedphi, changedDL,
			changedDR);
	/*
	 ODOMETRY_D_VARIANCE=kna2*changedDL;
	 double sigma2x = (0.5*pow(cos(gsl_matrix_get(this->predstate, 2, 0)),2)+2*pow(((changedDL+changedDR)/(2*b))*sin(gsl_matrix_get(this->predstate, 2, 0)),2))*ODOMETRY_D_VARIANCE;
	 double sigma2y = (0.5*pow(sin(gsl_matrix_get(this->predstate, 2, 0)),2)+2*pow(((changedDL+changedDR)/(2*b))*cos(gsl_matrix_get(this->predstate, 2, 0)),2))*ODOMETRY_D_VARIANCE;
	 double sigma2phi = (2/(b*b))*ODOMETRY_D_VARIANCE;
	 */

	gsl_matrix_set(this->P, 0, 0,
			gsl_matrix_get(this->P, 0, 0) + odometry_covariance[0]);
	gsl_matrix_set(this->P, 1, 1,
			gsl_matrix_get(this->P, 1, 1) + odometry_covariance[1]);
	gsl_matrix_set(this->P, 2, 2,
			gsl_matrix_get(this->P, 2, 2) + odometry_covariance[2]);
	if (PRINT_ROB_POS) {
		printf("ROBPOS=[ROBPOS [%2.7f ; %2.7f ; %2.7f ; 0 ]]; \n",
				gsl_matrix_get(this->state, 0, 0),
				gsl_matrix_get(this->state, 1, 0),
				gsl_matrix_get(this->state, 2, 0));
		printf("ROBUNCERT=[ROBUNCERT [%2.7f ; %2.7f ; %2.7f]];\n",
				gsl_matrix_get(this->P, 0, 0), gsl_matrix_get(this->P, 1, 1),
				gsl_matrix_get(this->P, 2, 2));

	}
	if (PRINT_LAND_MARKS) {
		//printf("velikost mapy je:%d",this->mapSize);
		for (var = 0; var <= this->mapSize - 1; ++var) {
			printf("LM%d=[LM%d [%2.7f ; %2.7f ; %2.7f ; %2.7f ]]; \n", var, var,
					gsl_matrix_get(this->state, var * 4 + 3, 0),
					gsl_matrix_get(this->state, var * 4 + 4, 0),
					gsl_matrix_get(this->state, var * 4 + 5, 0),
					gsl_matrix_get(this->state, var * 4 + 6, 0));
			printf(
					"LM%dUNCERT=[LM%dUNCERT [%2.7f ; %2.7f ; %2.7f ; %2.7f ]]; \n",
					var, var, gsl_matrix_get(this->P, var * 4 + 3, var * 4 + 3),
					gsl_matrix_get(this->P, var * 4 + 4, var * 4 + 4),
					gsl_matrix_get(this->P, var * 4 + 5, var * 4 + 5),
					gsl_matrix_get(this->P, var * 4 + 6, var * 4 + 6));
		}

	}
}

MapData Map::readFromFile(const char* filename) {
	FILE * file;
	MapData data;
	if (file = fopen(filename, "rb")) {

		int mmap = 0;
		int nmap = 0;
		int mcov = 0;
		int ncov = 0;
		fscanf(file, "%u\n", &mmap);
		fscanf(file, "%u\n\n", &nmap);
		gsl_matrix * map = gsl_matrix_alloc(mmap, nmap);
		gsl_matrix_fscanf(file, map);
		fscanf(file, "\n%u\n", &mcov);
		fscanf(file, "%u\n", &ncov);
		gsl_matrix * covariance = gsl_matrix_alloc(mcov, ncov);
		gsl_matrix_fscanf(file, covariance);
		fclose(file);
		data.map = map;
		data.covariance = covariance;
		return data;
	} else {
		printf("can not read matrix from file %s \n", filename);
		data.map = NULL;
		data.covariance = NULL;
		return data;
	}

}

int Map::writeToFile(const char* filename, MapData data) {
	FILE * file;
	if (file = fopen(filename, "wb")) {
		fprintf(file, "%lu\n", data.map->size1);
		fprintf(file, "%lu\n\n", data.map->size2);
		gsl_matrix_fprintf(file, data.map, "%g");
		fprintf(file, "\n");
		fprintf(file, "%lu\n", data.covariance->size1);
		fprintf(file, "%lu\n\n", data.covariance->size2);
		gsl_matrix_fprintf(file, data.covariance, "%g");
		fprintf(file, "\n");
		fclose(file);
		return 0;
	} else {
		printf("can not write matrix to file %s \n", filename);
		return 1;
	}

}
void Map::printMatrix(gsl_matrix *matrix) {
	int var, var2;
	for (var = 0; var < matrix->size1; ++var) {
		for (var2 = 0; var2 < matrix->size2; ++var2) {
			printf("%1.4e ", gsl_matrix_get(matrix, var, var2));
		}
		printf("\n");
	}
}
void Map::printArray(double *array, int lenght) {
	int var;
	for (var = 0; var < lenght; ++var) {
		printf("%2.6f ", array[var]);
	}
	printf("\n");
}

double Map::normalizeAngle(double angle) {
	angle = fmod(angle, 2 * PI);
	if (angle > PI) {
		angle = angle - 2 * PI;
	} else {
		if (angle < -PI) {
			angle = angle + 2 * PI;
		}
	}
	return angle;
}

double Map::normalizeAngleDiff(double diff) {
	diff = abs(diff);
	if (diff > PI) {
		diff = abs(diff - 2 * PI);
	}
	return diff;
}

int Map::minIndex(double* pole, int delka) {
	double min = pole[0];
	int index = 0;
	int var;
	for (var = 1; var < delka; ++var) {
		if (pole[var] <= min) {
			min = pole[var];
			index = var;
		}
	}
	//printf("I %d\n",index+1);
	return index;
}

void Map::convertCameraMeasurementS(float measuredpos[]) {
	//robot má kameru 3.15 od levého kraje - tedy na pozici y=-1.85
	//robot má kameru 4.9 od středu otáčení				x=4.9
	measuredpos[0] = measuredpos[0] + 0.049;
	measuredpos[1] = measuredpos[1] - 0.018;
	//measuredpos[2] se nezmění - promítnutí do plochy
	measuredpos[3] = measuredpos[3] + 0.09;	//z axis
}

void Map::convertCameraMeasurementKB(float measuredpos[]) {
	// upravit
	measuredpos[0] = measuredpos[0] + 0.049;
	measuredpos[1] = measuredpos[1] - 0.018;
	//measuredpos[2] se nezmění - promítnutí do plochy
	measuredpos[3] = measuredpos[3] + 0.09;	//z axis
}

void Map::convertCameraMeasurementAW(float measuredpos[], float hinge) {
	//robot má kameru 3.15 od levého kraje - tedy na pozici y=-1.85
	//robot má kameru 4.9 od středu otáčení				x=4.9
	float a = 0.006;//distance between central horizontal axes of arm and camrea
	float b = 0.09;	//distance on central horizontal axes to center of rotation of arm above wheel
	float c = 0.03;	//distance from center of rotation of arm above wheel - to ground
	float d = 0.05;	//distance from camera to center of robot
	double actual_y = -measuredpos[1];
	double actual_z = measuredpos[3];
//	printf("actual_y: %f \n",actual_y);
//	printf("actual_z: %f \n",actual_z);
//	printf("hinge: %f \n",hinge);

	double alfa = -PI + (PI / 2.0 - hinge / 2.0); //how to rotate y and z when image is rotated
//	printf("alfa %f\n", alfa );

	double otoceneY = cos(alfa) * actual_y - sin(alfa) * actual_z;
	double otoceneZ = sin(alfa) * actual_y + cos(alfa) * actual_z;
//	printf("otoceneY: %f \n",otoceneY);
//	printf("otoceneZ: %f \n",otoceneZ);
	measuredpos[0] = measuredpos[0] + 0.04;
	//old one
	//measuredpos[1]=sin(hinge/2)*0.05+(-cos((PI/2)-(hinge/2))*measuredpos[1]+sin((PI/2)-(hinge/2))*measuredpos[3]);
	measuredpos[1] = sin(hinge / 2) * d - otoceneY;
	measuredpos[2] = -measuredpos[2];
	measuredpos[3] = c + b * sin((PI / 2) - (hinge / 2))
			+ a * cos((PI / 2) - (hinge / 2)) + otoceneZ;	//z axis
}

void Map::convertCameraMeasurement(float measuredpos[], float hinge,
		RobotBase::RobotType type) {

	switch (type) {
	case RobotBase::ACTIVEWHEEL:
		convertCameraMeasurementAW(measuredpos, hinge);
		break;
	case RobotBase::SCOUTBOT:
		convertCameraMeasurementS(measuredpos);
		break;
	case RobotBase::KABOT:
		convertCameraMeasurementKB(measuredpos);
		break;
	default:
		printf("can not convert camera measurement with no robot type");
		break;
	}
}

void Map::calculateOdometryCovariance(double changedx, double changedy,
		double changedphi, double changedDL, double changedDR) {

	switch (this->robot_type) {
	case RobotBase::ACTIVEWHEEL:
		calculateOdometryCovarianceAW(changedx, changedy, changedphi);
		break;
	case RobotBase::SCOUTBOT:
		calculateOdometryCovarianceS(changedDL, changedDR, changedphi);
		break;
	case RobotBase::KABOT:
		calculateOdometryCovarianceKB(changedx, changedy, changedphi);
		break;
	default:
		printf("can not calculate odometry covariance with no robot type");
		break;
	}
}

void Map::calculateOdometryCovarianceAW(double dX, double dY, double dPHI) {

//odometry_covariance[0] =abs(dX)*0.000075321;
//odometry_covariance[1] =abs(dY)*0.000075321;
//odometry_covariance[2] =abs(dPHI)*0.0012177;
	odometry_covariance[0] = abs(dX) * 0.01;
	odometry_covariance[1] = abs(dY) * 0.01;
	double prvni[2] = { 0, dX };
	double druhy[2] = { 0, dY };

	odometry_covariance[2] = abs(dPHI) * 0.0012177 + 0, 174444444
			* euclideanDistance(prvni, druhy);
//+ (pow((ODOMETRY_DL_VARIANCE-ODOMETRY_DR_VARIANCE),2)/pow(b,4))*ODOMETRY_B_VARIANCE ;

}

void Map::calculateOdometryCovarianceKB(double dX, double dY, double dPHI) {

	odometry_covariance[0] = abs(dX) * 0.000075321;
	odometry_covariance[1] = abs(dY) * 0.000075321;
	odometry_covariance[2] = abs(dPHI) * 0.05;

//odometry_covariance[2] =abs(dPHI)*0.0012177;
//+ (pow((ODOMETRY_DL_VARIANCE-ODOMETRY_DR_VARIANCE),2)/pow(b,4))*ODOMETRY_B_VARIANCE ;

}

void Map::calculateOdometryCovarianceS(double dL, double dR,
		double changedphi) {
	double ODOMETRY_D_VARIANCE = (dL == 0 && dR == 0) ? 0 : 0.0000092871;
	if (dL != dR) {
		//printf("different dL and dR\n");
		double dRmindLna2 = pow(dR - dL, 2);
		double dLpdRl2dRmindL = (dL + dR) / (2 * (dR - dL));
		odometry_covariance[0] = pow(
				(b * dR / dRmindLna2)
						* (sin(
								gsl_matrix_get(this->predstate, 2, 0)
										+ changedphi)
								- sin(gsl_matrix_get(this->predstate, 2, 0)))
						+ dLpdRl2dRmindL
								* cos(
										gsl_matrix_get(this->predstate, 2, 0)
												+ changedphi), 2)
				* ODOMETRY_D_VARIANCE
				+ pow(
						(-b * dL / dRmindLna2)
								* (sin(
										gsl_matrix_get(this->predstate, 2, 0)
												+ changedphi)
										- sin(
												gsl_matrix_get(this->predstate,
														2, 0)))
								- dLpdRl2dRmindL
										* cos(
												gsl_matrix_get(this->predstate,
														2, 0) + changedphi), 2)
						* ODOMETRY_D_VARIANCE;
		odometry_covariance[1] = pow(
				(-b * dR / dRmindLna2)
						* (cos(
								gsl_matrix_get(this->predstate, 2, 0)
										+ changedphi)
								- cos(gsl_matrix_get(this->predstate, 2, 0)))
						+ dLpdRl2dRmindL
								* sin(
										gsl_matrix_get(this->predstate, 2, 0)
												+ changedphi), 2)
				* ODOMETRY_D_VARIANCE
				+ pow(
						(b * dL / dRmindLna2)
								* (cos(
										gsl_matrix_get(this->predstate, 2, 0)
												+ changedphi)
										- cos(
												gsl_matrix_get(this->predstate,
														2, 0)))
								- dLpdRl2dRmindL
										* sin(
												gsl_matrix_get(this->predstate,
														2, 0) + changedphi), 2)
						* ODOMETRY_D_VARIANCE;
		odometry_covariance[2] =
				(dL == 0 && dR == 0) ?
						0 : (2 / (b * b)) * (ODOMETRY_D_VARIANCE);
	} else {
		//printf("same dL and dR\n");
		odometry_covariance[0] = (0.5
				* pow(cos(gsl_matrix_get(this->predstate, 2, 0)), 2)
				+ (((dL + dR) * (dL + dR)) / (2 * b * b))
						* pow(sin(gsl_matrix_get(this->predstate, 2, 0)), 2))
				* ODOMETRY_D_VARIANCE;
		odometry_covariance[1] = (0.5
				* pow(sin(gsl_matrix_get(this->predstate, 2, 0)), 2)
				+ (((dL + dR) * (dL + dR)) / (2 * b * b))
						* pow(cos(gsl_matrix_get(this->predstate, 2, 0)), 2))
				* ODOMETRY_D_VARIANCE;
		odometry_covariance[2] = (2 / (b * b)) * ODOMETRY_D_VARIANCE;
	}
}

MappedObjectPosition* Map::getMappedPosition(int ithLM) {
	//ithLM is from 0 to mapsize-1

	if (ithLM < this->mapSize && ithLM > -1) {
		MappedObjectPosition* mappedObject = new MappedObjectPosition();
		mappedObject->type = mappedObjectTypes[ithLM];
		mappedObject->map_id = ithLM;
		mappedObject->xPosition = gsl_matrix_get(this->state, ithLM * 4 + 3, 0);
		mappedObject->yPosition = gsl_matrix_get(this->state, ithLM * 4 + 4, 0);
		mappedObject->phiPosition = gsl_matrix_get(this->state, ithLM * 4 + 5,
				0);
		mappedObject->zPosition = gsl_matrix_get(this->state, ithLM * 4 + 6, 0);
		return mappedObject;
	} else {
		printf("trying to get LM position out of map size\n");
		return NULL;
	}
}
/**
 * sequential determination of ith position of nearest LM
 */
int Map::nearestTypeID(double* position, MapObjectType type) {
	int num = -1;
	double dist = DBL_MAX;
	int var;
	for (var = 0; var < this->mapSize; ++var) {
		MappedObjectPosition* actualObj = this->getMappedPosition(var);
		double actual[2] = { actualObj->xPosition, actualObj->yPosition };
		double actualdist = euclideanDistance(position, actual);
		if (actualObj->type == type && actualdist < dist) {
			dist = actualdist;
			num = var;
		}
		delete actualObj;
	}
	return var;
}

int Map::saveObjectToMap(MappedObjectPosition* position,
		MappedObjectCovariance* covariance = NULL) {
	//state->
	int pozicevmape = this->mapSize; //pozice v mape je od 0-inf
	this->mapSize += 1;
	mappedObjectTypes.push_back(position->type);
	int oldsize = 4 * (this->mapSize - 1) + 3;
	int newsize = 4 * (this->mapSize) + 3;
	gsl_matrix *newstate = gsl_matrix_alloc(newsize, 1);
	gsl_matrix *newpredstate = gsl_matrix_alloc(newsize, 1);
	gsl_matrix *newP = gsl_matrix_calloc(newsize, newsize);
	gsl_matrix *newpredP = gsl_matrix_calloc(newsize, newsize);
	int var, var2;

	gsl_matrix_set(newstate, pozicevmape * 4 + 3, 0, position->xPosition);
	gsl_matrix_set(newpredstate, pozicevmape * 4 + 3, 0, position->xPosition);
	gsl_matrix_set(newstate, pozicevmape * 4 + 4, 0, position->yPosition);
	gsl_matrix_set(newpredstate, pozicevmape * 4 + 4, 0, position->yPosition);
	gsl_matrix_set(newstate, pozicevmape * 4 + 5, 0, position->phiPosition);
	gsl_matrix_set(newpredstate, pozicevmape * 4 + 5, 0, position->phiPosition);
	gsl_matrix_set(newstate, pozicevmape * 4 + 6, 0, position->zPosition);
	gsl_matrix_set(newpredstate, pozicevmape * 4 + 6, 0, position->zPosition);

	switch (position->type) {
	case NORMAL_CIRCLE: {
		gsl_matrix_set(newP, pozicevmape * 4 + 3, pozicevmape * 4 + 3,
				MEASUREMENT_XERROR);
		gsl_matrix_set(newpredP, pozicevmape * 4 + 3, pozicevmape * 4 + 3,
				MEASUREMENT_XERROR);
		gsl_matrix_set(newP, pozicevmape * 4 + 4, pozicevmape * 4 + 4,
				MEASUREMENT_YERROR);
		gsl_matrix_set(newpredP, pozicevmape * 4 + 4, pozicevmape * 4 + 4,
				MEASUREMENT_YERROR);
		gsl_matrix_set(newP, pozicevmape * 4 + 5, pozicevmape * 4 + 5,
				MEASUREMENT_PHIERROR);
		gsl_matrix_set(newpredP, pozicevmape * 4 + 5, pozicevmape * 4 + 5,
				MEASUREMENT_PHIERROR);
		gsl_matrix_set(newP, pozicevmape * 4 + 6, pozicevmape * 4 + 6,
				MEASUREMENT_ZERROR);
		gsl_matrix_set(newpredP, pozicevmape * 4 + 6, pozicevmape * 4 + 6,
				MEASUREMENT_ZERROR);
	}
		;
		break;
	case DOCK_CIRCLE: {
		gsl_matrix_set(newP, pozicevmape * 4 + 3, pozicevmape * 4 + 3,
				MEASUREMENT_XERROR);
		gsl_matrix_set(newpredP, pozicevmape * 4 + 3, pozicevmape * 4 + 3,
				MEASUREMENT_XERROR);
		gsl_matrix_set(newP, pozicevmape * 4 + 4, pozicevmape * 4 + 4,
				MEASUREMENT_YERROR);
		gsl_matrix_set(newpredP, pozicevmape * 4 + 4, pozicevmape * 4 + 4,
				MEASUREMENT_YERROR);
		gsl_matrix_set(newP, pozicevmape * 4 + 5, pozicevmape * 4 + 5,
				MEASUREMENT_PHIERROR);
		gsl_matrix_set(newpredP, pozicevmape * 4 + 5, pozicevmape * 4 + 5,
				MEASUREMENT_PHIERROR);
		gsl_matrix_set(newP, pozicevmape * 4 + 6, pozicevmape * 4 + 6,
				MEASUREMENT_ZERROR);
		gsl_matrix_set(newpredP, pozicevmape * 4 + 6, pozicevmape * 4 + 6,
				MEASUREMENT_ZERROR);
	}
		;
		break;
	default: {
		if (covariance == NULL) {
			gsl_matrix_set(newP, pozicevmape * 4 + 3, pozicevmape * 4 + 3,
					0.0005);
			gsl_matrix_set(newpredP, pozicevmape * 4 + 3, pozicevmape * 4 + 3,
					0.0005);
			gsl_matrix_set(newP, pozicevmape * 4 + 4, pozicevmape * 4 + 4,
					0.0005);
			gsl_matrix_set(newpredP, pozicevmape * 4 + 4, pozicevmape * 4 + 4,
					0.0005);
			gsl_matrix_set(newP, pozicevmape * 4 + 5, pozicevmape * 4 + 5,
					0.0005);
			gsl_matrix_set(newpredP, pozicevmape * 4 + 5, pozicevmape * 4 + 5,
					0.0005);
			gsl_matrix_set(newP, pozicevmape * 4 + 6, pozicevmape * 4 + 6,
					0.0005);
			gsl_matrix_set(newpredP, pozicevmape * 4 + 6, pozicevmape * 4 + 6,
					0.0005);
		} else {
			gsl_matrix_set(newP, pozicevmape * 4 + 3, pozicevmape * 4 + 3,
					covariance->xUncertainty);
			gsl_matrix_set(newpredP, pozicevmape * 4 + 3, pozicevmape * 4 + 3,
					covariance->xUncertainty);
			gsl_matrix_set(newP, pozicevmape * 4 + 4, pozicevmape * 4 + 4,
					covariance->yUncertainty);
			gsl_matrix_set(newpredP, pozicevmape * 4 + 4, pozicevmape * 4 + 4,
					covariance->yUncertainty);
			gsl_matrix_set(newP, pozicevmape * 4 + 5, pozicevmape * 4 + 5,
					covariance->phiUncertainty);
			gsl_matrix_set(newpredP, pozicevmape * 4 + 5, pozicevmape * 4 + 5,
					covariance->phiUncertainty);
			gsl_matrix_set(newP, pozicevmape * 4 + 6, pozicevmape * 4 + 6,
					covariance->zUncertainty);
			gsl_matrix_set(newpredP, pozicevmape * 4 + 6, pozicevmape * 4 + 6,
					covariance->zUncertainty);
		}
	}
		;
		break;
	}

	for (var = 0; var < oldsize; ++var) {
		gsl_matrix_set(newstate, var, 0, gsl_matrix_get(this->state, var, 0));
		gsl_matrix_set(newpredstate, var, 0,
				gsl_matrix_get(this->predstate, var, 0));
		for (var2 = 0; var2 < oldsize; ++var2) {
			gsl_matrix_set(newP, var, var2, gsl_matrix_get(this->P, var, var2));
			gsl_matrix_set(newpredP, var, var2,
					gsl_matrix_get(this->predP, var, var2));
		}
	}
//dealocating old matrices
	gsl_matrix_free(this->state);
	gsl_matrix_free(this->predstate);
	gsl_matrix_free(this->P);
	gsl_matrix_free(this->predP);
//setting new matrices
	this->state = newstate;
	this->predstate = newpredstate;
	this->P = newP;
	this->predP = newpredP;

}

void Map::addOtherRobotsObjects(CMessage message) {
	if (message.type == MSG_MAP_DATA) {
		MappedObjectPosition position;
		memcpy(&position, message.data, message.len);
		bool foundRobot = false;
		for (int var = 0; var < otherMapData.size(); ++var) {
			bool foundObject = false;
			if (otherMapData[var].robotID == position.mappedBy) {

				for (int var2 = 0;
						var2 < otherMapData[var].mappedObjects.size(); ++var2) {
					if (otherMapData[var].mappedObjects[var2].map_id
							== position.map_id) {
						foundObject = true;
						break;
					}
				}
				if (!foundObject) {
					otherMapData[var].mappedObjects.push_back(position);
				}
				foundRobot = true;
				break;
			}
		}
		if (!foundRobot) {
			OtherRobotMap otherMap;
			otherMap.robotID = position.mappedBy;
			otherMap.mappedObjects.push_back(position);
			otherMapData.push_back(otherMap);
		}
	}
}

