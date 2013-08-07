/*
 * Mark.cpp
 *
 *  Created on: 15.11.2012
 *      Author: robert
 */

#include "Map.h"
#include <stdio.h>
#include <cmath>


using namespace std;

bool PRINT_MATRICES=false;
double Map::PI =3.141592653589793238462;
double ODOMETRY_XERROR = 0.00014;
double ODOMETRY_YERROR = 0.00003;
double ODOMETRY_PHIERROR = 0.0001;
double MEASUREMENT_XERROR = 0.0003;
double MEASUREMENT_YERROR = 0.0003;
double MEASUREMENT_PHIERROR = 0.007;
double ODOMETRY_ERROR_UNCERTAINTY=0.005;
double TOLERANCE =0.3;
double TOLERANCEPHI=0.2;
double addP=0.2;


Map::Map(double* odometry,int time) {
	printf("Map initializing....\n");
	//odometry error
	this->odometry = odometry;
	this->secondAngle = gsl_matrix_alloc (4, 1);

	this->velikostmapy=0;
	this->R = gsl_matrix_calloc (3, 3);
	gsl_matrix_set(this->R,0, 0, ODOMETRY_XERROR);
	gsl_matrix_set(this->R,1,1, ODOMETRY_YERROR);
	gsl_matrix_set(this->R,2,2, ODOMETRY_PHIERROR);
    //measurement error
	this->Q = gsl_matrix_calloc (3, 3);
    gsl_matrix_set(this->Q,0, 0, MEASUREMENT_XERROR);
    gsl_matrix_set(this->Q,1, 1, MEASUREMENT_YERROR);
    gsl_matrix_set(this->Q,2, 2, MEASUREMENT_PHIERROR);
    //initiate robot state to 0 0 0
    this->state = gsl_matrix_calloc (3, 1);
    this->predstate = gsl_matrix_calloc (3, 1);
    //initiate covariance P to null 3x3 matrix
    this->P = gsl_matrix_calloc (3, 3);
	gsl_matrix_set(this->P,0, 0, ODOMETRY_XERROR);
	gsl_matrix_set(this->P,1,1, ODOMETRY_YERROR);
	gsl_matrix_set(this->P,2,2, ODOMETRY_PHIERROR);
    this->predP = gsl_matrix_calloc (3, 3);
	gsl_matrix_set(this->predP,0, 0, ODOMETRY_XERROR);
	gsl_matrix_set(this->predP,1,1, ODOMETRY_YERROR);
	gsl_matrix_set(this->predP,2,2, ODOMETRY_PHIERROR);
    this->time=time;
    printf("Map initialized\n");
}

Map::~Map() {
	gsl_matrix_free (this->state);
	gsl_matrix_free (this->P);
	gsl_matrix_free (this->Q);
    gsl_matrix_free (this->R);
}

void Map::filter(double robpos[],double measuredpos[], int time) {
		//printf("Filtering....\n");
		int timechanged=this->time-time;
		this->time=time;
		if(PRINT_MATRICES){
		printf("measuredpos=[");
		printArray(measuredpos,3);
		printf("]");
		}
		double changedx=robpos[0]-this->odometry[0];
		double changedy=robpos[1]-this->odometry[1];
		double changedphi=robpos[2]-this->odometry[2];
		//printf("%1.10f , %1.10f , %1.10f",changedx,changedy,changedphi);
		this->odometry=robpos;

		gsl_matrix_memcpy(this->predstate,this->state);

		int pocetprvku =this->velikostmapy*3+3;
		gsl_matrix_set(this->predstate,0, 0, gsl_matrix_get(this->state, 0, 0)+(changedx));
		gsl_matrix_set(this->predstate,1, 0, gsl_matrix_get(this->state, 1, 0)+(changedy));
		gsl_matrix_set(this->predstate,2, 0, gsl_matrix_get(this->state, 2, 0)+(changedphi));
		gsl_matrix_set(this->predstate,2, 0, normalizeAngle(gsl_matrix_get(this->predstate, 2, 0)));

	//6    predP=G*P*G' + F'*R*F'
		gsl_matrix_memcpy(this->predP,this->P);

		gsl_matrix_set(this->predP,0, 0, ODOMETRY_ERROR_UNCERTAINTY*(changedx*changedx)+gsl_matrix_get(this->P, 0, 0));
		gsl_matrix_set(this->predP,1, 1, ODOMETRY_ERROR_UNCERTAINTY*(changedy*changedy)+gsl_matrix_get(this->P, 1, 1));
		gsl_matrix_set(this->predP,2, 2, ODOMETRY_ERROR_UNCERTAINTY*(changedphi*changedphi)+gsl_matrix_get(this->P, 2, 2));



		if(PRINT_MATRICES){
					printf("P:\n");
					printMatrix(this->P);
					printf("predP:\n");
					printMatrix(this->predP);
			}


	//9 computing position of measurement in global space
		double measuredToCenterX =   cos(robpos[2])*measuredpos[0]-sin(robpos[2])*measuredpos[1]+robpos[0];
		double measuredToCenterY =   sin(robpos[2])*measuredpos[0]+cos(robpos[2])*measuredpos[1]+robpos[1];
		double measuredToCenterPHI = normalizeAngle(robpos[2]-measuredpos[2]);
		double measuredToCenterPHI2= normalizeAngle(robpos[2]+measuredpos[2]);
		if(PRINT_MATRICES){
					printf("MPTC %2.6f , %2.6f , %2.6f , %2.6f;... \n", measuredToCenterX, measuredToCenterY,measuredToCenterPHI,measuredToCenterPHI2);
					}
		//printf("after measured to center\n");
		bool mapatoNEobsahuje=true;
		int pozicevmape=-1;
		int sign=1;
		int var;
		for (var = 0; var < this->velikostmapy; ++var) {
			if(gsl_matrix_get(this->predstate, var*3+3, 0)-TOLERANCE<measuredToCenterX && gsl_matrix_get(this->predstate, var*3+3, 0)-TOLERANCE<measuredToCenterX){
				if(gsl_matrix_get(this->predstate, var*3+4, 0)-TOLERANCE<measuredToCenterY && gsl_matrix_get(this->predstate, var*3+4, 0)-TOLERANCE<measuredToCenterY){
							//je to ten objekt
						pozicevmape=var;
						mapatoNEobsahuje=false;

						if(gsl_matrix_get(this->secondAngle,3,pozicevmape)){
							double error12=normalizeAngleDiff(measuredToCenterPHI-gsl_matrix_get(this->predstate, var*3+5, 0));
							double error21=normalizeAngleDiff(measuredToCenterPHI2-gsl_matrix_get(this->predstate, var*3+5, 0));
							double pole[2]={error12,error21};
							int minindex=minIndex(pole,2);
							switch (minindex) {
								case 0:
									sign=1;
									break;
								case 1:
									sign=-1;
									break;
							}

						}else{
							//not yes sure which angle
							double error11=normalizeAngleDiff(measuredToCenterPHI-gsl_matrix_get(this->secondAngle, 0, var));
							double error12=normalizeAngleDiff(measuredToCenterPHI-gsl_matrix_get(this->secondAngle, 1, var));
							double error21=normalizeAngleDiff(measuredToCenterPHI2-gsl_matrix_get(this->secondAngle, 0, var));
							double error22=normalizeAngleDiff(measuredToCenterPHI2-gsl_matrix_get(this->secondAngle, 1, var));
							double pole[4]={error11,error12,error21,error22};
							int minindex=minIndex(pole,4);
							switch (minindex) {
								case 0:
									sign=1;
									if(gsl_matrix_get(this->secondAngle, 2, var)==1){
										if(error22>TOLERANCEPHI){
											gsl_matrix_set(this->secondAngle,3,pozicevmape,1);
										}
									}else{
										gsl_matrix_set(this->state,var*3+5,0,gsl_matrix_get(this->secondAngle, 0, var));
										gsl_matrix_set(this->predstate,var*3+5,0,gsl_matrix_get(this->secondAngle, 0, var));
										gsl_matrix_set(this->secondAngle, 2, var,1);
									}
									break;
								case 1:
									sign=1;
									if(gsl_matrix_get(this->secondAngle, 2, var)==1){
										gsl_matrix_set(this->state,var*3+5,0,gsl_matrix_get(this->secondAngle, 1, var));
										gsl_matrix_set(this->predstate,var*3+5,0,gsl_matrix_get(this->secondAngle, 1, var));
										gsl_matrix_set(this->secondAngle, 2, var,2);

									}else{
										if(error21>TOLERANCEPHI){
											gsl_matrix_set(this->secondAngle,3,pozicevmape,1);
										}
									}
									break;
								case 2:
									sign=-1;
									if(gsl_matrix_get(this->secondAngle, 2, var)==1){
										if(error12>TOLERANCEPHI){
											gsl_matrix_set(this->secondAngle,3,pozicevmape,1);
										}
										}else{
											gsl_matrix_set(this->state,var*3+5,0,gsl_matrix_get(this->secondAngle, 0, var));
											gsl_matrix_set(this->predstate,var*3+5,0,gsl_matrix_get(this->secondAngle, 0, var));
											gsl_matrix_set(this->secondAngle, 2, var,1);
										}
									break;
								case 3:
									sign=-1;
									if(gsl_matrix_get(this->secondAngle, 2, var)==1){
										gsl_matrix_set(this->state,var*3+5,0,gsl_matrix_get(this->secondAngle, 1, var));
										gsl_matrix_set(this->predstate,var*3+5,0,gsl_matrix_get(this->secondAngle, 1, var));
										gsl_matrix_set(this->secondAngle, 2, var,2);
									}else{
										if(error11>TOLERANCEPHI){
										gsl_matrix_set(this->secondAngle,3,pozicevmape,1);
										}
									}
									break;
							}
						}
						break;
			    }
			}
		}
		//odometry phi just for shorter write
		double predictedPhi=gsl_matrix_get(this->predstate, 2, 0);

		if(mapatoNEobsahuje){
		//	printf("new Landmark\n");

			pozicevmape=this->velikostmapy; //pozice v mape je od 0-inf
			this->velikostmapy+=1;
			if(PRINT_MATRICES){
			printf("velikost mapy %d \n", this->velikostmapy);

			}
			int oldsize=3*(this->velikostmapy-1)+3;
			int newsize=3*(this->velikostmapy)+3;
			pocetprvku=newsize;
			gsl_matrix *newsecondAngle = gsl_matrix_alloc (4, this->velikostmapy);
			gsl_matrix *newstate = gsl_matrix_alloc (newsize, 1);
			gsl_matrix *newpredstate = gsl_matrix_alloc (newsize, 1);
			gsl_matrix *newP = gsl_matrix_calloc (newsize, newsize);
			gsl_matrix *newpredP = gsl_matrix_calloc (newsize, newsize);
			int var,var2;
			for (var = 0; var < oldsize; ++var) {
				gsl_matrix_set(newstate,var, 0,gsl_matrix_get(this->state, var, 0));
				gsl_matrix_set(newpredstate,var, 0,gsl_matrix_get(this->predstate, var, 0));
				for (var2 = 0; var2 < oldsize; ++var2) {
					gsl_matrix_set(newP,var, var2,gsl_matrix_get(this->P, var, var2));
					gsl_matrix_set(newpredP,var, var2,gsl_matrix_get(this->predP, var, var2));
				}
			}
			for (var = 0; var < this->velikostmapy-1; ++var) {
				gsl_matrix_set(newsecondAngle,0, var,gsl_matrix_get(this->secondAngle, 0, var));
				gsl_matrix_set(newsecondAngle,1, var,gsl_matrix_get(this->secondAngle, 1, var));
				gsl_matrix_set(newsecondAngle,2, var,gsl_matrix_get(this->secondAngle, 2, var));
				gsl_matrix_set(newsecondAngle,3, var,gsl_matrix_get(this->secondAngle, 3, var));
			}
			gsl_matrix_set(newsecondAngle,0, this->velikostmapy-1,measuredToCenterPHI);
			gsl_matrix_set(newsecondAngle,1, this->velikostmapy-1,measuredToCenterPHI2);
			gsl_matrix_set(newsecondAngle,2, this->velikostmapy-1,1);
			gsl_matrix_set(newsecondAngle,3, this->velikostmapy-1,0);
			if(PRINT_MATRICES){
			printf("new secondangle:\n");
			printMatrix(newsecondAngle);
			}
			//setting new state and predicted state values
			int i=0;
			gsl_matrix_set(newstate,pozicevmape*3+3, 0,measuredToCenterX);
			gsl_matrix_set(newpredstate,pozicevmape*3+3, 0,measuredToCenterX);
			gsl_matrix_set(newstate,pozicevmape*3+4, 0,measuredToCenterY);
			gsl_matrix_set(newpredstate,pozicevmape*3+4, 0,measuredToCenterY);

			gsl_matrix_set(newstate,pozicevmape*3+5, 0,measuredToCenterPHI);
			gsl_matrix_set(newpredstate,pozicevmape*3+5, 0,measuredToCenterPHI);
			if(PRINT_MATRICES){
			printf("new state:\n");
			printMatrix(newstate);
			printf("new predictedstate:\n");
			printMatrix(newpredstate);
			}
			i=0;
			gsl_matrix *Pll = gsl_matrix_calloc (3, 3);
			gsl_matrix *Plx = gsl_matrix_calloc (3, oldsize);
			//jacoby matrix of measuredToCenter=g(robpos,measuredpos) by robpos
			gsl_matrix *Gr  = gsl_matrix_calloc (3, 3);
			////jacoby matrix of measuredToCenter=g(robpos,measuredpos) by measuredpos
			gsl_matrix *Gw  = gsl_matrix_calloc (3, 3);
			//setting parts of jacobian Gr
			gsl_matrix_set(Gr,0,0,1);
			gsl_matrix_set(Gr,1,1,1);
			gsl_matrix_set(Gr,0,2,-sin(predictedPhi)*measuredpos[0]-cos(predictedPhi)*measuredpos[1]);
			gsl_matrix_set(Gr,1,2,cos(predictedPhi)*measuredpos[0]-sin(predictedPhi)*measuredpos[1]);
			gsl_matrix_set(Gr,2,2,1);
			//setting parts of jacobian Gw
			gsl_matrix_set(Gw,0,0,cos(predictedPhi));
			gsl_matrix_set(Gw,0,1,-sin(predictedPhi));
			gsl_matrix_set(Gw,1,0,sin(predictedPhi));
			gsl_matrix_set(Gw,1,1,cos(predictedPhi));
			gsl_matrix_set(Gw,2,2,-1*sign);
			if(PRINT_MATRICES){
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
			gsl_matrix *Gw_Q  = gsl_matrix_calloc (3, 3);
			//Gw*Q
			gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0,Gw,this->Q,0.0,Gw_Q);
			gsl_matrix *Gw_Q_Gwt  = gsl_matrix_calloc (3, 3);
			//Gw*Q*Gw'
			gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0,Gw_Q,Gw,0.0,Gw_Q_Gwt);
			gsl_matrix *Gr_predP  = gsl_matrix_calloc (3, 3);
			//submatrix of P
			gsl_matrix_view P0_20_2=gsl_matrix_submatrix (this->predP, 0, 0, 3, 3);
			//Gr*predictedP(0:2,0:2)
			gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0,Gr,&P0_20_2.matrix,0.0,Gr_predP);
			gsl_matrix *Gr_predP_Grt  = gsl_matrix_calloc (3, 3);
			//Gw*R*Gw'
			gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0,Gr_predP,Gr,0.0,Gr_predP_Grt);
			//Pll=Gr*predictedP(0:2,0:2)*Gr'+Gw*Q*Gw'
			gsl_matrix_add (Pll, Gr_predP_Grt);
			gsl_matrix_add (Pll, Gw_Q_Gwt);

			gsl_matrix_view P0_20_end=gsl_matrix_submatrix (this->predP, 0, 0, 3, oldsize);
			gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0,Gr,&P0_20_end.matrix,0.0,Plx);
			if(PRINT_MATRICES){
			printf("Plx:\n");
			printMatrix(Plx);
			printf("Pll:\n");
			printMatrix(Pll);
			}


			//setting new covariance and predicted covariance values
			for (var = 0; var < 3; ++var) {

				for (var2 = 0; var2 < oldsize; ++var2) {
					gsl_matrix_set(newpredP,oldsize+var, var2,gsl_matrix_get(Plx, var, var2));
					gsl_matrix_set(newpredP,var2,var+oldsize,gsl_matrix_get(Plx, var, var2));
					}
				for (var2 = 0; var2 < 3; ++var2) {
					gsl_matrix_set(newpredP,var+oldsize, var2+oldsize,gsl_matrix_get(Pll, var, var2));
					}

			}
			if(PRINT_MATRICES){
			printf("newpredP:\n");
			printMatrix(newpredP);
			printf("newP:\n");
			printMatrix(newP);
			}
			//dealocationg old matrices
			gsl_matrix_free (this->state);
			gsl_matrix_free (this->predstate);
			gsl_matrix_free (this->P);
			gsl_matrix_free (this->predP);
			gsl_matrix_free (this->secondAngle);
			gsl_matrix_free (Gr);
		    gsl_matrix_free (Gw);
		    gsl_matrix_free (Gw_Q);
		    gsl_matrix_free (Gw_Q_Gwt);
		    gsl_matrix_free (Gr_predP);
		    gsl_matrix_free (Gr_predP_Grt);
		    gsl_matrix_free (Pll);
		    gsl_matrix_free (Plx);
			//setting new matrices
			this->state=newstate;
			this->predstate=newpredstate;
			this->P=newP;
			this->predP=newpredP;
			this->secondAngle=newsecondAngle;

		}

		//creating kalmans gain matrix
		gsl_matrix *K = gsl_matrix_calloc (pocetprvku , 3);
		//creating jacobi matrix enlarged to size of state
		gsl_matrix *H = gsl_matrix_calloc (3 , pocetprvku);

		//creating jacobi matrix enlarged to size of state
		double rozdilx = gsl_matrix_get(this->predstate, pozicevmape*3+3  , 0) - gsl_matrix_get(this->predstate, 0, 0);
	    double rozdily = gsl_matrix_get(this->predstate, pozicevmape*3+4, 0) - gsl_matrix_get(this->predstate, 1, 0);
	    if(PRINT_MATRICES){
	    printf("rozdilx:\n %2.6f \n",rozdilx);
		printf("rozdily:\n %2.6f \n",rozdily);
		printf("predictedPhi:\n %2.6f \n",predictedPhi);
	    }

		gsl_matrix_set(H,0 , 0,-cos(-predictedPhi));
		gsl_matrix_set(H,1 , 0,-sin(-predictedPhi));
		gsl_matrix_set(H,0 , 1, sin(-predictedPhi));
		gsl_matrix_set(H,1 , 1,-cos(-predictedPhi));
		gsl_matrix_set(H,0 , 2,sin(-predictedPhi)*rozdilx+cos(-predictedPhi)*rozdily);
		gsl_matrix_set(H,1 , 2,-cos(-predictedPhi)*rozdilx+sin(-predictedPhi)*rozdily);
		gsl_matrix_set(H,2 , 2,sign);
		gsl_matrix_set(H,0 , 3*pozicevmape+3  ,cos(-predictedPhi));
		gsl_matrix_set(H,1 , 3*pozicevmape+3  ,sin(-predictedPhi));
		gsl_matrix_set(H,0 , 3*pozicevmape+4,-sin(-predictedPhi));
		gsl_matrix_set(H,1 , 3*pozicevmape+4,cos(-predictedPhi));
		gsl_matrix_set(H,2 , 3*pozicevmape+5,-sign);
		if(PRINT_MATRICES){
		printf("H:\n");
		printMatrix(H);
		}
		//K=(predictedP*(H'))/(H*predictedP*(H')+Q);
		gsl_matrix *predP_Ht=gsl_matrix_calloc (pocetprvku , 3);
		//predictedP*(H')
		gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0,this->predP,H,0.0,predP_Ht);
		gsl_matrix *H_predP_Ht_Q=gsl_matrix_calloc (3, 3);
		//H*predictedP*(H')
		gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0,H,predP_Ht,0.0,H_predP_Ht_Q);
		//H*predictedP*(H')+Q
		gsl_matrix_add (H_predP_Ht_Q, this->Q);
		//(H*predictedP*(H')+Q)^-1
		if(PRINT_MATRICES){
		printf("H_predP_Ht_Q:\n");
		printMatrix(H_predP_Ht_Q);
		}
		gsl_matrix *invers = gsl_matrix_calloc (3, 3);
		gsl_permutation * permut = gsl_permutation_alloc (3);
		int s;
		gsl_linalg_LU_decomp (H_predP_Ht_Q, permut, &s);
		gsl_linalg_LU_invert (H_predP_Ht_Q, permut, invers);
		if(PRINT_MATRICES){
		printf("invers:\n");
		printMatrix(invers);
		}
		gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0,predP_Ht,invers,0.0,K);
		if(PRINT_MATRICES){
		printf("K:\n");
		printMatrix(K);
		}
		gsl_matrix *difference=gsl_matrix_alloc (3 , 1);
		gsl_matrix_set(difference,0 ,0,measuredpos[0]-(cos(-predictedPhi)*(rozdilx) - sin(-predictedPhi)*(rozdily)));
		gsl_matrix_set(difference,1 ,0,measuredpos[1]-(sin(-predictedPhi)*(rozdilx) + cos(-predictedPhi)*(rozdily)));
		gsl_matrix_set(difference,2 ,0,measuredpos[2]-sign*(normalizeAngle( predictedPhi - gsl_matrix_get(this->predstate, 3*pozicevmape+5, 0))));
		if(PRINT_MATRICES){

		printf("difference:\n");
		printMatrix(difference);
		}

		gsl_matrix *newP = gsl_matrix_calloc (velikostmapy*3+3, velikostmapy*3+3);
		gsl_matrix_memcpy (this->state, this->predstate);
		for (var = 0; var < velikostmapy*3+3; ++var) {
			//P=I
			gsl_matrix_set(newP,var ,var,1);
		}

		gsl_matrix *K_difference=gsl_matrix_calloc (velikostmapy*3+3, 1);
		//K*difference
		gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0,K,difference,0.0,K_difference);
		//state=predictedstate+K*difference
		if(PRINT_MATRICES){
		printf("predictedstate:\n");
		printMatrix(this->predstate);
		printf("K_difference:\n");
		printMatrix(K_difference);
		}
		gsl_matrix_add (this->state, K_difference);

		gsl_matrix *K_H=gsl_matrix_calloc (velikostmapy*3+3, velikostmapy*3+3);
		if(PRINT_MATRICES){
							printf("newP");
							printMatrix(newP);
							printf("K_H:\n");
							printMatrix(K_H);
		}
		//K*H
		gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0,K,H,0.0,K_H);
		//P=I-K*H
		gsl_matrix_sub (newP, K_H);
		if(PRINT_MATRICES){
							printf("newP");
							printMatrix(newP);

		}
		//P=(I-K*H)*predP
		gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0,newP,this->predP,0.0,this->P);

		if(PRINT_MATRICES){
							printf("P:\n");
							printMatrix(this->P);
							printf("predP:\n");
							printMatrix(this->predP);
		}

		gsl_matrix_free (newP);
		gsl_matrix_free (K_difference);
		gsl_matrix_free (K);
		gsl_matrix_free (K_H);
		gsl_matrix_free (predP_Ht);
		gsl_matrix_free (difference);
		if(true){
		for (var = 1; var <= this->velikostmapy; ++var) {
		printf("state%d=[state%d;%2.7f , %2.7f , %2.7f ]; \n",var,var,gsl_matrix_get(this->state, var*3, 0),gsl_matrix_get(this->state, var*3+1, 0),gsl_matrix_get(this->state, var*3+2, 0));
		}
		}

		}


MapData Map::readFromFile(const char* filename){
	FILE * file;
	 MapData data;
	if(file = fopen (filename, "rb")){

	      int mmap = 0;
	      int nmap = 0;
	      int mcov = 0;
	      int ncov = 0;
	      fscanf (file, "%u\n", &mmap);
	      fscanf (file, "%u\n\n", &nmap);
	      gsl_matrix * map = gsl_matrix_alloc (mmap, nmap);
	      gsl_matrix_fscanf (file, map);
          fscanf (file, "\n%u\n", &mcov);
          fscanf (file, "%u\n", &ncov);
          gsl_matrix * covariance = gsl_matrix_alloc (mcov, ncov);
          gsl_matrix_fscanf (file, covariance);
          fclose (file);
          data.map=map;
          data.covariance=covariance;
          return data;
	}else{
		printf("can not read matrix from file %s \n",filename);
		data.map=NULL;
		data.covariance=NULL;
		return data;
	}

}

int Map::writeToFile(const char* filename,MapData data){
	FILE * file;
	if(file = fopen (filename, "wb")){
		  fprintf(file,"%lu\n",data.map->size1);
		  fprintf(file,"%lu\n\n",data.map->size2);
		  gsl_matrix_fprintf(file, data.map,"%g");
		  fprintf(file,"\n");
		  fprintf(file,"%lu\n",data.covariance->size1);
		  fprintf(file,"%lu\n\n",data.covariance->size2);
		  gsl_matrix_fprintf(file, data.covariance,"%g");
		  fprintf(file,"\n");
          fclose (file);
          return 0;
	}else{
		printf("can not write matrix to file %s \n",filename);
		return 1;
	}

}
void Map::printMatrix(gsl_matrix *matrix){
	int var,var2;
	for (var = 0; var < matrix->size1; ++var) {
		for (var2 = 0; var2 < matrix->size2; ++var2) {
			printf("%1.4e ",gsl_matrix_get(matrix, var, var2));
		}
		printf("\n");
	}
}
void Map::printArray(double *array,int lenght){
	int var;
	for (var = 0; var < lenght; ++var) {
		printf("%2.6f ",array[var]);
	}
	printf("\n");
}

double Map::normalizeAngle(double angle){
	angle = fmod(angle,2*PI);
	if(angle>PI){
		angle=angle-2*PI;
	}
	return angle;
}

double Map::normalizeAngleDiff(double diff){
	diff = abs(diff);
	if(diff>PI){
		diff=abs(diff-2*PI);
	}
	return diff;
}

int Map::minIndex(double* pole,int delka){
	double min=pole[0];
	int index = 0;
	int var;
	for(var=1;var<delka;++var){
		if(pole[var]<=min){
			min=pole[var];
			index=var;
		}
	}
	//printf("I %d\n",index+1);
	return index;
}
