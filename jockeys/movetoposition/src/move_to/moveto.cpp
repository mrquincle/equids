/*
 * moveto.cpp
 *
 *  Created on: Jul 15, 2013
 *      Author: replicator
 */

#include "moveto.h"

float xKonc;
float yKonc;
float phiKonc;
float xPoc;
float yPoc;
float beta;
float phiActual;
float phiWanted;
float rUhel;
int P;
bool goalSet = false;

move_to::move_to(CMotors* motors, RobotBase::RobotType robot_type) {
	// TODO Auto-generated constructor stub
	motor = motors;
	typ = robot_type;
	alreadyTurned = false;
}

move_to::~move_to() {
	// TODO Auto-generated destructor stub
}

int sign(float a) {
	if (a > 0)
		return +1;
	if (a < 0)
		return -1;
	return 0;
}

int move_to::getTurn(float xPoc, float yPoc, float xKonc, float yKonc) {
	float smerniceX = xPoc - xKonc;
	float smerniceY = yPoc - yKonc;
	float normalaX = smerniceY;
	float normalaY = -smerniceX;
	//fprintf(stdout, "pred motorama\n");
	float x = motor->getPosition()[0];
	float y = motor->getPosition()[1];
	//fprintf(stdout,"return %i",sign(normalaX*x + normalaY*y));
	return sign(normalaX * x + normalaY * y);
}

int move_to::move(RobotPosition f, UbiPosition u) {
	int toReturn=0;
//	if (goalSet == false) {
	xKonc = f.x;
	yKonc = f.y;
	phiKonc = f.phi;
	xPoc = u.x;
	yPoc = u.y;
	beta = atan2(yKonc - yPoc, xKonc - xPoc);
//		goalSet = true;
//	}
	phiActual = motor->getPosition()[2];
	if (phiActual >= M_PI)
		phiActual -= 2 * M_PI;
	if (phiActual < -M_PI)
		phiActual += 2 * M_PI;
	printf("cil: %f %f %f aktual: %f %f phi aktual %f beta %f\n", xKonc, yKonc,
			phiKonc, u.x, u.y, phiActual, beta);
	//motor->setMotorPosition(u.x, u.y, phiActual);

	if (std::abs(u.x - f.x) > 0.15 || std::abs(u.y - f.y) > 0.15) {
		//int smer = getTurn(xPoc, yPoc, xKonc, yKonc);
		//fprintf(stdout, "smer : %i\n", smer);
		rUhel = phiActual - beta;
		if (rUhel >= M_PI)
			rUhel -= 2 * M_PI;
		if (rUhel < -M_PI)
			rUhel += 2 * M_PI;
		P = (int) 80 * rUhel;
		if (P > 60)
			P = 60;
		if (P < -60)
			P = -60;
		printf("setspeeeeeed: %i uhel: %f \n", P, rUhel);
		switch (typ) {
		case RobotBase::ACTIVEWHEEL: {
			if (abs(P) > 40) {
				motor->setSpeeds(0, -P);
				toReturn = 2;
				printf("tunrning on position \n");
			} else {
            if (abs(P)>15) {
				  motor->setSpeeds(60, -P*2);
            } else {
              motor->setSpeeds(60, -P*3);
            }               
				toReturn = 0;
			}

		}
			break;
		case RobotBase::SCOUTBOT: {
			//P = P;
         if (abs(P)>50) {
            motor->setSpeeds(50,P);
         } else if (abs(P)>20) {
            motor->setSpeeds(42,P);
         } else {
			   motor->setSpeeds(36, P);
         }
			toReturn= 0;
		}
			break;
		}

	} else {
		motor->setSpeeds(0, 0);
		toReturn = 1;
	}

	return toReturn;

//
//	float beta = std::atan2(y - motor->getPosition()[1],
//			x - motor->getPosition()[0]);
//
//	float vzd = std::sqrt(x * x + y * y);
//	float vzdActual = vzd
//			- std::sqrt(
//					motor->getPosition()[0] * motor->getPosition()[0]
//							+ motor->getPosition()[1]
//									* motor->getPosition()[1]);
//	float betaActual = std::atan2(y - motor->getPosition()[1],
//			x - motor->getPosition()[0]);
	//fprintf(stdout,"vzd : %f\n",vzd);
	//fprintf(stdout,"vzdActual : %f\n",vzdActual);
//	float vzdUjeta = std::sqrt(
//			motor->getPosition()[0] * motor->getPosition()[0]
//					+ motor->getPosition()[1] * motor->getPosition()[1]);

//	fprintf(stdout, "x: %f y: %f phi : %f\n", motor->getPosition()[0],
//			motor->getPosition()[1], motor->getPosition()[2]);
	//pocatecni otoceni
//	fprintf(stdout, "beta: %f, phiActual %f \n", beta, phiActual);
//	if (std::abs(std::abs(beta) - std::abs(phiActual)) > 0.16
//			&& (std::abs(std::abs(motor->getPosition()[0]) - std::abs(x)) > 0.05
//					|| std::abs(std::abs(motor->getPosition()[1]) - std::abs(y))
//							> 0.05)) { // >0.08  && !alreadyTurned
//		fprintf(stdout, "pocOtoceni \n");
//		int P;
//		if (std::abs(std::abs(beta) - std::abs(phiActual)) > 0.2)
//			P = 40; //P=40
//		else
//			P = 130 * std::abs(std::abs(beta) - std::abs(phiActual));
//
//		//motor->setSpeeds(P,sign(phiActual-phi)*(100-P));
//		if (P < 41) {
//			motor->setSpeeds(0, -sign(phiActual - beta) * P);
//			//motor->setSpeeds(0,-sign(phiActual-beta)*P);
//		} else {
//			fprintf(stdout, "ses debil\n");
//		}
//
//		phiActual = motor->getPosition()[2];
//		//usleep(50000);
//	} else if (std::abs(std::abs(motor->getPosition()[0]) - std::abs(x)) > 0.04
//			|| std::abs(std::abs(motor->getPosition()[1]) - std::abs(y))
//					> 0.04) { //vzdUjeta<vzd
//		alreadyTurned = true;
//		fprintf(stdout, "jizda \n");
//
//		int P = 40;
//		if (vzdActual > 0.05)
//			P = 40;
//		else
//			P = 40 * std::abs(vzdActual);
//		if (P < 35)
//			P = 35;
//		//motor->setSpeeds(P,sign(phiActual-phi)*(100-P));
//		if (P < 41) {
//			int smer = getTurn(xPoc, yPoc, x, y);
//			fprintf(stdout, "smer : %i\n", smer);
//
//			motor->setSpeeds(P, smer * 30);
//		} else {
//			fprintf(stdout, "ses debil\n");
//		}
//		//fprintf(stdout,"vzd : %f\n",vzd);
//		//fprintf(stdout,"vzsActual : %f\n",vzdActual);
//		//fprintf(stdout,"vzdUjeta : %f\n",vzdUjeta);
//
//		//fprintf(stdout,"x: %f\n",motor->getPosition()[0]);
//		//fprintf(stdout,"y : %f\n",motor->getPosition()[1]);
//		vzdUjeta = std::sqrt(
//				motor->getPosition()[0] * motor->getPosition()[0]
//						+ motor->getPosition()[1] * motor->getPosition()[1]);
//
//		vzdActual = std::sqrt(
//				(x - motor->getPosition()[0]) * (x - motor->getPosition()[0])
//						+ (y - motor->getPosition()[1])
//								* (y - motor->getPosition()[1]));
//		//usleep(5000);
//	} else if (std::abs(std::abs(phi) - std::abs(phiActual)) > 0.16) {
//
//		int P;
//		if (std::abs(std::abs(phi) - std::abs(phiActual)) > 0.2)
//			P = 40;
//		else
//			P = 130 * std::abs(std::abs(phi) - std::abs(phiActual));
//
//		//motor->setSpeeds(P,sign(phiActual-phi)*(100-P));
//		if (P < 41) {
//			motor->setSpeeds(0, sign(phi - phiActual) * P);
//		} else {
//			fprintf(stdout, "ses debil\n");
//		}
//		fprintf(stdout, "koncOtoceni %f > 0.08\n rychlost otaceni %f",
//				std::abs(std::abs(phi) - std::abs(phiActual)),
//				sign(phi - phiActual) * P);
//		fprintf(stdout, "x: %f\n", motor->getPosition()[0]);
//		fprintf(stdout, "y : %f\n", motor->getPosition()[1]);
//
//	} else {
//		motor->setSpeeds(0, 0);
//		fprintf(stdout, "x: %f\n", motor->getPosition()[0]);
//		fprintf(stdout, "y : %f\n", motor->getPosition()[1]);
//		//usleep(50000);
//		alreadyTurned = false;
//		return 1;
//	}
//	return 0;

}

int move_to::turn(RobotPosition f) {
	phiActual = motor->getPosition()[2];
	if (phiActual >= M_PI)
		phiActual -= 2 * M_PI;
	if (phiActual < -M_PI)
		phiActual += 2 * M_PI;
	phiWanted = f.phi;
	if (phiWanted >= M_PI)
		phiWanted -= 2 * M_PI;
	if (phiWanted < -M_PI)
		phiWanted += 2 * M_PI;

	rUhel = phiActual - phiWanted;
	if (rUhel >= M_PI)
		rUhel -= 2 * M_PI;
	if (rUhel < -M_PI)
		rUhel += 2 * M_PI;
	P = 1;
	printf("turning actual: %f wanted %f rUhel: %f PI: %f\n", phiActual, phiWanted,rUhel,M_PI);

//	if(rUhel<1){
//		P = std::abs(rUhel);
//	}
//	if(P < 0.5){
//		P = 0.5;
//	}

	if (std::abs(rUhel) > 0.17) {
		printf("nedotoceno\n");
		printf("tocim rychlosti %i, rozdil %f\n",(int)P*sign(rUhel)*motor->calibratedSpeed, rUhel);

		switch (typ) {
		case RobotBase::ACTIVEWHEEL: {
			motor->setSpeeds(0, (int)-P*sign(rUhel)*motor->calibratedSpeed);
		}
			break;
		case RobotBase::SCOUTBOT: {
			motor->setSpeeds(0, (int)-P*sign(rUhel)*motor->calibratedSpeed*0.7);
		}
			break;
		}
		return 0;
	} else {
		printf("dotoceno\n");
		motor->setSpeeds(0, 0);
		usleep(5000);
		return 1;

	}
}

