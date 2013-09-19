/*
 * moveto.h
 *
 *  Created on: Jul 15, 2013
 *      Author: replicator
 */
#include "../motor/CMotors.h"
#include <IRobot.h>
#include <cmath>
#include "../eth/messageDataType.h"

#ifndef MOVETO_H_
#define MOVETO_H_

class move_to {
public:
	move_to(CMotors* motors,RobotBase::RobotType robot_type);
	virtual ~move_to();
	int move(RobotPosition,UbiPosition);
	int turn(RobotPosition);
	int getTurn(float xPoc,float yPoc, float xKonc,float yKonc);
private:
	CMotors* motor;
	RobotBase::RobotType typ;
	bool alreadyTurned;
	RobotPosition finalPosition;
	//int lastAvoid;
	//CTimer* timer;
};

#endif /* MOVETO_H_ */
