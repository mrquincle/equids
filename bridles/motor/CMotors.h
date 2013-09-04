/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Drive the robot's motors
 * @file CMotors.h
 *
 * This file is created at Almende B.V. and Distributed Organisms B.V. It is open-source software and belongs to a
 * larger suite of software that is meant for research on self-organization principles and multi-agent systems where
 * learning algorithms are an important aspect.
 *
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless, we personally strongly object
 * against this software being used for military purposes, factory farming, animal experimentation, and "Universal
 * Declaration of Human Rights" violations.
 *
 * Copyright (c) 2013 Anne C. van Rossum <anne@almende.org>
 *
 * @author    Anne C. van Rossum
 * @date      Jun 6, 2013
 * @project   Replicator
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#ifndef CMOTORS_H_
#define CMOTORS_H_

#include <IRobot.h>
#include <CTimer.h>

//@todo: remove dependency of motors on this shared file with the "eth" bridle
#include <messageDataType.h>

/* *********************************************************************************************************************
 * Interface of CMotors
 * ********************************************************************************************************************/

/**
 * The number and location of the motors is different for each robot. However, our controllers should not be bothered
 * by that. Hence, this is a little wrapper that converts commands that set the speeds into specific commands to the
 * wheels.
 */
class CMotors {
public:
	CMotors(RobotBase *robot_base, RobotBase::RobotType robot_type);

	~CMotors();

	void init();

	void setRadianSpeeds(int forward, int radius);

	void setSpeeds(int forward, int turn);

	void randomSpeeds();

	void reversed(bool left_right_reversed);

	void halt();

	void go();

	void set_to_zero();

	//! Speed and radius are translated into commands for left and right
	void translate(int speed, int radius, int & left, int & right);

	//! Rotate on the spot
	void rotate(int degrees);

	void setMotorPosition(float x,float y,float phi);
	void setMotorSpeedsKB(int sFront,int sRear);
	void setMotorSpeedsAW(int leftD,int rightD,int top);
	void setMotorSpeedsS(int left,int right);
	int actualspeed1;//screw front KB, track left Scout, leftDown AW
	int actualspeed2;//screw rear, track right Scout, rightDown AW
	int actualspeed3;// top AW
	double odometry_koef1; //srew front KB , track left Scout, rightDown and leftDown AW
	double odometry_koef2; //screw side KB , track right  Scout, top AW
	double odometry_koef3; //scout track,
	int calibratedSpeed;
	double* getPosition();
	bool isMoving();
	void calibrate(MotorCalibResult calibrationResult);

protected:

	//! From speed command to value for wheel velocity
	int cmd_to_ctrl(unsigned int speed);
private:
	//! Reference to the robot class and type
	RobotBase *robot_base;
	RobotBase::RobotType robot_type;
	CTimer* timer;
	int lastTime;
	double buf[10];
	float posX;
	float posY;
	float posPhi;
	float dx, dy, dphi;
	void countOdometryTimeKB(int timediff);//count dead reckoning position change for KaBot
	void countOdometryTimeAW(int timediff,double hinge);//count dead reckoning position change for ActiveWheel
	void countOdometryTimeS(int timediff);//count dead reckoning position change for ScoutBot
	void evaluatePosition();

	int max_radius;
	int min_wheel_velocity; // minimum value to the motors
	int max_wheel_velocity; // maximum value to the motors
	int min_speed; // minimum value to be used by user of this class
	int max_speed; // maximum value to be used by user of this class
	int axle_track; // length of the axle, distance between the centerline of the two wheels on both side of an axle
	bool left_right_reversed;
};

#endif /* CMOTOR_H_ */
