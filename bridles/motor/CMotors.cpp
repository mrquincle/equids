/*
 * CMotors.cpp
 *
 *  Created on: Dec 16, 2010
 *      Author: anne
 */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cassert>
#include <CMotors.h>

#include <dim1algebra.hpp>

/**
 * This class, in the end, will be able to drive robots in multiple ways. A holonomic drive (in which a robot can
 * translate in any direction it wants without rotating) is really nice. However, not all robots are mechanically
 * equipped with a holonomic drive. The ActiveWheel in the Replicator project has three omniwheels with which a
 * holonomic drive can be implemented. The Karslruhe robots have screwdrives and can in theory translate in any
 * direction. In theory, because the speed in some of these directions is so little, that it does not move on many
 * surfaces. The Scout robots have a tank drive. Driver-oriented control would take the orientation of the driver into
 * account. In autonomous robots this is of lesser concern. Joystick control is easy to implement, the direction and
 * the amplitude of the joystick can be directly translated in a velocity vector. In games there is a difference between
 * tank and arcade drive. We can make this all fancy, but it the end, what is required is a controller that can have
 * input controls ranging from 0 to 100, indicating something standard and achievable for most types of robots.
 *
 * Hence, we go for something that is called "speed" and "radius"
 *
 */
CMotors::CMotors(RobotBase *robot_base, RobotBase::RobotType robot_type) {
	printf("Create motors object\n");
	if (robot_base == NULL) {
		fprintf(stderr, "robot_base is null, error in instantiation!\n");
	}
	this->robot_base = robot_base;
	this->robot_type = robot_type;

	min_wheel_velocity = 20;  // minimum value send to the motors to make the robot move
	max_wheel_velocity = 100; // maximum value the motors can take
	min_speed = 0;   // the minimum value we accept from the user
	max_speed = 100; // the maximum value we accept from the user

	max_radius = 1000; // the maximum value
	axle_track = 20;
	left_right_reversed = true;
}

CMotors::~CMotors() {

}

/**
 * Initialize the motors. Enable high-power voltage that is required to drive the motors.
 */
void CMotors::init() {
	printf("Enable motors, this turns on the high voltage circuitry on the robot\n");
	robot_base->EnableMotors(true);
}

/**
 * By the default the input is between 0 and 100. However, the command to the wheel should be shifted with a tad, or
 * else little values do not make the robot move at all. The maximum speed should be scaled such that the maximum
 * control command is reached for this specific type of motor.
 */
int CMotors::cmd_to_ctrl(unsigned int abs_speed) {
	dobots::cap_range<unsigned int>(abs_speed, min_speed, max_speed);
	return ((abs_speed * (max_wheel_velocity - min_wheel_velocity)) / 100) + min_wheel_velocity;

}

/**
 * Function to translate forward and radius commands (integers) into left and right commands for the wheels. I first
 * followed http://code.google.com/p/cellbots/wiki/TranslatingUserControls but it doesn't make sense to have something
 * like "turn" for a differential robot.
 *
 * @param speed              Going forward or reverse, value should be between -100 and 100 (or it will be clipped)
 * @param radius             Pick a point to the left or right and consider that as the center of a circle with a
 *                           radius ending up at the center of your robot.
 *
 */
void CMotors::translate(int speed, int radius, int & left, int & right) {
	dobots::cap_range(radius, -max_radius, max_radius);
	dobots::cap_range(speed, -max_speed, max_speed);
	std::cout << "Cap [speed,radius] to [" << speed << ',' << radius << ']' << std::endl;

	int abs_radius = abs(radius);
	int abs_speed = abs(speed);

	// map absolute speed to velocity
	int wheel_velocity = cmd_to_ctrl(abs_speed);
	std::cout << "Wheel velocity is " << wheel_velocity << std::endl;

	left = (int) (wheel_velocity * (abs_radius + axle_track) / (abs_radius + axle_track / 2.0));
	right = (int) (wheel_velocity * abs_radius / (abs_radius + axle_track / 2.0));

	// now we have a problem if we have both full forwards and full turn, we extend 100 for one of the wheels
	// hence if one of them exceeds 100, we scale both by the most excessive value
	dobots::cap_scale<int,double>(left, right, min_wheel_velocity, max_wheel_velocity);

	// for the ScoutBot, the right "wheel" is inverted
	if (left_right_reversed) left = -left;

	// add back the sign of the speed
	if (speed < 0) {
		left = -left;
		right = -right;
	}

	std::cout << "Translated [speed,radius]=[" << speed << ',' << radius << ']' << " into " <<
			"[left,right]=[" << left << ',' << right << ']' << std::endl;
}

void CMotors::setSpeeds(int forward, int turn) {
	switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		fprintf(stderr, "To be implemented\n");
		ActiveWheel *bot = (ActiveWheel*)robot_base;
//		bot->moveForward(forward);
//		bot->moveRight(turn);
		break;
	}
	case RobotBase::KABOT: {
		fprintf(stderr, "To be implemented\n");
		KaBot *bot = (KaBot*)robot_base;
//		bot->moveForward(forward);
//		bot->moveRight(turn);
		break;
	}
	case RobotBase::SCOUTBOT: {
		ScoutBot *bot = (ScoutBot*)robot_base;
		int left, right;
		translate(forward, turn, left, right);
		std::cout << "Send command to the wheels [left,right]=[" << left << ',' << right << ']' << std::endl;
		bot->Move(left, right);
		break;
	}
	default:
		fprintf(stderr, "There is no way to drive a robot without knowing its layout\n");
		break;
	}
}

//! Halt does not set last command, so go() can be used to continue
void CMotors::halt() {
	robot_base->EnableMotors(false);
}

void CMotors::go() {
	robot_base->EnableMotors(true);
}
