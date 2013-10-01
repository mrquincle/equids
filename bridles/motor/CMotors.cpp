/*
 * CMotors.cpp
 *
 * Created on: Dec 16, 2010
 * Author: anne
 */

#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/syslog.h>
#include <cassert>
#include <cmath>
#include <cstdio>

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
	log_prefix = "CMotors: ";
	log_level = LOG_NOTICE;
//	if (log_level >= LOG_INFO) {
//		std::cout << log_prefix << "Create motors object" << std::endl;
//	}
	if (robot_base == NULL) {
		std::cerr << log_prefix << "robot_base is null, error in instantiation!" << std::endl;
	}
	this->robot_base = robot_base;
	this->robot_type = robot_type;

	min_wheel_velocity = 20;  // minimum value send to the motors to make the robot move
	max_wheel_velocity = 100; // maximum value the motors can take
	min_speed = 0;   // the minimum value we accept from the user
	max_speed = 100; // the maximum value we accept from the user

	max_radius = 1000; // the maximum value
	axle_track = 10;

	motorOrientation1 = 1;
	motorOrientation2 = 1;
	motorOrientation3 = 1;

	buf[0]=0;
	buf[1]=0;
	buf[2]=0;
	buf[3]=0;
	buf[4]=0;
	buf[5]=0;
	actualspeed1=0;
	actualspeed2=0;
	actualspeed3=0;

	posX=0;
	posY=0;
	posPhi=0;
	dx=0;
	dy=0;
	dphi=0;

	timer = new CTimer();
	timer->start();
	lastTime=timer->getTime();
}

CMotors::~CMotors() {

}

/**
 * Initialize the motors. Enable high-power voltage that is required to drive the motors.
 */
void CMotors::init() {
	if (log_level >= LOG_INFO) {
		std::cout << log_prefix << "Enable motors, this turns on the high voltage circuitry on the robot" << std::endl;
	}

	// use an environmental variable REVERSE_TRACKS, by default reverse tracks is false
	char *reverse_tracks;
	reverse_tracks = getenv("MOTOR_ORIENTATION1");
	if (reverse_tracks) {
		std::string rev = std::string(reverse_tracks);
		std::transform(rev.begin(), rev.end(), rev.begin(), ::tolower);
		if (rev == "reversed") {
			motorOrientation1 = -1;
		}
	}
	reverse_tracks = getenv("MOTOR_ORIENTATION2");
	if (reverse_tracks) {
		std::string rev = std::string(reverse_tracks);
		std::transform(rev.begin(), rev.end(), rev.begin(), ::tolower);
		if (rev == "reversed") {
			motorOrientation2 = -1;
		}
	}
	reverse_tracks = getenv("MOTOR_ORIENTATION3");
	if (reverse_tracks) {
		std::string rev = std::string(reverse_tracks);
		std::transform(rev.begin(), rev.end(), rev.begin(), ::tolower);
		if (rev == "reversed") {
			motorOrientation3 = -1;
		}
	}

	readMotorOrientations();
	readCalibResult();

	calibratedSpeed=40;

	switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		odometry_koef1 = 1.186562e-06;
		odometry_koef2 = 1.436724e-06;
		odometry_koef3 = 8.650865e-01;
		calibratedSpeed=60;
		break;
	}
	case RobotBase::KABOT: {
		odometry_koef1 = 0.000000152;
		odometry_koef2 = 0.65;
		odometry_koef3 = 0.3713;
		break;
	}
	case RobotBase::SCOUTBOT: {
		odometry_koef1=0.000001253;
		odometry_koef2=0.000001253;
		odometry_koef3=0.12;
		break;
	}
	default:
		std::cerr << log_prefix << "can not set odometry coefficients" << std::endl;
		break;
	}

	if (motorOrientation1 < 0) {
		std::cout << log_prefix << "Motor 1 is in reverse" << std::endl;
	}
	if (motorOrientation2 < 0) {
		std::cout << log_prefix << "Motor 2 is in reverse" << std::endl;
	}
	if (motorOrientation3 < 0) {
		std::cout << log_prefix << "Motor 3 is in reverse" << std::endl;
	}

	this->go();
	srand(time(NULL));
}

void CMotors::calibrate(MotorCalibResult calibrationResult){
	this->odometry_koef1=calibrationResult.odometry_koef1;
	this->odometry_koef2=calibrationResult.odometry_koef2;
	this->odometry_koef3=calibrationResult.odometry_koef3;
	this->calibratedSpeed=calibrationResult.calibratedSpeed;
	if (log_level >= LOG_INFO) {
		std::cout << log_prefix << "calibrating on " << calibratedSpeed << std::endl;
	}
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
	if (log_level >= LOG_INFO) {
		std::cout << log_prefix << "Cap [speed,radius] to [" << speed << ',' << radius << ']' << std::endl;
	}

	int abs_radius = abs(radius);
	int abs_speed = abs(speed);

	if (abs_radius < axle_track && !speed) {
		// maybe something special for turning on the spot?
		left = 0;
		right = 0;
	} else {
		// map absolute speed to velocity
		int wheel_velocity = cmd_to_ctrl(abs_speed);
		if (log_level >= LOG_INFO) {
			std::cout << log_prefix << "Wheel velocity is " << wheel_velocity << std::endl;
		}

		left = (int) (wheel_velocity * (abs_radius + axle_track) / (abs_radius + axle_track / 2.0));
		right = (int) (wheel_velocity * abs_radius / (abs_radius + axle_track / 2.0));

		// now we have a problem if we have both full forwards and full turn, we extend 100 for one of the wheels
		// hence if one of them exceeds 100, we scale both by the most excessive value
		dobots::cap_scale<int,double>(left, right, min_wheel_velocity, max_wheel_velocity);
	}

//	right *= motorOrientation1;
//	left *= motorOrientation2;

	// add back the sign of the speed
	if (speed < 0) {
		left = -left;
		right = -right;
	}

	if (radius < 0) {
		int temp = left;
		left = right;
		right = temp;
	}

	if (log_level >= LOG_INFO) {
		std::cout << log_prefix << "Translated [speed,radius]=[" << speed << ',' << radius << ']' << " into " <<
			"[left,right]=[" << left << ',' << right << ']' << std::endl;
	}
}

/**
 * Function to move in a specific way. Forward can be positive and negative. Radius defines a circle with the robot
 * positioned, NOT in the center, but at the left or right side of the middle, see below. The robot is indicated with
 * "R", the radius is indicated with "r".
 *
 *                  *******
 *                ***********
 *              R *********** r
 *                ***********
 *                  *******
 *
 * The radius must be greater than the wheel base. The reason to have a function like this, is that not every robot can
 * turn on the spot. On the other hand, every robot can make a circle like above such that it in the end will end up at
 * "r".
 *
 * @param speed              Going forward or reverse, value should be between -100 and 100 (or it will be clipped)
 * @param radius             Pick a point to the left or right and consider that as the center of a circle with a
 *                           radius ending up at the center of your robot.
 */
void CMotors::setRadianSpeeds(int forward, int radius) {
	switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		if (log_level >= LOG_INFO) {
			std::cout << log_prefix << "Got command [forward,radius]=[" << forward << ',' << radius << ']' << std::endl;
		}
		int turn = 0;
		if (forward == 0) {
			if (log_level >= LOG_INFO) {
				std::cout << log_prefix << "Stop the wheels" << std::endl;
			}
//			set_to_zero();
			setSpeeds(0,0);
		} else {
			if (radius == -1) {
				turn = -50;
				forward = 0;
			} else if (radius == +1) {
				turn = 50;
				forward = 0;
			} else if (radius == -111) {
				if (forward > 0)
					turn = -50;
				else
					turn = +50;
			} else if (radius == +111) {
				if (forward > 0)
					turn = +50;
				else
					turn = -50;
			} else if (radius == 1000) { // go forward / backward (w.r.t. camera)
				turn = 0;
			}
			if (log_level >= LOG_INFO) {
				std::cout << log_prefix << "Send command to the wheels [forward,turn]=[" << forward << ',' << turn << ']' << std::endl;
			}
			setSpeeds(forward, turn);
		}
//		std::cerr << log_prefix <<"To be implemented" << std::endl;
//		int turn = radius / 10;
//		dobots::cap_range(turn, -100, 100);
//		ActiveWheel *bot = (ActiveWheel*)robot_base;
//		int left, right;
//		translate(forward, radius, left, right);
//		std::cout << log_prefix << "Send command to the wheels [left,right]=[" << left << ',' << right << ']' << std::endl;
//		bot->MoveWheelsFront(left, right);
//		bot->MoveWheelsRear(left, right);
		break;
	}
	case RobotBase::KABOT: {
		std::cerr << log_prefix << "To be implemented" << std::endl;
		KaBot *bot = (KaBot*)robot_base;
		//		bot->moveForward(forward);
		//		bot->moveRight(turn);
		break;
	}
	case RobotBase::SCOUTBOT: {
		ScoutBot *bot = (ScoutBot*)robot_base;
		int left, right;
		translate(forward, radius, left, right);
		if (log_level >= LOG_NOTICE) {
			std::cout << log_prefix << "Send command to the wheels [left,right]=[" << left << ',' << right << ']' << std::endl;
		}
		bot->Move(motorOrientation1*left,motorOrientation2*right);
		break;
	}
	default:
		std::cerr << log_prefix << "There is no way to drive a robot without knowing its layout" << std::endl;
		break;
	}
}

/**
 * Rotate on the spot. Actually depends on the battery level etc. Assumes degrees between [-180,180]
 */
void CMotors::rotate(int degrees) {
	switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		std::cerr << log_prefix << "To be implemented" << std::endl;
		ActiveWheel *bot = (ActiveWheel*)robot_base;
		int turn = 40;
		if (degrees < 0) {
			turn = -turn;
		}
		setSpeeds(0, turn);
		static const int us_per_degree = 100000; //25000;
		usleep(us_per_degree * abs(degrees));
		setSpeeds(0,0);
		break;
	}
	case RobotBase::KABOT: {
		std::cerr << log_prefix << "To be implemented" << std::endl;
		KaBot *bot = (KaBot*)robot_base;
		break;
	}
	case RobotBase::SCOUTBOT: {
		ScoutBot *bot = (ScoutBot*)robot_base;
		int left = 40; int right = -40;
		if (degrees < 0) {
			left = -left;
			right = -right;
		}
		if (log_level >= LOG_NOTICE) {
			std::cout << log_prefix << "Make wheels rotating [left,right]=[" << left << ',' << right << ']' << std::endl;
		}
		bot->Move(motorOrientation1*left,motorOrientation2*right);
		// rotations per second, in case left and right are +/- 40
		static const int us_per_degree = 20000; //25000;
		usleep(us_per_degree * abs(degrees));
		bot->Move(0,0);
		break;
	}
	default:
		std::cout << log_prefix << "There is no way to drive a robot without knowing its layout" << std::endl;
		break;
	}
}


/**
 * common function for all robots
 * forward is -100% to 100% of possible speed forward/backward.
 * turn is -100% to 100% of possible curving speed(-100% clockwise), it is curving
 * factor of set forward speed. Special case is setSpeeds(0, turn!=0) which is turning on place
 * Always try to use calibrated speed for motions to avoid nonlinearity in speed setting
 */
void CMotors::setSpeeds(int forward, int turn) {
	//std::cout << log_prefix << "robot type is %d\n",this->robot_type);
	switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		//drive differentially - means two down wheel speeds are equal
		//can not turn and forward 100% and 100% - therefore count equivalent
		//max turn is on place turning
		std::cout << "Set speeds to " << forward << " and " << turn << std::endl;
		int speedtop;
		int speeddown;
		float curving_factor=1-(abs(turn)/100.0);
		if(forward>0){
			if(turn>0){
				speedtop = forward*curving_factor;
				speeddown = forward;
			}else{
				speedtop = forward;
				speeddown = forward*curving_factor;
			}
		}else if (forward<0) {
			if(turn>0){
				speedtop = forward;
				speeddown = forward*curving_factor;
			}else{
				speedtop = forward*curving_factor;
				speeddown = forward;
			}
		}else {
			//special case when turning on place
			speedtop = -turn;
			speeddown = turn;
		}

		float top_faster=2*odometry_koef1/odometry_koef2;//count how much alone wheel must be faster
		setMotorSpeedsAW(-speeddown/top_faster,-speeddown/top_faster,speedtop);

		break;
	}
	case RobotBase::KABOT: {
		//max rotation is when one wheel is max speed and one null
		int speedfront;
		int speedrear;
		float curving_factor=1-(turn/500.0);
		if(forward>=turn){
			speedfront = forward;
			speedrear = forward;
		}else {
			if(forward>=0){
				speedfront = turn;
				speedrear = 0;
			}else{
				speedfront = 0;
				speedrear = turn;
			}
		}
		setMotorSpeedsKB(speedfront,speedrear);
		break;
	}
	case RobotBase::SCOUTBOT: {
		//drive forward with speed forward and rotate with speed turn
		//diferential drive
		int speedleft;
		int speedright;
		float curving_factor=1-(abs(turn)/100.0);
		if(forward>0){
			if(turn>0){
				speedleft = forward*curving_factor;
				speedright = forward;
			}else{
				speedleft = forward;
				speedright = forward*curving_factor;
			}
		}else if (forward<0) {
			if(turn>0){
				speedleft = forward;
				speedright = forward*curving_factor;
			}else{
				speedleft = forward*curving_factor;
				speedright = forward;
			}
		}else {
			//special case when turning on place
			speedleft = turn;
			speedright = -turn;
		}

		float leftfaster=odometry_koef1/odometry_koef2;
		setMotorSpeedsS(speedleft/leftfaster,-speedright);

		break;
		usleep(100000); // at least 0.1 second per command
	}
	default:
		std::cerr << log_prefix <<"There is no way to drive a robot without knowing its layout" << std::endl;
		break;
	}
}


/**
 * use carefully!!
 */
void CMotors::randomSpeeds(){
	this->setSpeeds(rand() % 30 + 30, rand() % 50 + 30 );
}

void CMotors::set_to_zero() {
	switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		ActiveWheel *bot = (ActiveWheel*)robot_base;
		bot->MoveWheelsFront(0,0);
		bot->MoveWheelsRear(0,0);
		break;
	}
	case RobotBase::KABOT: {
		KaBot *bot = (KaBot*)robot_base;
		bot->MoveScrewFront(0);
		bot->MoveScrewRear(0);
		break;
	}
	case RobotBase::SCOUTBOT: {
		ScoutBot *bot = (ScoutBot*)robot_base;
		bot->Move(0,0);
		break;
	}
	default:
		std::cerr << log_prefix <<"in function halt no robot type" << std::endl;
		break;
	}
}

//! Halt does not set last command, so go() can be used to continue
void CMotors::halt() {
	switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		ActiveWheel *bot = (ActiveWheel*)robot_base;
		bot->EnableMotors(false);
		break;
	}
	case RobotBase::KABOT: {
		KaBot *bot = (KaBot*)robot_base;
		bot->EnableMotors(false);
		break;
	}
	case RobotBase::SCOUTBOT: {
		ScoutBot *bot = (ScoutBot*)robot_base;
		bot->EnableMotors(false);
		break;
	}
	default:
		std::cerr << log_prefix <<"in function halt no robot type" << std::endl;
		break;
	}
}

void CMotors::go() {
	robot_base->EnableMotors(true);
	for (int var = 0; var < 4; ++var) {
		robot_base->enableAccelerometer(var,true);
	}

	/*switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		fstd::cout << log_prefix << stdout, "in go AW" << std::endl;
		ActiveWheel *bot = (ActiveWheel*)robot_base;
		fstd::cout << log_prefix << stdout, "in go AW 2" << std::endl;
		bot->EnableMotors(true);
		fstd::cout << log_prefix << stdout, "in go AW 3" << std::endl;
	break;
	}
	case RobotBase::KABOT: {
		fstd::cout << log_prefix << stdout, "in go KB" << std::endl;
		KaBot *bot = (KaBot*)robot_base;
		fstd::cout << log_prefix << stdout, "in go KB2" << std::endl;
		bot->EnableMotors(true);
		fstd::cout << log_prefix << stdout, "in go KB3" << std::endl;
	break;
	}
	case RobotBase::SCOUTBOT: {
		fstd::cout << log_prefix << stdout, "in go S" << std::endl;
		ScoutBot* bot = (ScoutBot*)robot_base;
		fstd::cout << log_prefix << stdout, "in go S2" << std::endl;
		usleep(500000);
		bot->EnableMotors(true);
		fstd::cout << log_prefix << stdout, "in go S3" << std::endl;
	break;
	}
	default:
	std::cerr << log_prefix <<"in go no robot base" << std::endl;
	break;
	}*/
}

void CMotors::setMotorSpeedsKB(int sFront,int sRear)
{
	if(robot_type==RobotBase::KABOT){
		countOdometryTimeKB(timer->getTime()-lastTime);
		if(actualspeed1!=sFront || actualspeed2!=sRear){
			actualspeed1=sFront;
			actualspeed2=sRear;
			KaBot *bot = (KaBot*)robot_base;
			bot->MoveScrewFront(motorOrientation1*actualspeed1);
			bot->MoveScrewRear(motorOrientation2*actualspeed2);
			usleep(10000);
		}
		lastTime=timer->getTime();
	}else{
		std::cout << log_prefix << "Can not move like KaBot" << std::endl;
	}
}

void CMotors::setMotorSpeedsAW(int leftD,int rightD,int top)
{
	if(robot_type==RobotBase::ACTIVEWHEEL){

		ActiveWheel *bot = (ActiveWheel*)robot_base;
		//buf[5] = bot -> GetHingeStatus().currentAngle*M_PI/180;
		countOdometryTimeAW(timer->getTime()-lastTime,buf[5]);
		if(actualspeed1!=leftD || actualspeed2!=rightD || actualspeed3!=top){
			actualspeed1=leftD;
			actualspeed2=rightD;
			actualspeed3=top;
			//dopredu

			bot->MoveWheelsFront(motorOrientation1*actualspeed1 , motorOrientation2*actualspeed2);
			bot->MoveWheelsRear(motorOrientation3*actualspeed3, 0);
		}
		lastTime=timer->getTime();
	}else{
		std::cout << log_prefix << "Can not move like AW" << std::endl;
	}
}

void CMotors::setMotorSpeedsS(int left,int right)
{	//std::cout << log_prefix << "setting speed Scout: %d %d\n",left, right);
	if(robot_type==RobotBase::SCOUTBOT){
		countOdometryTimeS(timer->getTime()-lastTime);
		if(actualspeed1!=left || actualspeed2!=right){
			actualspeed1=left;
			actualspeed2=right;
			ScoutBot *bot = (ScoutBot*)robot_base;
			bot->Move(motorOrientation1*actualspeed1,motorOrientation2*actualspeed2);
		}
		lastTime=timer->getTime();
	}else{
		std::cout << log_prefix << "Can not move like Scout" << std::endl;
	}

}

void CMotors::countOdometryTimeKB(int timediff){

	//float diagonala = sqrt(odometry_koef1 * odometry_koef1 + odometry_koef2 * odometry_koef2);

	float dfront = timediff * odometry_koef1 * actualspeed1;
	float drear = timediff * odometry_koef1 * actualspeed2;

	float delta = atan(odometry_koef2 / odometry_koef1); //uhel mezi rychlosti do strany a dopredu, atan(b/a)=delta
	float r = 0.3713;	//polovicni vydalenost sroubu
	dx = dfront * cos(posPhi + odometry_koef2) + drear * cos(posPhi - odometry_koef2);
	dy = dfront * sin(posPhi + odometry_koef2) + drear * sin(posPhi - odometry_koef2);
	dphi = (dfront * sin(posPhi + odometry_koef2) - drear * sin(posPhi - odometry_koef2)) * (r);

	posX += dx;
	posY += dy;
	posPhi += dphi;
	buf[0] = posX;
	buf[1] = posY;
	buf[2] = posPhi;
	buf[3] = dfront;
	buf[4] = drear;
}

void CMotors::countOdometryTimeAW(int timediff,double hinge){
	dphi=0;
	dx=0;
	dy=0;
	double delta=0.523598776;//60 stupnu
	//	double hinge=120;
	double L12=0.1051;
	double L3=2*sin(0.5*hinge)*0.105-0.05254;
	//std::cout << log_prefix << "%f\n",L3);
	L12=L12*odometry_koef3;
	L3=L3*odometry_koef3;

	//L3=0.1575;
	//L12=L12/1.225;
	//L3=L3/1.225;
	//std::cout << log_prefix << "timediff %d \n",timediff);
	double D=2*cos(delta)*(L12+L3*sin(delta));
	double ld=odometry_koef1*timediff*(-actualspeed1);
	double pd=odometry_koef1*timediff*(-actualspeed2);
	double h=odometry_koef2*timediff*(-actualspeed3);
	dx=(1/D)*((-L12*sin(posPhi)-L3*cos(delta-posPhi))*ld+(L12*sin(posPhi)-L3*cos(delta+posPhi))*pd+(2*L12*cos(delta)*cos(posPhi))*h);
	dy=(1/D)*((L12*cos(posPhi)+L3*sin(delta-posPhi))*ld+(-L12*cos(posPhi)-L3*sin(delta+posPhi))*pd+(2*L12*cos(delta)*sin(posPhi))*h);
	dphi=(1/D)*((cos(delta))*ld+(cos(delta))*pd+(sin(2*delta))*h);
	//std::cout << log_prefix << "timediff=[timediff %d];\n",timediff);
	posX -= dx;
	posY -= dy;
	double pok=sin(0.5*hinge)*0.135-0.0519615;
	pok=0.1575-0.105;
	posX += sin(posPhi)*pok ;
	posY += -cos(posPhi)*pok;
	posPhi += dphi;
	posX += -sin(posPhi)*pok ;
	posY += cos(posPhi)*pok;

	buf[0] = posX;
	buf[1] = posY;
	buf[2] = posPhi;

}

void CMotors::countOdometryTimeS(int timediff){
	//dl=timediff/(820000.0/leftSpeed);
	//dr=timediff/(820000.0/-rightSpeed);
	//hallData = robot->GetHallSensorValues2D();
	//std::cout << log_prefix << "pred:  R: %d L: %d\n",hallData.right,hallData.left);

	double dl=odometry_koef1*actualspeed1*timediff ;
	double dr=-odometry_koef2*actualspeed2*timediff;
	buf[3]=buf[3]+dl;
	buf[4]=buf[4]+dr;
	//std::cout << log_prefix << "ujel jsem levou odometry: %f\n",dl);
	//std::cout << log_prefix << "ujel jsem pravou odometry: %f\n",dr);
	dphi = (-dr+dl)/odometry_koef3;
	//	std::cout << log_prefix << "dphi: %f\n",dphi);

	if(dl==dr || dl==-dr){
		double stredniujeta=((dl + dr)/2.0);
		//std::cout << log_prefix << "stredniujeta: %f\n",stredniujeta);
		dx = stredniujeta*cos(posPhi+dphi);
		dy = stredniujeta*sin(posPhi+dphi);
	}else{
		float centric=(odometry_koef3*(dr+dl))/(2*(dr-dl));
		//std::cout << log_prefix << "centric: %f\n",centric);
		//std::cout << log_prefix << "posPhi: %f\n",posPhi);
		dx=-centric*(sin(posPhi+dphi)-sin(posPhi));
		dy=centric*(cos(posPhi+dphi)-cos(posPhi));
	}
	posX += dx;
	posY += dy;
	posPhi += dphi;
	buf[0] = posX;
	buf[1] = posY;
	buf[2] = posPhi;
	//std::cout << log_prefix << "uhel: %f\n",buf[2]);
}

void CMotors::setMotorPosition(float x,float y,float phi){
	this->buf[0]=x;
	this->buf[1]=y;
	this->buf[2]=phi;
	this->posX=x;
	this->posY=y;
	this->posPhi=phi;
	std::cout << log_prefix << "setting position " << buf[0] << ',' << buf[1] << ',' << buf[2] << std::endl;
}

double* CMotors::getPosition(){
	this->evaluatePosition();
	//this->buf[2]=normalizeAngle(this->buf[2]);
	return this->buf;
}
void CMotors::evaluatePosition(){
	//std::cout << log_prefix << "evaluate position" << std::endl;
	switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		this->setMotorSpeedsAW(	this->actualspeed1,this->actualspeed2,this->actualspeed3);
		break;
	}
	case RobotBase::KABOT: {
		this->setMotorSpeedsKB(this->actualspeed1,this->actualspeed2);
		break;
	}
	case RobotBase::SCOUTBOT: {
		this->setMotorSpeedsS(this->actualspeed1,this->actualspeed2);
		break;
	}
	default:
		std::cerr << log_prefix <<"cannot evaluate position for unknown robot type" << std::endl;
		break;
	}
}

bool CMotors::isMoving(){
	return this->actualspeed1!=0 || this->actualspeed2!=0 || this->actualspeed3!=0;
}

bool CMotors::readMotorOrientations() {
	FILE * file;
	int calibspeed;
	if (file = fopen("/flash/motorCalibration/motorOrientation.dat", "rb")) {

		fscanf(file, "%d\n%d\n%d\n", &motorOrientation1, &motorOrientation2,
				&motorOrientation3);
		std::cout << log_prefix << "read orientation of motors " << motorOrientation1 << ' ' << motorOrientation2 << ' '
				<< motorOrientation3 << std::endl;
		fclose(file);
		return true;
	} else {
		std::cout << log_prefix << "can not load motor orientation from file " << std::endl;
		return false;
	}
}

bool CMotors::readCalibResult() {
	FILE * file;
	double calibresult1;
	double calibresult2;
	double calibresult3;
	int calibspeed;
	if (file = fopen("/flash/motorCALIB.dat", "rb")) {
		fscanf(file, "%le\n%le\n%le\n%d\n", &calibresult1, &calibresult2,
				&calibresult3, &calibspeed);
		fclose(file);
		std::cout << log_prefix << "setting odometry coef 1,2,3: " << calibresult1 << ' ' << calibresult2 << ' ' <<
				calibresult3 << " and f " << calibspeed << " for speed" << std::endl;
		this->odometry_koef1 = calibresult1;
		this->odometry_koef2 = calibresult2;
		this->odometry_koef3 = calibresult3;
		this->calibratedSpeed = calibspeed;
		return true;
	} else {
		std::cout << log_prefix << "can not read calibresults from file " << std::endl;
		return false;
	}
}
