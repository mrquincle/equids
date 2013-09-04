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
#include <cassert>
#include <cmath>
#include <cstdio>
//#include <fcntl.h>
//#include <unistd.h>
//#include <sys/ioctl.h>

#include <CMotors.h>
#include <dim1algebra.hpp>

// Remark: better to use e.g. acos(-1)
#define PI 3.141592654

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
	axle_track = 10;
	left_right_reversed = false;

	printf("setting buf\n");
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
		fprintf(stderr, "can not set odometry koeficients\n");
		break;
	}
	fprintf(stdout, "time\n");
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
	printf("Enable motors, this turns on the high voltage circuitry on the robot\n");
	this->go();
	srand(time(NULL));
}

void CMotors::reversed(bool left_right_reversed) {
	this->left_right_reversed = left_right_reversed;
}

void CMotors::calibrate(MotorCalibResult calibrationResult){
	this->odometry_koef1=calibrationResult.odometry_koef1;
	this->odometry_koef2=calibrationResult.odometry_koef2;
	this->odometry_koef3=calibrationResult.odometry_koef3;
	this->calibratedSpeed=calibrationResult.calibratedSpeed;
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

	if (abs_radius < axle_track && !speed) {
		// maybe something special for turning on the spot?
		left = 0;
		right = 0;
	} else {
		// map absolute speed to velocity
		int wheel_velocity = cmd_to_ctrl(abs_speed);
		std::cout << "Wheel velocity is " << wheel_velocity << std::endl;

		left = (int) (wheel_velocity * (abs_radius + axle_track) / (abs_radius + axle_track / 2.0));
		right = (int) (wheel_velocity * abs_radius / (abs_radius + axle_track / 2.0));

		// now we have a problem if we have both full forwards and full turn, we extend 100 for one of the wheels
		// hence if one of them exceeds 100, we scale both by the most excessive value
		dobots::cap_scale<int,double>(left, right, min_wheel_velocity, max_wheel_velocity);

	}

	// for the ScoutBot, the right "wheel" is inverted
	//if (left_right_reversed) left = -left;
	if (left_right_reversed) right = -right;

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

	std::cout << "Translated [speed,radius]=[" << speed << ',' << radius << ']' << " into " <<
			"[left,right]=[" << left << ',' << right << ']' << std::endl;
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
		translate(forward, radius, left, right);
		std::cout << "Send command to the wheels [left,right]=[" << left << ',' << right << ']' << std::endl;
		bot->Move(left, right);
		break;
	}
	default:
		fprintf(stderr, "%s(): There is no way to drive a robot without knowing its layout\n", __func__);
		break;
	}
}

/**
 * Rotate on the spot. Actually depends on the battery level etc.
 */
void CMotors::rotate(int degrees) {
	switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		fprintf(stderr, "To be implemented\n");
		ActiveWheel *bot = (ActiveWheel*)robot_base;
		break;
	}
	case RobotBase::KABOT: {
		fprintf(stderr, "To be implemented\n");
		KaBot *bot = (KaBot*)robot_base;
		break;
	}
	case RobotBase::SCOUTBOT: {
		ScoutBot *bot = (ScoutBot*)robot_base;
		int left = 40;
		int right = left_right_reversed ? left : -left;
		std::cout << "Send command to the wheels [left,right]=[" << left << ',' << right << ']' << std::endl;
		bot->Move(left, right);
		sleep(degrees / 60);
		bot->Move(0,0);
		break;
	}
	default:
		fprintf(stderr, "There is no way to drive a robot without knowing its layout\n");
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
	switch (robot_type) {
	case RobotBase::ACTIVEWHEEL: {
		//drive diferentially - means two down wheel speeds are equal
		//can not turn and forward 100% and 100% - therefore count ekvivalent
		//max turn is on place turnning
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
		fprintf(stderr, "There is no way to drive a robot without knowing its layout\n");
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
		fprintf(stderr, "in function halt no robot type\n");
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
		fprintf(stderr, "in function halt no robot type\n");
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
		fprintf(stdout, "in go AW\n");
		ActiveWheel *bot = (ActiveWheel*)robot_base;
		fprintf(stdout, "in go AW 2\n");
		bot->EnableMotors(true);
		fprintf(stdout, "in go AW 3\n");
	break;
	}
	case RobotBase::KABOT: {
		fprintf(stdout, "in go KB\n");
		KaBot *bot = (KaBot*)robot_base;
		fprintf(stdout, "in go KB2\n");
		bot->EnableMotors(true);
		fprintf(stdout, "in go KB3\n");
	break;
	}
	case RobotBase::SCOUTBOT: {
		fprintf(stdout, "in go S\n");
		ScoutBot* bot = (ScoutBot*)robot_base;
		fprintf(stdout, "in go S2\n");
		usleep(500000);
		bot->EnableMotors(true);
		fprintf(stdout, "in go S3\n");
	break;
	}
	default:
	fprintf(stderr, "in go no robot base\n");
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
			bot->MoveScrewFront(actualspeed1);
			bot->MoveScrewRear(actualspeed2);
		}
		lastTime=timer->getTime();
	}else{
		std::cout << "Can not move like KaBot" << std::endl;
	}
}

void CMotors::setMotorSpeedsAW(int leftD,int rightD,int top)
{
	if(robot_type==RobotBase::ACTIVEWHEEL){

		ActiveWheel *bot = (ActiveWheel*)robot_base;
		//buf[5] = bot -> GetHingeStatus().currentAngle*PI/180;
		countOdometryTimeAW(timer->getTime()-lastTime,buf[5]);
		if(actualspeed1!=leftD || actualspeed2!=rightD || actualspeed3!=top){
			actualspeed1=leftD;
			actualspeed2=rightD;
			actualspeed3=top;
			//dopredu

			bot->MoveWheelsFront(actualspeed1 , actualspeed2);
			bot->MoveWheelsRear(actualspeed3, 0);
		}
		lastTime=timer->getTime();
	}else{
		std::cout << "Can not move like AW" << std::endl;
	}
}

void CMotors::setMotorSpeedsS(int left,int right)
{	//printf("setting speed Scout: %d %d\n",left, right);
	if(robot_type==RobotBase::SCOUTBOT){
		countOdometryTimeS(timer->getTime()-lastTime);
		if(actualspeed1!=left || actualspeed2!=right){
			actualspeed1=left;
			actualspeed2=right;
			ScoutBot *bot = (ScoutBot*)robot_base;
			bot->Move(actualspeed1,actualspeed2);
		}
		lastTime=timer->getTime();
	}else{
		std::cout << "Can not move like Scout" << std::endl;
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
	//printf("%f\n",L3);
	L12=L12*odometry_koef3;
	L3=L3*odometry_koef3;

	//L3=0.1575;
	//L12=L12/1.225;
	//L3=L3/1.225;

	double D=2*cos(delta)*(L12+L3*sin(delta));
	double ld=odometry_koef1*timediff*(-actualspeed1);
	double pd=odometry_koef1*timediff*(-actualspeed2);
	double h=odometry_koef2*timediff*(-actualspeed3);
	dx=(1/D)*((-L12*sin(posPhi)-L3*cos(delta-posPhi))*ld+(L12*sin(posPhi)-L3*cos(delta+posPhi))*pd+(2*L12*cos(delta)*cos(posPhi))*h);
	dy=(1/D)*((L12*cos(posPhi)+L3*sin(delta-posPhi))*ld+(-L12*cos(posPhi)-L3*sin(delta+posPhi))*pd+(2*L12*cos(delta)*sin(posPhi))*h);
	dphi=(1/D)*((cos(delta))*ld+(cos(delta))*pd+(sin(2*delta))*h);
	//printf("timediff=[timediff %d];\n",timediff);
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
	//printf("pred:  R: %d L: %d\n",hallData.right,hallData.left);

	double dl=odometry_koef1*actualspeed1*timediff ;
	double dr=-odometry_koef2*actualspeed2*timediff;
	buf[3]=buf[3]+dl;
	buf[4]=buf[4]+dr;
	//printf("ujel jsem levou odometry: %f\n",dl);
	//printf("ujel jsem pravou odometry: %f\n",dr);
	dphi = (-dr+dl)/odometry_koef3;
	//	printf("dphi: %f\n",dphi);

	if(dl==dr || dl==-dr){
		double stredniujeta=((dl + dr)/2.0);
		//printf("stredniujeta: %f\n",stredniujeta);
		dx = stredniujeta*cos(posPhi+dphi);
		dy = stredniujeta*sin(posPhi+dphi);
	}else{
		float centric=(odometry_koef3*(dr+dl))/(2*(dr-dl));
		//printf("centric: %f\n",centric);
		//printf("posPhi: %f\n",posPhi);
		dx=-centric*(sin(posPhi+dphi)-sin(posPhi));
		dy=centric*(cos(posPhi+dphi)-cos(posPhi));
	}
	posX += dx;
	posY += dy;
	posPhi += dphi;
	buf[0] = posX;
	buf[1] = posY;
	buf[2] = posPhi;
	//printf("uhel: %f\n",buf[2]);
}

void CMotors::setMotorPosition(float x,float y,float phi){
	this->buf[0]=x;
	this->buf[1]=y;
	this->buf[2]=phi;
	this->posX=x;
	this->posY=y;
	this->posPhi=phi;
	printf("setting position %f %f %f\n",buf[0],buf[1],buf[2]);
}

double* CMotors::getPosition(){
	this->evaluatePosition();
	//this->buf[2]=normalizeAngle(this->buf[2]);
	return this->buf;
}
void CMotors::evaluatePosition(){
	//printf("evaluate position\n");
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
		fprintf(stderr, "cannot evalueta position for unknown robot type\n");
		break;
	}
}

bool CMotors::isMoving(){
	return this->actualspeed1!=0 || this->actualspeed2!=0 || this->actualspeed3!=0;
}
