/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file CLeds.cpp
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
 * @date      Jul 4, 2013
 * @project   Replicator 
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

// infrared communication in irobot
#include <comm/IRComm.h>

#include <CLeds.h>

#include <CMotors.h>

#include <cassert>
#include <dim1algebra.hpp>
#include <syslog.h> // log-levels

/**
 * Create object that drives the leds, the normal ones, as well as the infrared ones.
 */
CLeds::CLeds(RobotBase *robot_base, RobotBase::RobotType robot_type) {
	motors = new CMotors(robot_base, robot_type);
	type = robot_type;
	robot = robot_base;
	irled_count = 8;
	board_count = 4;

	// assumes bias of zero, use calibrate() if that's not okay for your robots
	offset_reflective.resize(irled_count, 0);
	offset_ambient.resize(irled_count, 0);
	offset_proximity.resize(irled_count, 0);

	// histogram for each led, for each mode
	hist_reflective.add(irled_count);
	hist_ambient.add(irled_count);
	hist_proximity.add(irled_count);

	// set histogram options
	window_size = 100;
	hist_reflective.set_sliding_window(window_size);
	hist_ambient.set_sliding_window(window_size);
	hist_proximity.set_sliding_window(window_size);

	save_to_file = true;

	// get configuration data (for the leds, mainly calibration values)
	if(type == RobotBase::KABOT)
		optionfile_name = "/flash/morph/kit_option.cfg";
	else if(type == RobotBase::ACTIVEWHEEL)
		optionfile_name =  "/flash/morph/aw_option.cfg";
	else if(type == RobotBase::SCOUTBOT)
		optionfile_name = "/flash/morph/scout_option.cfg";
	optionfile.Load(optionfile_name);

	// temporary array to not have to query each ambient, reflective, etc. value from the MSP each time, but get them
	// all once
	ir_values.resize(board_count);

	log_level = LOG_EMERG;

	message_recv = false;

    pthread_mutex_init(&ir_rx_mutex, NULL);

    receiving_messages = false;
    messages_sent = 0;
}

CLeds::~CLeds() {
	receiving_messages = false;
	usleep(100000);
}

/**
 * Set the leds in the right initial mode. If a board is not running, a flag is set, so these values are not necessarily
 * taken into account by subsequent functions.
 */
void CLeds::init() {
	std::cout << "Turn off the normal LEDs for less inference" << std::endl;
	power_all(LT_NORMAL, false);

//	std::cout << "Turn on the reflective IR LEDs" << std::endl;
	// seems to work only when doing it twice in a row
//	power_all(LT_REFLECTIVE, true);
//	sleep(1);
//	power_all(LT_REFLECTIVE, true);
//	sleep(1);

	// correction, let us use the proximity mode
//	power_all(LT_PROXIMITY, true);
//	sleep(1);
//	power_all(LT_PROXIMITY, true);

	board_running.resize(board_count, 0);
	for (int i = 0; i < board_count; i++) {
		board_running[i] = robot->IsBoardRunning(i);
	}

	robot->SetIRLED(SPI_D, 0x7);
}

int CLeds::reflective(int i, bool offset) {
	assert (i < 8);
	int value = 0;
	int side = led_index_to_side(i);

	if (board_running[side]) {
		value = ir_values[i/2].sensor[1-i%2].reflective - (offset ? offset_reflective[i] : 0);
	}
	if (log_level >= LOG_DEBUG) std::cout << "Reflective value [" << i << "]: " << value << std::endl;
	return value;
}

int CLeds::ambient(int i, bool offset) {
	assert (i < 8);
	int value = 0;
	int side = led_index_to_side(i);
	if (board_running[side]) {
		value = ir_values[i/2].sensor[1-i%2].ambient - (offset ? offset_ambient[i] : 0);
	}
	return value;
}

int CLeds::proximity(int i, bool offset) {
	assert (i < 8);
	int value = 0;
	int side = led_index_to_side(i);
	if (board_running[side]) {
		value = ir_values[i/2].sensor[1-i%2].proximity - (offset ? offset_proximity[i] : 0);
	}
	return value;
}

void *IRCommRxThread(void *l) {
    std::cout << "Start IRCommRxThread" << std::endl;
    CLeds * leds = (CLeds*) l;
    leds->receiving_messages = true;
    while(leds->receiving_messages) {
        pthread_mutex_lock(&leds->ir_rx_mutex);
        while(IRComm::HasMessage()) {
            IRComm::ReadMessage();
            leds->message_recv = true;
        }
        pthread_mutex_unlock(&leds->ir_rx_mutex);

        usleep(20000);
    }
    std::cout << "Quit IRCommRxThread" << std::endl;
}

void CLeds::send_message(const std::string & msg) {
	static int side = 0;
	if (board_running[side])
		IRComm::SendMessage(side, msg.c_str(), msg.length());
	side = (side + 1) % board_count;
	usleep(20000);
	messages_sent++;
}

bool CLeds::message_received() {
	if (message_recv) {
		pthread_mutex_lock(&ir_rx_mutex);
		message_recv = false;
		pthread_mutex_unlock(&ir_rx_mutex);
		return true;
	}
	return false;
}

bool CLeds::encounter() {
	if (!receiving_messages) {
		int ret_rx = pthread_create(&ircomm_rx_thread, NULL, IRCommRxThread, this);
		if (ret_rx != 0) {
			perror("IRComm pthread_create.\n");
			return false;
		}
	}
	send_message("hello");
	return message_received();
}

/**
 * The leds are all attached to different boards, this function describes the mapping from led index to the board it is
 * attached to.
 *
 * todo: this is different for the activewheel
 */
int CLeds::led_index_to_side(int i) {
	int side;
	switch (i) {
	case 0: case 1: {
		side = robot->GetSide(RobotBase::FRONT);
		break;
	} case 2: case 3: {
		side = robot->GetSide(RobotBase::LEFT);
		break;
	} case 4: case 5: {
		side = robot->GetSide(RobotBase::REAR);
		break;
	} case 6: case 7: {
		side = robot->GetSide(RobotBase::RIGHT);
		break;
	}
	}
	return side;
}

/**
 * The actual queries to the MSPs, if anywhere, there should be delays build in between calls to the same MSP via this
 * function.
 */
void CLeds::sample() {
	for (int i = 0; i < 8; i++) {
		int side = led_index_to_side(i);
		if (board_running[side]) {
			ir_values[i/2] = robot->GetIRValues(side);
		}
	}
}

/**
 * The IRPULSE0 etc commands are defined in irobot/InterfaceTypes.h. What they mean you have to guess.
 */
void CLeds::power_all(LedType led_type, bool on) {
	switch (led_type) {
	case LT_REFLECTIVE:
		if (on)
			for (int i = 0; i < 4; ++i) {
				// turn on these leds
				robot->SetIRLED(i, IRLED0|IRLED1|IRLED2);
				// set all the LEDs to pulsing, in the case of the active wheel, there are 6, so IRPULSE runs till 5
				robot->SetIRPulse(i, IRPULSE0|IRPULSE1|IRPULSE2|IRPULSE3|IRPULSE4|IRPULSE5);
				//robot->SetIRLED(i, IRLEDOFF, LED0|LED1|LED2, IRPULSE0|IRPULSE1|IRPULSE2|IRPULSE3|IRPULSE4|IRPULSE5);
				// the IR mode is set to off, but the LEDs will be turned on in a different manner, with SetIRPulse
				robot->SetIRMode(i, IRLEDOFF);
				//  enable receiving on all the LEDs
				robot->SetIRRX(i, true);

				// calibrate IR, is not used by Wenguo
				robot->CalibrateIR(i);
				usleep(100000);
			}
		break;
	case LT_AMBIENT:
		std::cerr << "Todo: figure out these settings" << std::endl;
		if (on)
			for (int i = 0; i < 4; ++i) {
				robot->SetIRRX(i, false);
			}
		break;
	case LT_PROXIMITY:
		if (on) {
			// for all boards, turn on all IR leds, configure them all to pulse, and set the mode to "proximity"
			// this will cause the MSP to calculate a moving average over it
			for (int i = 0; i < 4; ++i) {
				robot->SetIRLED(i, IRLED0|IRLED1|IRLED2);
				robot->SetIRPulse(i, IRPULSE0|IRPULSE1|IRPULSE2);
				robot->SetIRMode(i, IRLEDPROXIMITY);
				robot->SetIRRX(i, true);
			}
		}
		break;
	case LT_NORMAL:
		for (int i = 0; i < 4; ++i)
			robot->SetLEDAll(i, on ? LED_RED : LED_OFF);
		break;
	}
}

/**
 * Transform color in enumeration format to a proper bitmask.
 */
void CLeds::color(LedColor color) {
	uint8_t c;
	colored = true;
	switch (color) {
	case LC_RED:          c = 0b00000011; break;
	case LC_YELLOW:       c = 0b00001111; break;
	case LC_GREEN:        c = 0b00001100; break;
	case LC_CYAN:         c = 0b00111100; break;
	case LC_BLUE:         c = 0b00110000; break;
	case LC_MAGENTA:      c = 0b00110011; break;
	case LC_WHITE:        c = 0b00111111; break;
	case LC_ORANGE:       c = 0b00000111; break;
	case LC_OFF: case LC_ENUM_SIZE: default:
		c = 0b00000000; break;
	}
	for (int i = 0; i < 4; ++i)
		robot->SetLEDAll(i, c);
}

void CLeds::colorToggle(LedColor color) {
	if(colored){
		this->color(LC_OFF);
		colored=false;
	} else {
		this->color(color);
		colored=true;
	}
}

// in FSM state WAITING
//switch on proximity ir leds and ir pulsing
//for(uint8_t i=0; i< NUM_DOCKS; i++)
//    SetIRLED(i, IRLEDOFF, LED1, IRPULSE0|IRPULSE1);

/**
 * This is the same way Wenguo calibrates the IR. The only difference is that he stores them to a file, so he only needs
 * to calibrate the robot once and then he uses this configuration. Both the reflective and the ambient IR leds are
 * calibrated.
 *
 * todo: also write the variance of the input. sensors that do not vary a lot depending on the orientation of the robot
 * are much better for collision avoidance etc.
 *
 * todo: check if the calibration can actually be done like this, or must be done separately for each mode
 */
void CLeds::calibrate(bool turn_around) {
	std::cout << "Turn off the normal LEDs for calibration purposes" << std::endl;
	power_all(LT_NORMAL, false);
	power_all(LT_REFLECTIVE, true);

	if (turn_around)
		motors->setSpeeds(0, 40);

	static int32_t count = 100;
	static int32_t between_rounds_sleep = 10000;
	std::cout << "We will turn for " << (double)((count * between_rounds_sleep) / (double)1000000) << " seconds, " <<
			"calibrating the infrared sensors" << std::endl;
	assert((count * between_rounds_sleep) < 60000000); // make sure it stays under a minute

	for (int t = 0; t < count; ++t) {
		update();
		for(int i=0; i < irled_count; i++)
		{
			offset_reflective[i] += reflective(i, false);
			offset_ambient[i] += ambient(i, false);
			offset_proximity[i] += proximity(i, false);
		}
		usleep(between_rounds_sleep);
	}

	// take average of every individual led over time
	for(int i=0;i<irled_count;i++) {
		offset_reflective[i] /= count;
		offset_ambient[i] /= count;
		offset_proximity[i] /= count;
	}

	// by default save to file
	if(save_to_file) {
		int entity = optionfile.LookupEntity("Global");
		if (entity < 0) {
			std::cerr << "The class \"worldfile\" is too basic to add entities or properties. " <<
					"You have to do that yourself manually in " << optionfile_name << std::endl;
		} else {
			char default_str[64];
			for(int i=0;i<irled_count;i++) {
				std::cout << "Write reflective offset " << offset_reflective[i] << " for led " << i << std::endl;
				std::cout << "Write ambient offset " << offset_ambient[i] << " for led " << i << std::endl;
				std::cout << "Write proximity offset " << offset_proximity[i] << " for led " << i << std::endl;

				snprintf(default_str, sizeof(default_str), "%d", offset_reflective[i]);
				optionfile.WriteTupleString(entity, "reflective_calibrated", i, default_str);
				snprintf(default_str, sizeof(default_str), "%d", offset_ambient[i]);
				optionfile.WriteTupleString(entity, "ambient_calibrated", i, default_str);
				snprintf(default_str, sizeof(default_str), "%d", offset_proximity[i]);
				optionfile.WriteTupleString(entity, "ambient_proximity", i, default_str);
			}
			optionfile.Save(optionfile_name);
		}
	}

	power_all(LT_REFLECTIVE, false);
	std::cout << "Turn the normal LEDs back on" << std::endl;
	power_all(LT_NORMAL);
}

/**
 * If previously calibrated, get the values from the file. If the file doesn't exist or the sections in the file do not
 * exist, it will just drop out.
 */
void CLeds::get_calibration() {
	optionfile.Load(optionfile_name);

	int entity = optionfile.LookupEntity("Global");
	if (entity < 0) {
		std::cerr << "The option file does not have an entity \"Global\"" << std::endl;
		return;
	}

	std::string property_name;
	property_name = "reflective_calibrated";
	if( CProperty* prop = optionfile.GetProperty( entity, property_name.c_str() ) ) {
		for(int i=0;i<irled_count;i++) {
			offset_reflective[i] = atoi(optionfile.GetPropertyValue(prop, i));
		}
	} else {
		std::cerr << "No property called " << property_name << std::endl;
	}
	property_name = "ambient_calibrated";
	if( CProperty* prop = optionfile.GetProperty( entity, property_name.c_str() ) ) {
		for(int i=0;i<irled_count;i++) {
			offset_ambient[i] = atoi(optionfile.GetPropertyValue(prop, i));
		}
	} else {
		std::cerr << "No property called " << property_name << std::endl;
	}
	property_name = "proximity_calibrated";
	if( CProperty* prop = optionfile.GetProperty( entity, property_name.c_str() ) ) {
		for(int i=0;i<irled_count;i++) {
			offset_proximity[i] = atoi(optionfile.GetPropertyValue(prop, i));
		}
	} else {
		std::cerr << "No property called " << property_name << std::endl;
	}

	std::cout << "Calibrated values for reflective IR sensors: ";
	dobots::print(offset_reflective.begin(), offset_reflective.end());

	std::cout << "Calibrated values for ambient sensors: ";
	dobots::print(offset_ambient.begin(), offset_ambient.end());

	std::cout << "Calibrated values for proximity sensors: ";
	dobots::print(offset_proximity.begin(), offset_proximity.end());

}

/**
 * Returns in 2000 milliseconds times the number of IR leds, so 0.016 seconds.
 */
void CLeds::update() {
	sample();

	int us = 1000;
	for (int i=0; i < irled_count; i++) {
		hist_ambient.push(i, ambient(i));
		usleep(us);
		hist_reflective.push(i, reflective(i));
		usleep(us);
		hist_proximity.push(i, proximity(i));
		usleep(us);
	}
	if (log_level >= LOG_INFO) {
		std::vector<int> values;
		values.resize(irled_count);

		hist_reflective.average(values.begin());
		std::cout << "Reflective (smoothed) ";
		dobots::print(values.begin(), values.end());

		hist_ambient.average(values.begin());
		std::cout << "Ambient (smoothed) ";
		dobots::print(values.begin(), values.end());

		hist_proximity.average(values.begin());
		std::cout << "Proximity (twice smoothed) ";
		dobots::print(values.begin(), values.end());

	}
}

/**
 * Just update this given sensor. It will not actually get information from the sensor itself, use it with sample() for
 * that.
 */
void CLeds::update(int i) {
	hist_ambient.push(i, ambient(i));
	hist_reflective.push(i, reflective(i));
	hist_proximity.push(i, proximity(i));
}

/**
 * The distance is calculated in the same way Wenguo does. By just averaging over a series of values and subtracting
 * the offset calculated in the calibration phase. We return the reflective LED values.
 *
 * Distance uses the reflective mode. So, this means passive objects like walls etc. can be measured. This is different
 * from the proximity mode, where an active object (another robot) is detected by the signals it emits.
 */
int CLeds::distance(int i) {
	update(i);
	//return hist_ambient.average(i);
	return hist_reflective.average(i);
}

/**
 * Returns a sign for the speed and returns the radius so that the robot drives away from closest point.
 * @param sign_speed         given(!) a speed, this function adds a sign
 * @param radius             radius will be returned
 *
 * Internal to this function this uses the IR LEDs (not the ambient ones). An angle of 0 corresponds to driving
 * straightforward. There are 8 LEDS, so they differ with 360/8=45 degrees. The LED at 0, is half that distance rotated
 * from forwards, so at -22.5 degrees, LED 1 is at +22.5 degrees, LED 2 at 67.5 degrees etc.
 *
 *       forward
 *     __1_____0__
 *    |           |
 *   2|           |7
 *    |           |
 *   3|           |6
 *    |___________|
 *       4     5
 *
 * The LEDs in the forward direction have larger weights, so the robot is tempted to go forwards. The final angle that
 * rolls out corresponds with the angle with the largest "vote". The robot will always go in a direction the opposite
 * of the highest value (which is assumed to be the most spacious direction).
 */
void CLeds::direction(int & sign_speed, int & radius) {
	update();

	float beta = 360 / irled_count; // = 360/8 is 45 degrees
	float start = -beta/2.0;

	// get the averaged values from each LED
	std::vector<float> avg_values;
	avg_values.resize(irled_count, 0);
	hist_reflective.average(avg_values.begin());

	std::cout << "Averages: ";
	for (int i = 0; i < avg_values.size(); ++i) {
		std::cout << avg_values[i] << ' ';
	}
	std::cout << std::endl;

	// not every LED is as important, weight the ones in the front more than the ones at the rear
	std::vector<float> weights;
	weights.resize(irled_count, 0);
	weights[0] = 1.7;
	weights[1] = 1.7;
	weights[2] = 1.4;
	weights[3] = 1.2;
	weights[4] = 1;
	weights[5] = 1;
	weights[6] = 1.4;
	weights[7] = 1.2;

	// multiply values with above weights
	std::transform(avg_values.begin(), avg_values.end(), weights.begin(), avg_values.begin(), std::multiplies<float>());

	std::cout << "Weighted averages: ";
	for (int i = 0; i < avg_values.size(); ++i) {
		std::cout << avg_values[i] << ' ';
	}
	std::cout << std::endl;

	// smooth the values with their neighbours
	std::vector<int32_t> values;
	values.resize(irled_count, 0);
	dobots::window_add(avg_values.begin(), avg_values.end(), values.begin(), 1);

	// get the "winning" LED
	std::vector<int32_t>::iterator iter;
	iter = std::max_element(values.begin(), values.end());
	int angle_index = std::distance(values.begin(), iter);

	// highest value is most dangerous, so pick the opposite side
	angle_index = (angle_index + 4) % 8;

	sign_speed = abs(sign_speed);

	// To turn quick you need a small radius
	int turn_quick = 20;

	// To turn slow (or go straight) you need a large radius
	int turn_slow = 1000;

	switch (angle_index) {
	case 0: {
		sign_speed = -sign_speed;
		radius = turn_slow;
		break;
	}
	case 1: {
		sign_speed = -sign_speed;
		radius = -turn_slow;
		break;
	}
	case 2: {
		sign_speed = -sign_speed;
		radius = -turn_quick;
		break;
	}
	case 7: {
		sign_speed = -sign_speed;
		radius = turn_quick;
		break;
	}
	case 3: {
		//sign_speed = +sign_speed;
		radius = -turn_quick;
		break;
	}
	case 6: {
		radius = turn_quick;
		break;
	}
	case 4: {
		radius = -turn_slow;
		break;
	}
	case 5: {
		radius = turn_slow;
		break;
	}

	}
	// make 4 directions is good enough
	//	angle_index /= 2;

	// get from index to angle
	//	angle = beta * 2 * angle_index; // + start
}


bool CLeds::collision() {
	update(LL_FRONT_LEFT);
	update(LL_FRONT_RIGHT);

	int threshold = 50;

	//return hist_ambient.average(i);
	int collision = 0;
	int avg_fl = hist_reflective.average((int)LL_FRONT_LEFT) ;
	int avg_fr = hist_reflective.average((int)LL_FRONT_RIGHT) ;
	std::cout << avg_fl << " and " << avg_fr << std::endl;

	// values go up if there is something close, in this case the right one responds much more indifferent to lighting
	// conditions
	if (avg_fr > threshold) {
		return true;
	}
	//	if (avg_fl + avg_fr > threshold) {
	//		return true;
	//	}
	return false;
}
