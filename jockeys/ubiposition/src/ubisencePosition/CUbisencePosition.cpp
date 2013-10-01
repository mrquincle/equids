/*
 * CUbisencePosition.cpp
 *
 *  Created on: 14.7.2013
 *      Author: robert
 */

#include "CUbisencePosition.h"
#define WAIT_PROGRAM_uS 250000
#define WAIT_FOR_COORDINATES_TIMEOUT_MS 5000

CUbisencePosition::CUbisencePosition() {
	sem_init(&dataSem, 0, 1);
	own_ubitag = new Ubitag();
	coordinates = new Coordinates();
	wapi = new WAPI();
	stop = true;
	validPosition = false;
	threadFinished = false;
	thread = NULL;
}

CUbisencePosition::~CUbisencePosition() {
	wapi->disjoin();
	delete coordinates;
	delete wapi;
	delete own_ubitag;
}

void* connectLoopUP(void *serv) {
	int wapi_error;
	CUbisencePosition* ubisencePosition = (CUbisencePosition*) serv;
	while (!ubisencePosition->stop) {
		sem_wait(&ubisencePosition->dataSem);
		wapi_error = ubisencePosition->wapi->position(
				*ubisencePosition->coordinates,
				WAIT_FOR_COORDINATES_TIMEOUT_MS);
		sem_post(&ubisencePosition->dataSem);
		if (WAPI::WAPI_OK == wapi_error) {
			ubisencePosition->validPosition = true;
			//cout << "Position: " << ubisencePosition->coordinates->toString() << endl;
			usleep(WAIT_PROGRAM_uS);
		} else {
			ubisencePosition->validPosition = false;
			//std::cout << "Cannot get position wapi_error " << wapi_error << endl;
			usleep(WAIT_PROGRAM_uS);
		}
	}
	ubisencePosition->threadFinished = true;
	return NULL;
}

int CUbisencePosition::initServer(const int channel) {
	stop = false;
	int error =0;
	int wapi_error;
	assert (wapi != NULL);
	wapi_error = wapi->join(channel);
	if (WAPI::WAPI_OK != wapi_error) {
		std::cout << "Cannot join to channel for ubisence" << channel
				<< " wapi_error " << wapi_error << endl;
		return (EXIT_FAILURE);
	}
	std::cout << "Joined to channel " << channel << endl;
	do {
		wapi_error = wapi->nodeInfo(*own_ubitag);
		if (WAPI::WAPI_OK != wapi_error ) {

			std::cout << "Waiting for identity" << endl;
			usleep(WAIT_PROGRAM_uS);

		}
		if(error > 20){
			return -1;
		}
	} while (WAPI::WAPI_OK != wapi_error);
	thread = (pthread_t*) malloc(sizeof(pthread_t));
	pthread_create(thread, NULL, &connectLoopUP, (void*) this);
	return 0;
}

void CUbisencePosition::stopServer() {
	stop = true;
	usleep(30);
	pthread_cancel(*thread);
}

UbiPosition CUbisencePosition::getPosition() {
	UbiPosition position;
	sem_wait(&this->dataSem);
	position.x = this->coordinates->getX() / 1000.0;
	position.y = this->coordinates->getY() / 1000.0;
	position.z = this->coordinates->getZ() / 1000.0;
	position.time_stamp = this->coordinates->getTimestamp();
	sem_post(&this->dataSem);
	return position;
}
