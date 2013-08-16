/*
 * CUbisencePosition.h
 *
 *  Created on: 14.7.2013
 *      Author: robert
 */

#ifndef CUBISENCEPOSITION_H_
#define CUBISENCEPOSITION_H_

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <semaphore.h>
#include <pthread.h>
#include <wapi/wapi.h>
#include <messageDataType.h>
using namespace std;
using namespace wapi;

void* serverLoopUP(void* serv);

class CUbisencePosition {
public:
	CUbisencePosition();
	virtual ~CUbisencePosition();
	int initServer(const int channel);
	int stopServer();
	UbiPosition getPosition();
	bool validPosition;
	bool threadFinished;
	WAPI* wapi;
	Coordinates* coordinates;
	sem_t dataSem;
	bool stop;
private:

	Ubitag* own_ubitag;
};

#endif /* CUBISENCEPOSITION_H_ */
