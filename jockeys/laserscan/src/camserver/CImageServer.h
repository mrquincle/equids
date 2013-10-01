/*
 * File name: CImageServer.h
 * Date:      2006/10/12 12:18
 * Author:    
 */

#ifndef __CIMAGESERVER_H__
#define __CIMAGESERVER_H__

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <string>
#include <semaphore.h>
#include <pthread.h>

#include <CRawImage.h>

#define NUM_CONNECTIONS 100
typedef struct 
{
	int socket;
	sem_t* sem;
}SClientInfoIMG;

/*typedef enum
{
	CM_GET = 0,
	CM_QUIT
}
ECameraMessage;*/

void* serverLoop(void* serv);

class CImageServer {
public:

	CImageServer(sem_t *imsem,CRawImage* image);
	~CImageServer();
	int initServer(const char* port);

	int checkForMessage(int socket);
	int sendImage(int socket);
	int closeConnection(int socket);
	void stopServer();

	int connected;
	int serverSocket;
	int mySocket;
	sem_t *dataSem;
	sem_t connectSem;
	sem_t captureSem;
	int messageRead;
	CRawImage *image;
	bool stop;

	//! Set prefix for log messages
	inline void setLogPrefix(std::string log_prefix) {
		this->log_prefix = log_prefix + "CImageServer: ";
	}
	std::string log_prefix;

};

#endif

/* end of CImageServer.h */
