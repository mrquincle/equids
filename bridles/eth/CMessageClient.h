#ifndef CMESSAGECLIENT_H
#define CMESSAGECLIENT_H

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "CMessage.h"
#include "ipc.hh"
#include <semaphore.h>
#include <vector>
#include <pthread.h>
#include "wapi/wapi.h"

#define NUM_CONNECTIONS 100

void* serverLoop(void* serv);

class CMessageClient {

	IPC::IPC  jockey_IPC;
	CMessage mm;

public:

	CMessageClient(const char* ips);
	~CMessageClient();
	int initServer(const char* port);
	//void update(double odo[],bool buttons[],int rotation[],int irr[]);
	//		CMessage getMessage();

	//! Read a new message from the server, will return message with message type MSG_NONE if there is none
	const CMessage & getMessage();

	//! Send a new message, you have to deallocate msg.data yourself. This can immediately be done on returning
	void sendMessage(CMessage &msg) {
		jockey_IPC.SendData(msg.type, (uint8_t*)msg.data, msg.len);
	}

	//! Send a new message, you have to deallocate data yourself.
	void sendMessage(int type, void *data, int len) {
		jockey_IPC.SendData(type, (uint8_t*)data, len);
	}

	//bool getClientInfo(int socket,bool data[]);
	//CMessage checkForMessage();
	//int sendPosition(int socket,double buffer[]);
	//int sendDoubles(int socket,double buffer[],int len);
	//int sendInts(int socket,int buffer[],int len);
	//int sendBools(int socket,bool buffer[],int len);
	int closeConnection();
	int connected;
	//int serverSocket;
	//int mySocket;
	sem_t dataSem,connectSem;
	CMessage message;
	//double odometry[10];
	//bool buttons[10];
	//int rotation[1];
	//int ir[4];
	int last_ptr;
	const char* ip;
	std::vector<CMessage *> lastMessages;
};

#endif
