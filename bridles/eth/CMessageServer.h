/*
 * File name: CMessageServer.h
 * Date:      2006/10/12 12:18
 * Author:    
 */

#ifndef __CMESSAGESERVER_H__
#define __CMESSAGESERVER_H__

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
#include <pthread.h>

#define NUM_CONNECTIONS 100
typedef struct 
{
	bool write,odometry,rotation,buttons,ir;
	CMessage message;
	int time;
	int socket;
	sem_t* sem;
} SClientInfo;

void* serverLoop(void* serv);

class CMessageServer{

        IPC::IPC  jockey_IPC;
   
	public:

		CMessageServer();
		~CMessageServer();
		int initServer(const char* port);
		//void update(double odo[],bool buttons[],int rotation[],int irr[]);
		CMessage getMessage();
      void sendMessage(CMessage &msg) {
         jockey_IPC.SendData(msg.type, (uint8_t*)msg.data, msg.len);
      }
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
		int messageRead;
};

#endif

/* end of CMessageServer.h */
