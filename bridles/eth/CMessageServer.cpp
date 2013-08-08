#include "CMessageServer.h"

bool debug = false;

CMessageServer::CMessageServer()
{
	sem_init(&dataSem,0,1);	
	sem_init(&connectSem,0,1);	
	messageRead = 0;
}

CMessageServer::~CMessageServer()
{
}

static void addMessage(const ELolMessage *msg, void * connection, void * serv)
{
   CMessageServer* server = (CMessageServer*) serv;
   if (server!=NULL) {
      sem_wait(&server->dataSem);
      server->message.set(msg);
      server->messageRead = 0;
      sem_post(&server->dataSem);
   }
}


int CMessageServer::initServer(const char* port)
{

   jockey_IPC.SetCallback(addMessage, this);

   jockey_IPC.Name("jockey IPC");

   jockey_IPC.Start("localhost", atoi(port), true);

   return 0;
}


CMessage CMessageServer::getMessage()
{
	CMessage result;

	sem_wait(&dataSem);
	result = message;

	messageRead++; 
	message.type = MSG_NONE;

	sem_post(&dataSem);
	return result;
}

/*
bool CMessageServer::getClientInfo(int socket,bool data[])
{
	bool result;
	char buffer[1000];
	int receiveResult = recv(socket,data,5*sizeof(bool),MSG_WAITALL);
	if ( receiveResult >0 && connected == 1)
	{
		sprintf(buffer,"Client requires");
		if (data[0]) sprintf(buffer,"%s to drive the robot and obtain ",buffer); else sprintf(buffer,"to read ");
		if (data[1]) sprintf(buffer,"%s odometric, ",buffer);
		if (data[2]) sprintf(buffer,"%s rotation speed ",buffer);
		if (data[3]) sprintf(buffer,"%s pressed buttons ",buffer);
		if (data[4]) sprintf(buffer,"%s ir measurements ",buffer);
		sprintf(buffer,"%sdata.",buffer);
		if (debug) fprintf(stdout,"%s",buffer);
		result = true;
	}
	else
	{
		if (debug) fprintf(stdout,"Disconnect detected.");
		result = false;
	}
	return result;
}
*/

/*
CMessage CMessageServer::checkForMessage()
{
	CMessage message;
	int receiveResult = recv(socket,&message.buf,MESSAGE_LENGTH,MSG_WAITALL);
	if ( receiveResult >0)
	{
		if (debug) fprintf(stdout,"Packet accepted length %i.", receiveResult);
		message.unpack();
	}
	else
	{
		if (debug) fprintf(stdout,"Disconnect detected.");
		message.type = MSG_QUIT;
	}
	return message;
}
*/

/*
void CMessageServer::update(double posit[],bool butto[],int rotat[],int irr[])
{
	sem_wait(&dataSem);
	memcpy(odometry,posit,10*sizeof(double));	
	memcpy(buttons,butto,10*sizeof(bool));
	memcpy(rotation,rotat,sizeof(int));
	memcpy(ir,irr,4*sizeof(int));
	sem_post(&dataSem);
}
*/

/*
int CMessageServer::sendPosition(int socket,double buffer[])
{
	    if (send(socket,buffer,4*sizeof(double),MSG_NOSIGNAL) == 4*sizeof(double)) return 0; else
		 {
			 if (debug) fprintf(stdout,"Network error");
			 return -1;
		 }
		 return 0;
}

int CMessageServer::sendInts(int socket,int buffer[],int len)
{
	if (send(socket,buffer,len*sizeof(int),MSG_NOSIGNAL) == (int)(len*sizeof(int))) return 0; else
	{
		if (debug) fprintf(stdout,"Network error");
		return -1;
	}
	return 0;
}


int CMessageServer::sendDoubles(int socket,double buffer[],int len)
{
	int a =	send(socket,buffer,len*sizeof(double),MSG_NOSIGNAL);
	if (a == (int)(len*sizeof(double)))  return 0; else
	{
		if (debug) fprintf(stdout,"Network error, %i ",a);
		return -1;
	}
	return 0;
}

int CMessageServer::sendBools(int socket,bool buffer[],int len)
{
	    if (send(socket,buffer,len*sizeof(bool),MSG_NOSIGNAL) == (int)(len*sizeof(bool))) return 0; else
		 {
			 if (debug) fprintf(stdout,"Network error");
			 return -1;
		 }
		 return 0;
}
*/

int CMessageServer::closeConnection()
{
	jockey_IPC.Stop();
	connected = false;
	return 0;
}

