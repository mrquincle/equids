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

void* connectLoop(void *serv)
{
	struct sockaddr_in clientAddr;
	socklen_t addrLen = sizeof(clientAddr);
	CMessageServer* server = (CMessageServer*) serv;
	int newServer = 0;
	while (true)
	{
		newServer = accept(server->serverSocket, (struct sockaddr *)&clientAddr,&addrLen);
		if (newServer > -1){
			if (debug) fprintf(stdout,"Incoming connection from %s.",inet_ntoa(clientAddr.sin_addr));
			if (debug) fprintf(stdout,"Incoming connection accepted on socket level %i.",newServer);
			sem_wait(&server->connectSem);
			server->mySocket = newServer;
			sem_post(&server->connectSem);
			pthread_t* thread=(pthread_t*)malloc(sizeof(pthread_t));
			pthread_create(thread,NULL,&serverLoop,(void*)server);
		}else{
			if (debug) fprintf(stdout,"Accept on listening socked failed.");
		}
	}
	return NULL;
}

int CMessageServer::initServer(const char* port)
{
	int used_port = atoi(port);
	struct sockaddr_in mySocketAddr;
	mySocketAddr.sin_family = AF_INET;
	mySocketAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	mySocketAddr.sin_port = htons(used_port);
	serverSocket = socket(AF_INET, SOCK_STREAM, 0);
	if (serverSocket < 0)
	{
		if (debug) fprintf(stdout,"Cannot create socket ");
		return -1;
	}
	if (bind(serverSocket,( struct sockaddr *)&mySocketAddr,sizeof(mySocketAddr)) < 0)
	{
		if (debug) fprintf(stdout,"Cannot bind socket.");
		return -2;
	}
	if (listen(serverSocket,1) < 0)
	{
		if (debug) fprintf(stdout,"cannot make socket listen.");
	}
	pthread_t* thread=(pthread_t*)malloc(sizeof(pthread_t));
	pthread_create(thread,NULL,&connectLoop,(void*)this);
	return 0;
}

void* serverLoop(void* serv)
{
	SClientInfo info;
	CMessage message; 
	CMessageServer* server = (CMessageServer*) serv;
	sem_wait(&server->connectSem);
	info.socket = server->mySocket;
	info.sem = &server->dataSem;
	sem_post(&server->connectSem);
	server->getClientInfo(info.socket,&info.write);
	int dataOk = 0;
	bool connected = true;
	while (connected){
		dataOk = 0;
		message = server->checkForMessage(info.socket);
		if (debug) fprintf(stdout,"Message received %s %i %i %i,%i,%i,%i from %i.",message.getStrType(),message.value1,message.value2,info.socket,message.buf[0],message.buf[1],message.buf[2],message.buf[3]);
		sem_wait(info.sem);
		if (info.odometry) server->sendDoubles(info.socket,server->odometry,10);
		if (info.buttons) server->sendBools(info.socket,server->buttons,10);
		if (info.rotation) server->sendInts(info.socket,server->rotation,1);
		if (info.ir) server->sendInts(info.socket,server->ir,4);
		sem_post(info.sem);
		if (info.write){
			if (debug) fprintf(stdout,"Sending message to main server.");
			sem_wait(info.sem);
			server->message = message;
			server->messageRead = 0; 
			sem_post(info.sem);
		}
		if (message.type == MSG_QUIT) connected = false;
	}
	server->closeConnection(info.socket);
	return NULL;
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

CMessage CMessageServer::checkForMessage(int socket)
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

void CMessageServer::update(double posit[],bool butto[],int rotat[],int irr[]) 
{
	sem_wait(&dataSem);
	memcpy(odometry,posit,10*sizeof(double));	
	memcpy(buttons,butto,10*sizeof(bool));
	memcpy(rotation,rotat,sizeof(int));
	memcpy(ir,irr,4*sizeof(int));
	sem_post(&dataSem);
}

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

int CMessageServer::closeConnection(int socket)
{
	close(socket);
	connected = false;
	return 0;
}

