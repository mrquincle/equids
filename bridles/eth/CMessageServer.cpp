#include "CMessageServer.h"

bool debug = false;

CMessageServer::CMessageServer() {
	sem_init(&dataSem, 0, 1);
	sem_init(&connectSem, 0, 1);
	last_ptr = 0;
   mm.type = MSG_NONE;
   mm.data = NULL;
}

CMessageServer::~CMessageServer() {
}

static void addMessage(const ELolMessage *msg, void * connection, void * serv) {
	CMessageServer* server = (CMessageServer*) serv;

	if (server != NULL) {
		sem_wait(&server->dataSem);
		bool found = false;
		for (int var = 0; var < server->lastMessages.size(); ++var) {
			if (server->lastMessages[var]->type
					== (TMessageType) msg->command) {
				if (server->lastMessages[var]->len != msg->length) {
               delete server->lastMessages[var];
					server->lastMessages.erase(
							server->lastMessages.begin() + var);
					CMessage* message = new CMessage();
					message->set(msg);
               message->valid = true;
					server->lastMessages.push_back(message);
				} else {
               if (msg->length>0) {
					   memcpy(server->lastMessages[var]->data, msg->data,
							msg->length);
               }
               server->lastMessages[var]->valid = true;
				}
				found = true;
				break;
			}
		}
		if (!found) {
			CMessage* message = new CMessage();
			message->set(msg);
			message->valid = true;
			server->lastMessages.push_back(message);
		}
		sem_post(&server->dataSem);
	} else {
      fprintf(stderr, "NULL server ERROR\n");
   }
}

int CMessageServer::initServer(const char* port) {

	jockey_IPC.SetCallback(addMessage, this);

	jockey_IPC.Name("jockey IPC");

	jockey_IPC.Start("localhost", atoi(port), true);

	return 0;
}

const CMessage & CMessageServer::getMessage() {
   int save_ptr;
	
   sem_wait(&dataSem);
   if (mm.data!=NULL) {
      delete[] mm.data;
      mm.data = NULL;
   }
   if (last_ptr>=lastMessages.size()) {
      last_ptr=0;
   }
   save_ptr = last_ptr;
    //  fprintf(stderr, "testing message ptr %i save ptr %i lastmessages size %i\n", last_ptr, save_ptr, lastMessages.size());
	while (last_ptr< lastMessages.size() && !lastMessages[last_ptr]->valid) {
		last_ptr++;
		if (last_ptr>=lastMessages.size()) {
			last_ptr=0;
		}
		if (save_ptr==last_ptr) {
			mm.type = MSG_NONE;
         last_ptr = lastMessages.size();
         break;
		}
	}
	if (last_ptr< lastMessages.size()) { 
	   mm.set(lastMessages[last_ptr]);
      lastMessages[last_ptr]->valid = false;
	   last_ptr++;
	   if (last_ptr>=lastMessages.size()) {
	      last_ptr=0;
 	   }
   } else {
      mm.type = MSG_NONE;
   }
	sem_post(&dataSem);
	return mm;
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

int CMessageServer::closeConnection() {
	jockey_IPC.Stop();
	connected = false;
	return 0;
}

