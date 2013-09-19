#include "CMessageServer.h"

#ifdef CVUT_DEBUG
#include "wapi/wapi.h"
#endif

bool debug = false;

CMessageServer::CMessageServer() {
	sem_init(&dataSem, 0, 1);
	sem_init(&connectSem, 0, 1);
	last_ptr = 0;
	mm.type = MSG_NONE;
	mm.data = NULL;
#ifdef CVUT_DEBUG
	this->ip = ips;
#endif
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

int CMessageServer::closeConnection() {
	jockey_IPC.Stop();
	connected = false;
	return 0;
}

