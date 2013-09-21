#include "CJockey.h"
#include "CEquids.h"

CJockey::CJockey() {
	pid = -1;
	port_num = -1;
	sprintf(name, "Not defined");
	argv[0] = NULL;
	started = false;
	sem_init(&messSem, 0, 1);
}

CJockey::~CJockey() {
	jockey_IPC.Stop();
}

static void getMessageCallback(const ELolMessage *msg, void * connection, void * jock) {
	CJockey* jockey = (CJockey *) jock;
	if (jockey != NULL) {
		jockey->addMessage(msg);
	}
}

bool CJockey::addMessage(const ELolMessage *msg) {
	if (msg->command == MSG_QUIT) {
		quit();
	} else if (msg->command == MSG_ACKNOWLEDGE) {
		acknowledge = 1;
	} else if (redirection(msg)) {
//redirection already done inside redirection(msg)
	} else {

		sem_wait(&messSem);
		if (incomingMessages.size() > 50) {
			printf(
					"%s incoming message buffer overfull - delete 10 oldes messages\n",name);
			incomingMessages.erase(incomingMessages.begin(),
					incomingMessages.begin() + 10);
		}
		CMessage message;
		message.set(msg);
		incomingMessages.push_back(message);
		sem_post(&messSem);
	}
	return true;
}

CMessage CJockey::getMessage() {
	CMessage message;
	sem_wait(&messSem);
	//printf("num incoming messages %d \n",incomingMessages.size());
	//usleep(1000000);
	if (incomingMessages.size() > 0) {
		//if(incomingMessages[0].type==MSG_)
		message = incomingMessages[0];
		message.len=incomingMessages[0].len;
		uint8_t* newdata = new uint8_t[message.len];
		memcpy(newdata,incomingMessages[0].data,message.len);
		message.data = newdata;
		incomingMessages.erase(incomingMessages.begin());
	} else {
		message.type = MSG_NONE;
		message.valid = false;
		message.data = NULL;
		message.len = 0;
	}
	sem_post(&messSem);
	return message;
}

bool CJockey::init(int my_pid, CEquids *master) {
	char name_IPC[120];
	pid = my_pid;
	equids=master;
	jockey_IPC.SetCallback(getMessageCallback, this);
	sprintf(name_IPC, "%s_IPC", name);
	jockey_IPC.Name(name_IPC);
	jockey_IPC.Start("localhost", port_num, false);
}

void CJockey::quit() {
	jockey_IPC.SendData(MSG_QUIT, NULL, 0);
}

void CJockey::stop(bool wait_acknow) {
	jockey_IPC.SendData(MSG_STOP, NULL, 0);
	this->started = false;
	if (wait_acknow) {
		while (this->acknowledge == 0) {
			usleep(10000);
		}
	}
	this->redirectinTable.clear();
	this->incomingMessages.clear();
}

bool CJockey::redirection(const ELolMessage *msg) {
	bool isredirected=false;
	for (int var = 0; var < redirectinTable.size(); ++var) {
			if(redirectinTable[var].messagetype == msg->command){
				//printf("try redirecting %d to %d\n",msg->command,redirectinTable[var].toJockey);
				equids->getJockey(redirectinTable[var].toJockey)->SendMessage(msg->command,(void*) msg->data, msg->length);
				//printf("redirecting %d to %d\n",msg->command,redirectinTable[var].toJockey);
				isredirected=true;
				break;
			}
		}
	return isredirected;
}

void CJockey::addRedirection(int redirectTo, TMessageType redirectedMessT) {
	Redirection red = { redirectTo, redirectedMessT };
	redirectinTable.push_back(red);
}
void CJockey::removeRedirection(int redirectTo, TMessageType redirectedMessT) {
	for (int var = 0; var < redirectinTable.size(); ++var) {
		if(redirectinTable[var].toJockey == redirectTo && redirectinTable[var].messagetype == redirectedMessT){
			redirectinTable.erase (redirectinTable.begin()+var);
		}
	}
}

