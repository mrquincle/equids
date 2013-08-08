#include "CJockey.h"
#include "CEquids.h"

CJockey::CJockey() {
	pid = -1;
	port_num = -1;
	sprintf(name, "Not defined");
	argv[0] = NULL;
	started = false;
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
	messageRead = 0;
	if (msg->command == MSG_QUIT) {
		quit();
	} else if (msg->command == MSG_ACKNOWLEDGE) {
		acknowledge = 1;
	} else if (redirection(msg)) {
//redirection already done inside redirection(msg)
	} else {
		last_length = msg->length;
		last_msg = msg->command;
		memcpy(last_data, msg->data, (last_length>MAX_DATA) ? MAX_DATA : last_length);
		last_ptr = msg->data;
      message.set(msg);
	}
	return true;
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

void CJockey::stop(bool wait_acknow){
	   	jockey_IPC.SendData(MSG_STOP, NULL, 0);
	   	if(wait_acknow){
	   	 while (this->acknowledge==0) {
	   	            usleep(10000);
	   	         }
	   	}
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

