/*
 * File name: CJockey.h
 * Date:      2006/10/12 11:56
 * Author:    Petr Stepan, CTU from Prague
 */

#ifndef __CJOCKEY_H__
#define __CJOCKEY_H__

#define MAX_JOCKEYS_PARAMS  10
#define MAX_NAME_LEN 50
#define MAX_DATA 128

#include "ipc.hh"
#include "CMessage.h"
#include <semaphore.h>
#include <vector>


class CEquids;
class CJockey
{

struct Redirection{
int toJockey;
TMessageType messagetype;
};

public:
	IPC::IPC jockey_IPC;
	char name[MAX_NAME_LEN];
	char *argv[MAX_JOCKEYS_PARAMS + 1];
	int port_num;
	int pid;
	int acknowledge;

	CJockey();
	~CJockey();
	bool init(int my_pid, CEquids* const);
	bool addMessage(const ELolMessage *msg);
	void SendMessage(CMessage &msg) {
		acknowledge = 0;
		jockey_IPC.SendData(msg.type, (uint8_t*) msg.data, msg.len);
	}
	void SendMessage(int type, void *data, int len) {
		acknowledge = 0;
		jockey_IPC.SendData(type, (uint8_t*) data, len);
	}
	void quit();
	void stop(bool wait_acknow);
	void addRedirection(int redirectTo, TMessageType redirectedMessT);
	void removeRedirection(int redirectTo, TMessageType redirectedMessT);
	bool started;
	CMessage getMessage();
private:
	CMessage message;
	CEquids* equids;
	sem_t messSem;
	bool redirection(const ELolMessage *msg);
	std::vector<Redirection> redirectinTable;
	std::vector<CMessage> incomingMessages;

};

#endif

/* end of CJockey.h */
