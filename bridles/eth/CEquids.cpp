#include "CEquids.h"
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

#define MAX_BUFFER 1024

#define NAME "ActionSelection"

#define DEBUG NAME << '[' << getpid() << "] " << __func__ << "(): "

CEquids::CEquids() {
	num_jockeys = 0;
	runningJockey = -1;
}

CEquids::~CEquids() {
	//   quit();
}

/**
 * Read the jockeys from file.
 */
int CEquids::analyze(char *buf, FILE *fd) {
	int ret = 1;
	int p=0;
	char *tmp;
	char port_str[10];
	CJockey *j = &jockeys[num_jockeys];

	if ((tmp=strchr(buf, '#'))!=NULL) {
		*tmp = 0;
	}
	tmp = strtok(buf, " ,");
	while (tmp!=NULL && p<(MAX_JOCKEYS_PARAMS-1)) {
		if (p==0) {
			// first argument should be the full binary name
			strncpy(j->name, tmp, MAX_NAME_LEN);
			p++;
		} else if (p==1) {
			// second argument is the port, will always be added
			j->argv[0] = strdup(tmp);
			j->port_num = 50000+num_jockeys;
			sprintf(port_str,"%i",j->port_num);
			j->argv[1] = strdup(port_str);
			p++;
		} else {
			// other arguments
			int ptr = strlen(tmp)-1;
			while (ptr>0 && (tmp[ptr]=='\n' || tmp[ptr]=='\r')) {
				ptr--;
			}
			if (strlen(tmp)>0) {
				tmp[ptr+1]=0;
				j->argv[p] = strdup(tmp);
				printf("Parse argument for %s: %s\n", j->name, tmp);
				p++;
			}
		}
		tmp = strtok(NULL, " ,");
	}

	if (p==0) {
		ret = 0;
	} else if (p>1) {
		j->argv[p]=NULL;
		ret = 1;
	} else {
		ret = -1;
	}
	return ret;
} 

bool CEquids::start() {
	int i;
	int pid;
	bool error = false;
	char exe[128];

	for (int i=0; i<num_jockeys; i++) {
		pid = vfork();
		if (pid==0) {
			fprintf(stdout, "Starting process %s with %s ", jockeys[i].argv[0], jockeys[i].argv[1]);
			char *a; int j = 2;
			while ((a = jockeys[i].argv[j]) != NULL) {
				printf("%s ", a);
				j++;
			}
			printf("\n");
			if (execvp(jockeys[i].argv[0], jockeys[i].argv)<0) {
				sprintf(exe,"%s", jockeys[i].argv[0]);
				fprintf(stderr, "Cannot exec process %s\n", exe);
				error = true;
			}
			exit(1);
		} else if (pid>0) {
			jockeys[i].init(pid,this);
			jockeys[i].SendMessage(MSG_INIT, NULL, 0);
			while (jockeys[i].acknowledge==0) {
				usleep(10000);
			}
		} else {
			fprintf(stderr, "Cannot fork new process. Error %i\n",pid);
			error = true;
		}
	}
	return !error;
}

bool CEquids::init(const char *filename) {
	bool result = false;
	bool error = false;
	char buf[MAX_BUFFER];

	FILE *fp = fopen(filename, "r");
	if (fp) {
		while (fgets(buf, MAX_BUFFER, fp)!=NULL && !error && num_jockeys<(MAX_JOCKEYS-1)) {
			int a = analyze(buf, fp);
			if (a==1) {
				num_jockeys++;
			} else if (a<0) {
				error = true;
			}
		}
		fclose(fp);
		if (!error) {
			result = start();
		}
	}
	return result;
}
//permanently means that it is not stored in runnigng jockey
void CEquids::initJockey(int j, bool permanently) {
	usleep(500000);
	if(!jockeys[j].started){
		if (j>=0 && j<num_jockeys) {
			if(!permanently) {
				runningJockey = j;
			}
			std::cout << DEBUG << "Send MSG_START to jockey " << jockeys[j].name << std::endl;
			jockeys[j].SendMessage(MSG_START, NULL, 0);
			while (jockeys[j].acknowledge==0) {
				usleep(10000);
			}
			std::cout << DEBUG << "Got acknowledgment from jockey " << jockeys[j].name << " for MSG_START" << std::endl;
			jockeys[j].started = true;
		} else {
			fprintf(stderr, "Error! This jockey does not exist!\n");
		}
	}else{
		printf("Error! try to start already running jockey \n");
	}
	if (permanently) {
		std::cout << DEBUG << "Jockey " << jockeys[j].name << " will run permanently!" << std::endl;
	}
}

//! Returns identifiers, not indices!
void CEquids::getAllRunningJockeys(std::vector<vocab_t> &jockeyIds) {
	jockeyIds.clear();
	for (int i = 0; i < num_jockeys; ++i) {
		if (jockeys[i].started) jockeyIds.push_back(jockeys[i].vocab_id);
	}
}

void CEquids::switchToJockey(int j) {
	// do not switch if this is already the running jockey
	if (runningJockey == j) {
		// let us just not print anything, because if we can switch to the same jockey, the FSM becomes much smaller,
		// so it is a nice feature (and should not have a lot of debugging output).
//		printf("Jockey %s is already running, no need to switch\n", jockeys[j].name);
		return;
	} else {
		if (runningJockey>=0 && runningJockey<num_jockeys)
			std::cout << DEBUG << "Switch from jockey " << jockeys[runningJockey].name << " to jockey " << jockeys[j].name << std::endl;
	}

	if (runningJockey>=0 && runningJockey<num_jockeys) {
		std::cout << DEBUG << "Send MSG_STOP to jockey " << jockeys[runningJockey].name << std::endl;
		jockeys[runningJockey].SendMessage(MSG_STOP, NULL, 0);
		while (jockeys[runningJockey].acknowledge==0) {
			usleep(10000);
		}
		std::cout << DEBUG << "Got acknowledgment from jockey " << jockeys[runningJockey].name << " for MSG_STOP" << std::endl;
		jockeys[runningJockey].started = false;
	}
	if (j>=0 && j<num_jockeys) {
		runningJockey = j;
		std::cout << DEBUG << "Send MSG_START to jockey " << jockeys[j].name << std::endl;
		jockeys[j].SendMessage(MSG_START, NULL, 0);
		while (jockeys[j].acknowledge==0) {
			usleep(10000);
		}
		std::cout << DEBUG << "Got acknowledgment from jockey " << jockeys[j].name << " for MSG_START" << std::endl;
		jockeys[runningJockey].started = true;
	}
}

void CEquids::sendMessage(int jockey, CMessage &m) {
	if (jockey>=0 && jockey<num_jockeys) {
		jockeys[jockey].SendMessage(m);
	}
}

void CEquids::sendMessage(int jockey,int type, void *data, int len){
	if (jockey>=0 && jockey<num_jockeys) {
		jockeys[jockey].SendMessage(type, data, len);
	}
}

CMessage CEquids::getMessage(int j) {
	CMessage message;
	if (j >= 0 && j < num_jockeys) {
		message = jockeys[j].getMessage();
	} else {
		message.type = MSG_NONE;
		message.valid = false;

	}
	return message;
}

void CEquids::sendMessageToALL(int type, void *data, int len){
	for (int var = 0; var < num_jockeys; ++var) {
		jockeys[var].SendMessage(type, data, len);
	}
}

int CEquids::find(const char *name, vocab_t vocab_id) {
	int ret=-1;

	for (int i=0; i<num_jockeys && ret<0; i++) {
		if (strcmp(name, jockeys[i].name)==0) {
			jockeys[i].vocab_id = vocab_id;
			ret = i;
		}
	}
	return ret;
}

void CEquids::quit() {
	int ptr=0;
	int st=0;

	if (runningJockey>=0 && runningJockey<num_jockeys) {
		jockeys[runningJockey].SendMessage(MSG_STOP, NULL, 0);
		while (jockeys[runningJockey].acknowledge==0) {
			usleep(10000);
		}
	}
	for (int i=0; i<num_jockeys; i++) {
		jockeys[i].quit();
	}
	sleep(1);
	while(ptr<num_jockeys) {
		wait();
		fprintf(stdout, "finished process \n");
		ptr++;
	}
	num_jockeys=0;
}
