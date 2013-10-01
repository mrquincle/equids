/*
 * File name: CEquids.h
 * Date:      2006/10/12 11:56
 * Author:    Petr Stepan, CTU from Prague
 */

#ifndef __CEQUIDS_H__
#define __CEQUIDS_H__

#include "CJockey.h"
#include <stdio.h>
#include <vector>

#include <CMessage.h>

#define MAX_JOCKEYS 20


class CEquids
{
	CJockey jockeys[MAX_JOCKEYS];
	int analyze(char *line, FILE *fp);
	bool start();
	int num_jockeys;
	int runningJockey;
	int message;
public:
	CEquids();
	~CEquids();
	CJockey* getRunningJockey(){return &this->jockeys[this->runningJockey];};

	//! Return vocabs (identifiers not indices) of all running jockeys, also the permanent ones
	void getAllRunningJockeys(std::vector<vocab_t> &jockeyIds);
	CJockey* getJockey(int jockeyNumber){return &this->jockeys[jockeyNumber];};
	bool init(const char *filename);
	// permanently means that jockey do not stores its id to running_jockey parameter
	void initJockey(int j,bool permanently=false);
	void switchToJockey(int j);
	void sendMessage(int jockey, CMessage &m);
	void sendMessage(int jockey, int type, void *data, int len);
	void sendMessageToALL(int type, void *data, int len);
	CMessage getMessage(int j);
	int getNum_jockeys(){return num_jockeys;};

	//! Find now uses a unique identifier "vocab_id" that you have to set in CMessage.h
	//! Done on purpose here, although it's ugly, so you wan't forget
	int find(const char *name, vocab_t vocab_id);
	void quit();
};

#endif
/* end of CEquids.h */
