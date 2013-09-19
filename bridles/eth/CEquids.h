/*
 * File name: CEquids.h
 * Date:      2006/10/12 11:56
 * Author:    Petr Stepan, CTU from Prague
 */

#ifndef __CEQUIDS_H__
#define __CEQUIDS_H__

#include "CJockey.h"
#include <stdio.h>
#define MAX_JOCKEYS 20


class CEquids
{
   CJockey jockeys[MAX_JOCKEYS];
   int analyze(char *line, FILE *fp);
   bool start(void);
   int num_jockeys;
   int runningJockey;
   int message;
public:
	CEquids();
	~CEquids();
	CJockey* getRunningJockey(){return &this->jockeys[this->runningJockey];};
	CJockey* getJockey(int jockeyNumber){return &this->jockeys[jockeyNumber];};
	bool init(const char *filename);
	void initJockey(int j);
	void switchToJockey(int j);
	void sendMessage(int jockey, CMessage &m);
	void sendMessage(int jockey, int type, void *data, int len);
	void sendMessageToALL(int type, void *data, int len);
	CMessage getMessage(int j);

   int  find(const char *name);
   void quit();
};

#endif
/* end of CEquids.h */
