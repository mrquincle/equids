/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Timing of functions
 * @file CTimer.h
 *
 * This file is created at Czech Technical Unversity, Prague.
 *
 * Copyright (c) 2013 Tom Krajnik
 *
 * @author      Tom Krajnik
 * @date        Jan 18, 2011
 * @project     Replicator
 * @university  Czech Technical Unversity, Prague.
 * @case        Navigation
 */

#ifndef CTIMER_H
#define CTIMER_H

#include <sys/time.h>
#include <stdlib.h>

#define TIMEOUT_INTERVAL 40000

class CTimer
{
	public:
		CTimer(int timeOut = TIMEOUT_INTERVAL);
		~CTimer();

		void reset(int timeOut = TIMEOUT_INTERVAL);
		bool paused();

		int pause();
		int start();
		int getTime();
		bool timeOut();

		// Set frequency of tick
		void setFreq(int freq);

		// Set period (use this or setFreq) if you want to use tick()
		void setPeriod(int period);

		// This function can be used to obtain a frequency (calls usleep)
		void tick();
	private:
		int getRealTime();

		int startTime;
		int pauseTime;
		bool running;
		int timeoutInterval;
		int period;
		int lastTick;
};

#endif
