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

#include "CTimer.h"

#include <stdio.h>
#include <unistd.h>

CTimer::CTimer(int timeout)
{
  reset();
  timeoutInterval = timeout;
  lastTick = getRealTime();
  period = 1000;
  pause();
}

CTimer::~CTimer()
{}

void CTimer::reset(int timeout)
{
  timeoutInterval = timeout;
  startTime = getRealTime();
  pauseTime = startTime;
}

//! Get current time in milliseconds
int CTimer::getRealTime()
{
  struct  timeval currentTime;
  gettimeofday(&currentTime, NULL);
  return currentTime.tv_sec*1000 + currentTime.tv_usec/1000;
}

int CTimer::getTime()
{
  int result;
  if (running)
  {
    result = getRealTime() - startTime;
  }
  else
  {
    result = pauseTime - startTime;
  }
  return result;
}

bool CTimer::timeOut()
{
  return getTime() > timeoutInterval;
}

bool CTimer::paused()
{
	return (running==false);
}

int CTimer::pause()
{
  running = false;
  return pauseTime = getRealTime();
}

int CTimer::start()
{
  startTime += (getRealTime() - pauseTime);
  running = true;

  return getTime();
}

/**
 * Frequency per second. Will be converted to period in ms. The accuracy is
 * not good, we just divide integers. So, don't use a frequency above say 100Hz.
 * Use setPeriod instead (although accuracy of tick() is not 100% anyway).
 */
void CTimer::setFreq(int freq) {
	period = 1000/freq;
}

void CTimer::setPeriod(int period) {
	if (period > 5000) {
		fprintf(stdout, "A period over 5000 is not supported to protect you....\n");
		this->period = 5000;
		return;
	}
	this->period = period;
}

/**
 * An inaccurate tick. It sleeps a while defined previously by setFreq.
 * There is a cap on sleeping for maximum 5 second.
 */
void CTimer::tick() {
	int timePassed = getRealTime() - lastTick;
	int timeToSleep = period - timePassed;
//	fprintf(stdout, "Time passed %i and time to sleep %i\n", timePassed, timeToSleep);
	if (timeToSleep > 5000) {
		sleep(5);
	} else if (timeToSleep > 0) {
		usleep(timeToSleep * 1000);
	}
	lastTick = getRealTime();
}
