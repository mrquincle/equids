/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Back and forth controller
 * @file backandforth.cpp
 *
 * This file is created at Almende B.V. and Distributed Organisms B.V. It is open-source software and belongs to a
 * larger suite of software that is meant for research on self-organization principles and multi-agent systems where
 * learning algorithms are an important aspect.
 *
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless, we personally strongly object
 * against this software being used for military purposes, factory farming, animal experimentation, and "Universal
 * Declaration of Human Rights" violations.
 *
 * Copyright (c) 2013 Anne C. van Rossum <anne@almende.org>
 *
 * @author    Anne C. van Rossum
 * @date      Jul 30, 2012
 * @project   Replicator
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Testing
 */

#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <signal.h>

/***********************************************************************************************************************
 * Middleware includes
 **********************************************************************************************************************/

#include <IRobot.h>

//#include <comm/IRComm.h>

/***********************************************************************************************************************
 * Jockey framework includes
 **********************************************************************************************************************/

#include "../eth/CMessage.h"
#include "../eth/CMessageServer.h"
#include "../motor/CMotors.h"

/***********************************************************************************************************************
 * Implementation
 **********************************************************************************************************************/

//! The name of the controller can be used for controller selection
std::string NAME = "BackAndForth";

#define DEBUG \
NAME << '[' << getpid() << "] " << __func__ << "(): "

/**
 * If the user presses Ctrl+C, this can be used to do memory deallocation or a last communication with the MSPs.
 */
void interrupt_signal_handler(int signal) {
   if (signal == SIGINT) {
      //RobotBase::MSPReset();
      exit(0);
   }
}

/**
 * Basically only turns on and off the laser for a couple of times.
 */
int main(int argc, char **argv) {
   int speed;
   std::string port;
   struct sigaction a;
   

   std::cout << DEBUG << "Started with params " << argv[1] << ", " << argv[2] << std::endl;
   
   a.sa_handler = &interrupt_signal_handler;
   sigaction(SIGINT, &a, NULL);
   
   RobotBase* robot = RobotBase::Instance();
   RobotBase::RobotType robot_type = RobotBase::Initialize(NAME);

   for (int i = 0; i < 4; ++i)
      robot->SetPrintEnabled(i, false);
   
   if (argc > 2) {
      port = std::string(argv[1]);
      if (sscanf(argv[2], "%i", &speed)!=1) {
          speed = 50;
          std::cout << DEBUG << "Second parametr must be speed. Used standard speed 50 against" << argv[2] << std::endl;
      }
      std::cout << DEBUG << "Started with OK params " << argv[1] << ", " << argv[2] << std::endl;
   } else {
      std::cout << DEBUG << "Usage: port_number speed" << std::endl;
      return 1;
   }
   
   //	IRobotFactory factory;
   //	RobotBase* robot = factory.GetRobot();
   //	RobotBase::RobotType robot_type = factory.GetType();
   
   switch(robot_type) {
      case RobotBase::UNKNOWN: std::cout << "Detected unknown robot" << std::endl; break;
      case RobotBase::KABOT: std::cout << "Detected Karlsruhe robot" << std::endl; break;
      case RobotBase::ACTIVEWHEEL: std::cout << "Detected Active Wheel robot" << std::endl; break;
      case RobotBase::SCOUTBOT: std::cout << "Detected Scout robot" << std::endl; break;
      default:
         std::cout << DEBUG << "No known type (even not unknown). Did initialization go well?" << std::endl;
   }
   
   
   std::cout << "Create motor object" << std::endl;
   CMotors motors(robot, robot_type);

   std::cout << "Create (receiving) message server on port " << port << std::endl;
   CMessageServer *server;
   server = new CMessageServer();
   server->initServer(port.c_str());
   
   bool stopRobot = false;
   CMessage message;
   message.type = MSG_NONE;
   while (!stopRobot){
      message = server->getMessage();
      
      if (message.type != MSG_NONE)
         std::cout << "Command: " << message.getStrType() << ' ' << message.value1 << ',' \
         << message.value2 << std::endl;
      
      switch (message.type){
         case MSG_INIT:
            // you can initialize your controller
            usleep(10000);
            server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
            break;
         case MSG_START: {
            robot->pauseSPI(false);
            while (robot->isSPIPaused()) {
               usleep(10000);
            }
            std::cout << argv[2] << "Started SPI communication for controller on IPC" << std::endl;
            motors.setSpeeds(speed, -speed);
            server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
            break;
         }
         case MSG_STOP: {
            motors.setSpeeds(0,0);
            usleep(10000);
            robot->pauseSPI(true);
            while (!robot->isSPIPaused()){
               usleep(10000);
            }
            std::cout << argv[2] << "Paused SPI communication for controller on IPC" << std::endl;
            server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
            break;
         }
         case MSG_QUIT: {
            std::cout << "QUIT controller" << std::endl;
            motors.setSpeeds(0,0);
            usleep(10000);
            robot->pauseSPI(true);
            stopRobot = true;
            break;
         }
         case MSG_NONE: {
            break;
         }
         default: {
            std::cerr << DEBUG << "Did not understand message " << message.type << std::endl;
            break;
         }
      }
      
      if (message.type == MSG_NONE) {
         usleep(20000); // check every 0.02 second or do something usefull
      }
      message.type = MSG_NONE;
   }
   
   std::cout << "Stopping" << std::endl;
   return 0;
}


