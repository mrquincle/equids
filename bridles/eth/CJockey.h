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

class CJockey
{
public:
   IPC::IPC jockey_IPC;
   char name[MAX_NAME_LEN];
   char *argv[MAX_JOCKEYS_PARAMS+1];
   int port_num;
   int pid;
   int acknowledge;
   int last_type;
   unsigned char last_data[MAX_DATA];
   const unsigned char *last_ptr;
   CMessage last_msg;

   CJockey();
   ~CJockey();
   bool init(int my_pid);
   bool addMessage(const ELolMessage *msg);
   void SendMessage(CMessage &msg) {
      acknowledge = 0;
      jockey_IPC.SendData(msg.type, (uint8_t*)msg.data, msg.len);
   }
   void SendMessage(int type, void *data, int len) {
      acknowledge = 0;
      jockey_IPC.SendData(type, (uint8_t*)data, len);
   }
   CMessage *getMessage();
   void quit();
};

#endif

/* end of CJockey.h */
