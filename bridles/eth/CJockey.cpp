#include "CJockey.h"


CJockey::CJockey() {
   pid=-1;
   port_num = -1;
   sprintf(name,"Not defined");
   argv[0]=NULL;
}

CJockey::~CJockey() {
   jockey_IPC.Stop();
}

static void getMessage(const ELolMessage *msg, void * connection, void * jock)
{
   CJockey* jockey = (CJockey *) jock;
   if (jockey!=NULL) {
      jockey->addMessage(msg);
   }
}

bool CJockey::addMessage(const ELolMessage *msg) {
   if (msg->command == MSG_QUIT) {
      quit();
   } else if (msg->command == MSG_ACKNOWLEDGE) {
      acknowledge = 1;
   } else {
      last_msg = msg->command;
      memcpy(last_data, msg->data, MAX_DATA);
      last_ptr = msg->data;
   }
   return true;
}

bool CJockey::init(int my_pid) {
   char name_IPC[120];
   pid = my_pid;
   jockey_IPC.SetCallback(getMessage, this);
   sprintf(name_IPC, "%s_IPC", name);
   jockey_IPC.Name(name_IPC);
   jockey_IPC.Start("localhost", port_num, false);
}

void CJockey::quit() {
   jockey_IPC.SendData(MSG_QUIT, NULL, 0);
}