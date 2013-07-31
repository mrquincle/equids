#include "CEquids.h"
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#define MAX_BUFFER 1024

CEquids::CEquids() {
   num_jockeys=0;
}

CEquids::~CEquids() {
   quit();
}

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
         strncpy(j->name, tmp, MAX_NAME_LEN);
      } else if (p==1) {
         j->argv[0] = strdup(tmp);
         j->port_num = 50000+num_jockeys;
         sprintf(port_str,"%i",j->port_num);
         j->argv[1] = strdup(port_str);
      } else {
         j->argv[p] =  strdup(tmp);
      }
      p++;
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
         //sprintf(exe,"/flash/%s", jockeys[i].argv[0]);
         fprintf(stdout, "Starting process %s with %s\n", jockeys[i].argv[0], jockeys[i].argv[1]);
         if (execvp(jockeys[i].argv[0], jockeys[i].argv)<0) {
            fprintf(stderr, "Cannot exec process %s\n", exe);
            error = true;
         }
         exit(1);
      } else if (pid>0) {
         jockeys[i].init(pid);
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
         int a =analyze(buf, fp);
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

void CEquids::initJockey(int j) {
   usleep(500000);
   if (j>=0 && j<num_jockeys) {
      runningJockey = j;
      jockeys[j].SendMessage(MSG_START, NULL, 0);
      while (jockeys[j].acknowledge==0) {
         usleep(10000);
      }
   }
}

void CEquids::switchToJockey(int j) {
   if (runningJockey>=0 && runningJockey<num_jockeys) {
      jockeys[runningJockey].SendMessage(MSG_STOP, NULL, 0);
      while (jockeys[runningJockey].acknowledge==0) {
         usleep(10000);
      }
   }
   if (j>=0 && j<num_jockeys) {
      runningJockey = j;
      jockeys[j].SendMessage(MSG_START, NULL, 0);
      while (jockeys[j].acknowledge==0) {
         usleep(10000);
      }
   }
}

void CEquids::sendMessage(int j, CMessage &m) {
   if (j>=0 && j<num_jockeys) {
      jockeys[j].SendMessage(m);
   }
}

int CEquids::find(const char *name) {
   int ret=-1;
   
   for (int i=0; i<num_jockeys && ret<0; i++) {
      if (strcmp(name, jockeys[i].name)==0) {
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
