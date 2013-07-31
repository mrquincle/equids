/*
 * File name: CMessage.h
 * Date:      2006/10/12 11:56
 * Author:    
 */

#ifndef __CMESSAGE_H__
#define __CMESSAGE_H__

#include "ethlolmsg.h"

#define MESSAGE_LENGTH  

typedef enum
{
	MSG_NONE = 0,
	MSG_START,
	MSG_STOP,
	MSG_RESET,
	MSG_QUIT,
   MSG_ACKNOWLEDGE,
   MSG_INIT,
	MSG_SPEED,
	MSG_HINGE,
	MSG_POS,
	MSG_NUMBER 
} TMessageType;

class CMessage
{
	public:
		CMessage();
		~CMessage();
		unsigned char buf[];
		void pack();
		void unpack();
        void set(const ELolMessage*msg);
		const char* getStrType();
		TMessageType type;
        int len;
      const uint8_t *data;
		int value1;
		int value2;
		int value3;
};

#endif

/* end of CMessage.h */
