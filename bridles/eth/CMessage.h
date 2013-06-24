/*
 * File name: CMessage.h
 * Date:      2006/10/12 11:56
 * Author:    
 */

#ifndef __CMESSAGE_H__
#define __CMESSAGE_H__

#define MESSAGE_LENGTH 10 

typedef enum
{
	MSG_NONE = 0,
	MSG_START,
	MSG_STOP,
	MSG_RESET,
	MSG_QUIT,
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
		unsigned char buf[MESSAGE_LENGTH+1];
		void pack();
		void unpack();
		const char* getStrType();
		TMessageType type;
		int value1;
		int value2;
		int value3;
};

#endif

/* end of CMessage.h */
