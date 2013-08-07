/*
 * File name: CImageMessage.h
 * Date:      2006/10/12 11:56
 * Author:    
 */

#ifndef __CIMMESSAGE_H__
#define __CIMMESSAGE_H__

#define IMAGE_MESSAGE_LENGTH 7

typedef enum
{
	MSGI_NONE = 0,
	MSGI_GET_IMAGE,
	MSGI_LEARN,
	MSGI_RESET,
	MSGI_SAVE,
	MSGI_PAUSE,
	MSGI_RESUME,
	MSGI_STEP,
	MSGI_NUMBER 
} TImageMessageType;

class CImageMessage
{
	public:
		CImageMessage();
		~CImageMessage();
		unsigned char buf[IMAGE_MESSAGE_LENGTH+1];
		void pack();
		void unpack();
		const char* getStrType();
		TImageMessageType type;
		int value1;
		int value2;
};

#endif

/* end of CImageMessage.h */
