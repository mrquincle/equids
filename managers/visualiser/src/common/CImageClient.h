#ifndef CIMAGECLIENT_H
#define CIMAGECLIENT_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include "CRawImage.h"
#include "CMessage.h"
#include <unistd.h>
/**
@author Tom Krajnik
*/
class CImageClient
{
public:
    CImageClient();
    ~CImageClient();

    int sendMessage(CMessage message);
    int sendSmallMessage(unsigned char message);
    int connectServer(const char * ip,const char* port);
    int checkForImage(CRawImage* image);
    int disconnectServer();

private:
    int socketNumber;
    int totalReceived;
};

#endif
