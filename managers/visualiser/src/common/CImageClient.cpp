#include "CImageClient.h"

//#define NETWORK_BLOCK MSG_DONTWAIT
#define NETWORK_BLOCK 0 

CImageClient::CImageClient()
{
	totalReceived = 0;
}


CImageClient::~CImageClient()
{}

int CImageClient::connectServer(const char * ip,const char* port)
{
  int result = -1;
  socketNumber = socket(AF_INET, SOCK_STREAM, 0);
  if (socketNumber <0 )
  {
    return -1;
  }
  struct sockaddr_in server_addr;
  struct hostent *host_info;
  host_info =  gethostbyname(ip);
  if (host_info != NULL)
  {
    server_addr.sin_family = host_info->h_addrtype;
    memcpy((char *) &server_addr.sin_addr.s_addr,
           host_info->h_addr_list[0], host_info->h_length);
    server_addr.sin_port = htons(atoi(port));
    fprintf(stdout,"Connecting to %s:%s \n",ip,port);
    result = connect(socketNumber,(struct sockaddr*) &server_addr,sizeof(server_addr));
    if (result == 0)
    {
      fprintf(stderr,"Connection established.\n");
    }
    else
    {
      fprintf(stderr,"Connect error is %s \n",strerror(errno));
    }
  }
  return result;
}

int CImageClient::sendSmallMessage(unsigned char i)
{
	unsigned char b = i;
	int result = -1;
	int len =  send(socketNumber,&b,1,0);
	if (len == 1){
		result = 0;
	}else{
		fprintf(stderr,"Send problem \n",strerror(errno));
	}
	return result;
}



int CImageClient::sendMessage(CMessage message)
{
  int result = -1;
  message.pack();
  int len =  send(socketNumber,&message.buf,MESSAGE_LENGTH,0);
  if (len == MESSAGE_LENGTH){
	 result = 0;
//	 fprintf(stdout,"Send length %i. \n",len);
  }
  return result;
}

int CImageClient::checkForImage(CRawImage* image)
{
  char data[image->size];
  int result = 1;

  int lengthReceived = 0;
  int errcount = 0;
  while (errcount < 5 && totalReceived != image->size){
	  lengthReceived = recv(socketNumber,&data[totalReceived],image->size-totalReceived,NETWORK_BLOCK); 
	  if (lengthReceived < 0){
		  usleep(10000);
		  errcount++;
	  }else{
		  totalReceived += lengthReceived;
	  }
  }
  fprintf(stderr,"Received %i %i\n",totalReceived,image->size);
  if (totalReceived==image->size){
	  memcpy(image->data,data,image->size);
	  totalReceived = 0;
  }
  return result;
}

int CImageClient::disconnectServer()
{
  return close(socketNumber);
}
