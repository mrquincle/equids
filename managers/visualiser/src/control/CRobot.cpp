#include "CRobot.h"

CRobot::CRobot()
{
	initialized = false;
	lastOdometry = 0;
	commDelay = 20000;
}

CRobot::~CRobot()
{
}

bool CRobot::init(const char* portName)
{
	char cfgStr[1000];
	sprintf(cfgStr,"stty -F %s 19200 -parenb -parodd cs8 -hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff  -iuclc -ixany -imaxbel -opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0   ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop  -echoprt -echoctl -echoke",portName);
	system(cfgStr);			

	port = open(portName, O_RDWR|O_NOCTTY);
	if (port == -1){
		fprintf(stderr,"Cannot open port %s\n,",portName);
		fprintf(stderr,"Error initializing the robot\n");
	}
	initialized = (port != -1);
	return initialized; 
}
