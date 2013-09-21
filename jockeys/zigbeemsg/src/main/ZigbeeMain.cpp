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

#include "wapi/wapi.h"
#include "CMessageServer.h"
#define WAIT_TO_IDENTITY_TIMEOUT_S 1
#define RECEIVE_MESSAGE_TIMEOUT_MS 5000
#define ZIGBEE_CHANNEL 60

using namespace wapi;
CMessageServer *server;
static bool stopJockey = false;

/**
 * If the user presses Ctrl+C, this can be used to do memory deallocation or a last communication with the MSPs.
 */
void interrupt_signal_handler(int signal) {
   if (signal == SIGINT) {
      stopJockey=true;
   }
}


static Ubitag my_ubitag;

int zigbeeInit(WAPI *wapi)
{
		int wapi_error;
      int cnt;
		int channel=ZIGBEE_CHANNEL;
		
      wapi_error = wapi->join(channel);
		if(WAPI::WAPI_OK != wapi_error)	{
		   fprintf(stderr, "Cannot join to channel %i wapi_error %i\n", channel, wapi_error);
		   return 0;
		}
		printf("Joined to channel %i\n",channel);
		
      do	{
		   wapi_error = wapi->nodeInfo(my_ubitag);
		   if(WAPI::WAPI_OK != wapi_error)	{
			  fprintf(stderr, "Waiting for identity\n");
			  sleep(WAIT_TO_IDENTITY_TIMEOUT_S);

		   }
		} while(WAPI::WAPI_OK != wapi_error && cnt++<10);
	
	   return wapi_error==WAPI::WAPI_OK;
}

int zigbeeSend(WAPI *wapi, const uint8_t *data, int len) {
	int wapi_error;

	if (len > 0) {
		Message send_msg;
		Ubitag* ubi_brd;
		uint64_t ubi_destination;
		if (len > sizeof(uint64_t)) {
			memcpy(&ubi_destination, data, sizeof(uint64_t));

			ubi_brd = new Ubitag(ubi_destination);
			printf("sending message to: %lld\n", (long long)ubi_destination);
			send_msg.SetDestination(*ubi_brd);
			send_msg.SetHops(0);
			send_msg.SetSignalStrength(0);
			send_msg.SetChannel(ZIGBEE_CHANNEL);
			send_msg.SetData((const char*)data, len);
			wapi_error = wapi->send(send_msg);
		}
	}
	return wapi_error == WAPI::WAPI_OK;
}

int main(int argc, char **argv) {
   struct sigaction a;
   WAPI wapi(0);
   Message receive_msg;
   bool wapi_init;
   int wapi_error;

	fprintf(stdout, "Ubisence position starting %s\n", argv[1]);

	a.sa_handler = &interrupt_signal_handler;
	sigaction(SIGINT, &a, NULL);

	if (argc <= 1) {
		fprintf(stderr, "Usage: port_number\n");
		return 1;
	}

	printf("Create receiving message server on port %s\n", argv[1]);

	server = new CMessageServer();
	server->initServer(argv[1]);

	fprintf(stderr, "Init server finished\n");

	wapi_init = zigbeeInit(&wapi);

	CMessage message;
	message.type = MSG_NONE;
	while (!stopJockey) {
		//fprintf(stderr,"Get message\n");
		message = server->getMessage();

		if (message.type != MSG_NONE) {
			printf("Command: %s\n", message.getStrType());
		}

		switch (message.type) {
		case MSG_INIT:{
			// init WAPI
			server->sendMessage(MSG_ACKNOWLEDGE, NULL, 0);
		};
			break;

		case MSG_START: {
			break;
		}
		case MSG_STOP: {
			break;
		}
		case MSG_QUIT: {
			stopJockey = true;
			break;
		}
		case MSG_NONE: {
			break;
		}
		case MSG_ZIGBEE_MSG: {
			if (!wapi_init) {
				wapi_init = zigbeeInit(&wapi);
			}
			if (wapi_init) {

				CMessage unpacked = CMessage::unpackZBMessage(message);
				printf("try to send message %d over zigbee with size %d\n", unpacked.type,message.len);

				if (!zigbeeSend(&wapi, message.data, message.len)) {
					fprintf(stderr, "WAPI error sending message len %i\n",
							message.len);
				}
			}

		};break;
		default: {
			fprintf(stderr, "Did not understand message %i\n", message.type);
			break;
		}
		}

		if (!wapi_init) {
			wapi_init = zigbeeInit(&wapi);
		}
		if (wapi_init) {
			wapi_error = wapi.receive(receive_msg, 1);
			if (WAPI::WAPI_OK == wapi_error) {
				server->sendMessage(MSG_ZIGBEE_MSG, (void*) receive_msg.Data(),
						receive_msg.Size());
			}
		}

		if (message.type == MSG_NONE && wapi_error != WAPI::WAPI_OK) {
			usleep(50000); // check every 0.02 second or do something usefull
		}
	}

	printf("Stopping Zigbee Messenger\n");
	return 0;
}

