/*
 * Main header file.
 * (c) Jan Horacek 2016
 * Version: 1.0
 */

#ifndef MAIN_H
#define MAIN_H

#include "GenericTypeDefs.h"

#define VERSION_HW      0x42
#define VERSION_SW      0x14

#define ROUND_MAX       5
#define ROUND_RACK      4 // 4. round is round when RACK is sent to active devices

// device currently being adressed by master
typedef struct {
	BYTE index;   // requested device index
	BYTE timeout; // after timeout is too big, next device is picked
	BOOL reacted; // if the device has reacted to normal inquiry
	BYTE round;   // current round
    BOOL finished;// whether the whole message was received
} current;

#define PORT_TIMEOUT 10 // 100 ms to consider port changed (yes, really, it is power)

typedef struct {
	BYTE timeout;
	BOOL state;
} port_history;

typedef union {
	struct {
		BOOL status : 1;
		BOOL active_devices : 1;
		BOOL keep_alive : 1;
	} bits;
	BYTE all;
} master_waiting;

#define KA_RECEIVE_MAX      500 // 5 s = keep-alive receive timeout
#define KA_SEND_INTERVAL    100 // 1 s = keep-alive packet sending

typedef struct {
	BOOL receive : 1; // disable bus after not receiving keep-alive packets
	BOOL send : 1;    // send keep-alive packets
	WORD receive_timer;
	BYTE send_timer;
} alive;

#endif /* MAIN_H */
