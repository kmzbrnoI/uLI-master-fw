/*
 * Main header file.
 * (c) Jan Horacek 2016
 * Version: 1.0
 */

#ifndef MAIN_H
#define	MAIN_H

#include "GenericTypeDefs.h"

#define VERSION_HW		0x40
#define VERSION_SW		0x10

#define ROUND_MAX          5
#define ROUND_RACK         4    // 4. round is round when RACK is sent to active devices

#undef DEBUG
#define RACK_ENABLE

// device currently being adressed by master
typedef struct {
	BYTE index;				// requested device index
	BYTE timeout;			// after timeout is too big, next device is picked
	BOOL reacted;			// if the device has reacted to normal inquiry
    BYTE round;             // current round
} current;

#define PORT_TIMEOUT	10	 // 100 ms to consider port changed (yes, really, its power)

typedef struct {
	BYTE timeout;
	BOOL state;
} port_history;

typedef struct {
	BOOL status :1;
    BOOL active_devices :1;
} master_waiting;

#endif /* MAIN_H */

