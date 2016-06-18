/*
 * Main header file.
 * (c) Jan Horacek 2016
 * Version: 1.0
 */

#ifndef MAIN_H
#define	MAIN_H

#include "GenericTypeDefs.h"

// device currently being adressed by master
typedef struct {
    BYTE index;             // requested device index
    BYTE timeout;           // after timeout is too big, next device is picked
    BOOL reacted;           // if the device has reacted to normal inquiry
} current;

#endif /* MAIN_H */

