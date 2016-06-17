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
    BYTE index;
    BYTE timeout;
} current;

// one XpressNET device
typedef struct {
    BOOL active;
}device ;

#endif /* MAIN_H */

