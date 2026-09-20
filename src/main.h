/*
 * Main header file.
 * (c) Jan Horacek 2016
 * Version: 1.0
 */

#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>

extern uint8_t version_hw;
#define VERSION_HW_OLD  0x42
#define VERSION_HW_5    0x50
#define VERSION_SW      0x15

#define ROUND_MAX       5
#define ROUND_RACK      4 // 4. round is round when RACK is sent to active devices

// device currently being adressed by master
typedef struct {
    uint8_t index;   // requested device index
    uint8_t timeout; // after timeout is too big, next device is picked
    bool reacted; // if the device has reacted to normal inquiry
    uint8_t round;   // current round
    bool finished;// whether the whole message was received
} current;

#define PORT_TIMEOUT 10 // 100 ms to consider port changed (yes, really, it is power)

typedef struct {
    uint8_t timeout;
    bool state;
} port_history;

typedef union {
    struct {
        bool status : 1;
        bool active_devices : 1;
        bool keep_alive : 1;
    } bits;
    uint8_t all;
} master_waiting;

#define KA_RECEIVE_MAX      500 // 5 s = keep-alive receive timeout
#define KA_SEND_INTERVAL    100 // 1 s = keep-alive packet sending

typedef struct {
    bool receive : 1; // disable bus after not receiving keep-alive packets
    bool send : 1;    // send keep-alive packets
    uint16_t receive_timer;
    uint8_t send_timer;
} alive;

#endif /* MAIN_H */
