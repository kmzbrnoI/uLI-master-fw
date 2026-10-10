/* uLI-mater main firmware file. */

/** INCLUDES ******************************************************************/

#include <xc.h>
#include <stdbool.h>
#include <inttypes.h>

#include "HardwareProfile.h"
#include "config.h"
#include "main.h"
#include "ringBuffer.h"
#include "usart.h"
#include "usb.h"
#include "usb_config.h"
#include "usb_device.h"
#include "usb_device_cdc.h"

/** DEFINES *******************************************************************/

#define msg_len(buf,start)      (((buf).data[((start) + 1) % RINGBUF_SIZE] & 0x0F)+3)

#define USB_last_message_len    ringDistance(&ring_USB_datain, last_start, ring_USB_datain.ptr_e)
#define USART_last_message_len  ringDistance(&ring_USART_datain, USART_last_start, ring_USART_datain.ptr_e)

#define IsRACKRound             (current_dev.round == ROUND_RACK)

#define USB_MAX_TIMEOUT         10 // 100 ms
#define USART_MAX_TIMEOUT       50 // 500 us

#define DEVICE_COUNT            32 // XpressNET device count
#define NI_TIMEOUT              12 // normal inquiery timeout = 120 us

#define MLED_IN_MAX_TIMEOUT      5 // 50 ms
#define MLED_OUT_MAX_TIMEOUT     5 // 50 ms

#define PWR_LED_SHORT_COUNT     15 // 150 ms
#define PWR_LED_LONG_COUNT      40 // 400 ms
#define PWR_LED_FERR_COUNT      10 // status led indicates >10 framing errors

#define TIMEOUT_ERR_TIMEOUT     20 // 200 ms

/** VARIABLES *****************************************************************/

uint8_t USB_Out_Buffer[32];
uint8_t version_hw;

// USB -> USART ring buffer
volatile ring_generic ring_USB_datain;
// USART -> USB ring buffer
volatile ring_generic ring_USART_datain;

// XpressNET device currently being requested
volatile current current_dev = { 0, 0, 0, 0 };

// time between 2 bytes received from USB
// increment every 100 us -> 100 ms timeout = 1 000
volatile uint8_t usb_timeout = 0;

// time between 2 bytes received from USART
// increment every 100 us -> 100 ms timeout = 1 000
volatile uint16_t usart_timeout = 0;
volatile uint8_t USART_last_start = 0;

// callback being called after byte is sent to USART
void (*volatile sent_callback)(void) = NULL;

// ondex of byte in ring_USB_datain to be sent to USART
volatile uint8_t usart_to_send = 0;
volatile bool usart_last_byte_sent = false;

volatile uint32_t active_devices = 0;
volatile uint32_t dirty_devices = 0;

volatile alive keep_alive = { 0, 0, 0, 0 };

volatile uint8_t mLED_In_Timeout = 2 * MLED_IN_MAX_TIMEOUT;
volatile uint8_t mLED_Out_Timeout = 2 * MLED_OUT_MAX_TIMEOUT;

// Power led blinks pwr_led_status times, then stays blank for some time
// and then repeats the whole cycle. This lets user to see software status.
volatile uint8_t pwr_led_base_timeout = PWR_LED_SHORT_COUNT;
volatile uint8_t pwr_led_base_counter = 0;
volatile uint8_t pwr_led_status_counter = 0;
volatile uint8_t pwr_led_status = 2;

volatile port_history sense_hist = { 0, 0 };
volatile master_waiting master_send_waiting = { 0 };

volatile uint8_t timeout_err_counter = TIMEOUT_ERR_TIMEOUT;

/** PRIVATE PROTOTYPES ********************************************************/

static void init(void);
static void init_devices(void);
static uint8_t calc_parity(uint8_t data);
static bool parity(uint8_t byte);
static void check_device_data_to_USB(void);
static void timer_10ms(void);
static uint8_t detect_hw_version(void);
static void resetBus(void);
static uint8_t xor(const uint8_t data[], uint8_t len);

// USB functions
static void USB_send(void);
static void USB_receive(void);
static void dump_buf_to_USB(ring_generic* buf);
static void parse_command_for_master(uint8_t start, uint8_t len);
static void USB_buffer_status(void);

// USART (XpressNET) functions
static void USART_receive_interrupt(void);
static void USART_check_timeouts(void);
static void USART_send_next_frame(void);
static void USART_send_rest_of_message(void);
static void USART_request_next_device(void);
static void USART_ni_sent(void);
static void USART_send(void);

/** INTERRUPTS ****************************************************************/

void __interrupt(high_priority) high_isr(void) {
    // USART send interrupt
    if ((PIE1bits.TXIE) && (PIR1bits.TXIF))
        if (sent_callback)
            sent_callback();

    // USART receive interrupt
    if (PIR1bits.RCIF)
        USART_receive_interrupt();
}

void __interrupt(low_priority) low_isr(void) {
    static volatile uint16_t ten_ms_counter = 0;

    if ((PIE1bits.TMR2IE) && (PIR1bits.TMR2IF)) {
        // Timer2 on 10 us

        // USART currently requested device timeout
        if ((current_dev.timeout > 0) && (current_dev.timeout < NI_TIMEOUT))
            current_dev.timeout++;

        // XpressNET direction is turned to "IN" as soon as possible after
        // last byte was sent to XpressNET.
        // This is done independently on any callbacks. This needs to be done really fast!
        if ((usart_last_byte_sent) && (TXSTAbits.TRMT))
            XPRESSNET_DIR = XPRESSNET_IN;

        // Detection of USART device answering normal inquiry.
        if ((!BAUDCONbits.RCIDL) && (!current_dev.reacted) && (XPRESSNET_DIR == XPRESSNET_IN)) {
            // receiver detected start bit -> wait for all data
            current_dev.reacted = true;
            current_dev.timeout = 0; // device answered -> provide long window
            usart_timeout = 0;
        }

        // usart receive timeout
        if (usart_timeout < USART_MAX_TIMEOUT)
            usart_timeout++;

        ten_ms_counter++;
        if (ten_ms_counter >= 1000) {
            ten_ms_counter = 0;
            timer_10ms();
        }

        PIR1bits.TMR2IF = 0; // reset overflow flag
    }
}

/** FUNCTIONS *****************************************************************/

void main(void) {
    init();
    USBDeviceAttach();

    while (true) {
        USBDeviceTasks();

        // Normal inquiry answer timeout.
        // This function is not placed in interrupt to serve interrupt as
        // fast as possible.
        if ((current_dev.timeout >= NI_TIMEOUT) && (IO_XNPWR_get()) && (sense_hist.state)) {
            // device did not answer in 120 us

#ifdef RACK_ENABLE
            if (IsRACKRound) {
                // device did not answer request for acknowledgement
                if ((dirty_devices >> current_dev.index) & 0b1) {
                    // for second time -> device is not active
                    dirty_devices &= ~((uint32_t)1 << current_dev.index);
                    active_devices &= ~((uint32_t)1 << current_dev.index);
                    master_send_waiting.bits.active_devices = true;
                } else {
                    // for first time -> notice
                    dirty_devices |= ((uint32_t)1 << current_dev.index);
                }
            }
#endif

            current_dev.timeout = 0;
            USART_send_next_frame();
        }

        // Transmission to USART ended.
        // This function is not placed in interrupt to serve interrupt as
        // fast as possible.
        if ((usart_last_byte_sent) && (TXSTAbits.TRMT)) {
            usart_last_byte_sent = 0;
            if (sent_callback) { sent_callback(); }
        }

        USB_receive();
        USB_send();
        USART_check_timeouts();
        CDCTxService();

        // clear watchdog timer
        ClrWdt();
    }
}

void init(void) {
#if (defined(__18CXX) & !defined(PIC18F87J50_PIM))
    ADCON1 |= 0x0F; // Default all pins to digital
#endif

    // init ring buffers
    ringInit(&ring_USB_datain);
    ringInit(&ring_USART_datain);

    // switch off AD convertors (USART is not working when not switched off manually)
    ANSEL = 0x00;
    ANSELH = 0x00;

    // enable PORTA and PORTB pull-ups (because of USART reading)
    INTCON2bits.RABPU = 0;

    version_hw = detect_hw_version();

    // Initialize all GPIO
    IO_init();

    // setup timer2 on 100 us
    T2CONbits.T2CKPS = 0b01; // prescaler 4x
    PR2 = 30;                // setup timer period register to interrupt every 10 us
    TMR2 = 0x00;             // reset timer counter
    PIR1bits.TMR2IF = 0;     // reset overflow flag
    PIE1bits.TMR2IE = 1;     // enable timer2 interrupts
    IPR1bits.TMR2IP = 0;     // timer2 interrupt low level
    INTCONbits.PEIE = 1;     // Enable peripheral interrupts
    T2CONbits.TMR2ON = 1;    // enable timer2

    init_devices();
    USBDeviceInit();
    USARTInit();

    INTCONbits.GIEL = 1;        // Enable low-level interrupts
    INTCONbits.GIEH = 1;        // Enable high-level interrupts
    RCONbits.IPEN = 1;          // enable all interrupts
}

void timer_10ms(void) {
    // usb receive timeout
    if (usb_timeout < USB_MAX_TIMEOUT)
        usb_timeout++;

    // keep-alive
    if (keep_alive.send) {
        keep_alive.send_timer++;
        if (keep_alive.send_timer >= KA_SEND_INTERVAL) {
            keep_alive.send_timer = 0;
            master_send_waiting.bits.keep_alive = true;
        }
    }

    if (keep_alive.receive) {
        keep_alive.receive_timer++;
        if (keep_alive.receive_timer >= KA_RECEIVE_MAX) {
            // computer crashed -> turn the bus off
            IO_XNPWR_set(false);
            resetBus();
            keep_alive.receive_timer = 0;
            keep_alive.receive = false;
            master_send_waiting.bits.status = true;
        }
    }

#ifndef DEBUG
    // mLEDIn timeout
    if (mLED_In_Timeout < 2 * MLED_IN_MAX_TIMEOUT) {
        mLED_In_Timeout++;
        if (mLED_In_Timeout == MLED_IN_MAX_TIMEOUT) {
            IO_LED_In_On();
        }
    }

    // mLEDOut timeout
    if ((mLED_Out_Timeout < 2 * MLED_OUT_MAX_TIMEOUT) && (USBGetDeviceState() == CONFIGURED_STATE)) {
        mLED_Out_Timeout++;
        if (mLED_Out_Timeout == MLED_OUT_MAX_TIMEOUT) {
            IO_LED_Out_Off();
        }
    }
#endif

    // pwrLED toggling
    pwr_led_base_counter++;
    if (pwr_led_base_counter >= pwr_led_base_timeout) {
        pwr_led_base_counter = 0;
        pwr_led_status_counter++;

        if (pwr_led_status_counter == 2 * pwr_led_status) {
            // wait between cycles
            pwr_led_base_timeout = PWR_LED_LONG_COUNT;
            IO_LED_Pwr_Off();
        } else if (pwr_led_status_counter > 2 * pwr_led_status) {
            // new base cycle
            pwr_led_base_timeout = PWR_LED_SHORT_COUNT;
            pwr_led_status_counter = 0;
            IO_LED_Pwr_On();
        } else {
            IO_LED_Pwr_Toggle();
        }
    }

    // sense history
    if (sense_hist.state != IO_SENSE_get()) {
        if (sense_hist.timeout < PORT_TIMEOUT) {
            sense_hist.timeout++;
            if (sense_hist.timeout >= PORT_TIMEOUT) {
                sense_hist.state = IO_SENSE_get();
                if (!IO_SENSE_get())
                    resetBus();
                sense_hist.timeout = 0;
                master_send_waiting.bits.status = true;
            }
        }
    } else {
        sense_hist.timeout = 0;
    }

    if (timeout_err_counter < TIMEOUT_ERR_TIMEOUT)
        timeout_err_counter++;
}

bool USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void* pdata, uint16_t size) {
    USBCDCEventHandler(event, pdata, size);

    switch( (int) event )
    {
        case EVENT_TRANSFER:
            break;

        case EVENT_SOF:
            break;

        case EVENT_SUSPEND:
            IO_LED_Out_On();
            ringInit(&ring_USART_datain);
            ringInit(&ring_USB_datain);
            break;

        case EVENT_RESUME:
            IO_LED_Out_Off();
            break;

        case EVENT_CONFIGURED:
            CDCInitEP();
            IO_LED_Out_Off();
            break;

        case EVENT_SET_DESCRIPTOR:
            break;

        case EVENT_EP0_REQUEST:
            USBCheckCDCRequest();
            break;

        case EVENT_BUS_ERROR:
            break;

        case EVENT_TRANSFER_TERMINATED:
            break;

        default:
            break;
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////
/* CHECKING USART IN TIMEOUT
 * This function is called in main loop, timing is not very critical.
 * It checks for timeouts from XpressNET devices. For example, when device sends
 * only part of the message, this timeout ensures clearing of master`s buffers
 * with unfinished message.
 */

void USART_check_timeouts(void) {
    // check for timeout
    if (((USART_last_start != ring_USART_datain.ptr_e) || (current_dev.reacted))
        && (usart_timeout >= USART_MAX_TIMEOUT) && (!current_dev.finished)) {

        // the (!current_dev.finished) condition guarantees us this if will
        // not be entered after the message was received

        // disable receive interrupt, so it does not interfere with this function
        PIE1bits.RCIE = 0;

        // delete last incoming message and wait for next message
		ringRewindEnd(&ring_USART_datain, USART_last_start);
        usart_timeout = 0;
        current_dev.reacted = false;

        // inform PC about timeout
        if (timeout_err_counter == TIMEOUT_ERR_TIMEOUT) {
            timeout_err_counter = 0;
            master_send_waiting.bits.usart_incoming_timeout = true;
        }

        // send next message to XpressNET
        USART_send_next_frame();
    }
}

////////////////////////////////////////////////////////////////////////////////
/* RECEIVING DATA FROM XPRESSNET DEVICES
 * This function is called in high-priority interrupt, be careful about
 * interferences with main code!
 * This function must be as fast as possible!
 */

void USART_receive_interrupt(void) {
    // We do not check xor in this function intentionally.
    // XOR should be checked in PC.

    static nine_data received = { 0, 0 };
    uint8_t tmp, parity;

    usart_timeout = 0;

    received = USARTReadByte();

    if (current_dev.finished) {
        // next byte was received after the end of message -> probably
        // bad length -> increase timeout to let the device transfer
        // all the data
        current_dev.timeout = NI_TIMEOUT / 2;
        return;
    }

    current_dev.reacted = true;
    current_dev.timeout = 0;

#ifdef RACK_ENABLE
    if (!((active_devices >> current_dev.index) & 0b1)) {
        active_devices |= ((uint32_t)1 << current_dev.index);
        master_send_waiting.bits.active_devices = true;
    }
    dirty_devices &= ~((uint32_t)1 << current_dev.index);
#endif

    if (ringFreeSpace(&ring_USART_datain) < 2) {
        // reset buffer and wait for next message
		ringRewindEnd(&ring_USART_datain, USART_last_start);
        return;
    }

    // The content of function "ringAddByte" is inlined to this function (because of speed).

    if (USART_last_start == ring_USART_datain.ptr_e) {
        // first byte -> add call byte before first byte

        // parity function is inlined (because of speed)
        parity = 0;
        if ((tmp = current_dev.index) & 0b1) parity = !parity;
        if ((tmp = tmp >> 1) & 0b1) parity = !parity;
        if ((tmp = tmp >> 1) & 0b1) parity = !parity;
        if ((tmp = tmp >> 1) & 0b1) parity = !parity;
        if ((tmp = tmp >> 1) & 0b1) parity = !parity;

        ringAddByte(&ring_USART_datain, (uint8_t)(current_dev.index + (0b11 << 5) + (parity << 7)));
    }

    ringAddByte(&ring_USART_datain, received.data);

    if (USART_last_message_len >= msg_len(ring_USART_datain, USART_last_start)) {
#ifdef RACK_ENABLE
        if (IsRACKRound) {
			ringRewind(ring_USART_datain, USART_last_start);
        } else {
            USART_last_start = ring_USART_datain.ptr_e;
        }
#else
        USART_last_start = ring_USART_datain.ptr_e;
#endif

        current_dev.finished = true;

        // whole message received -> wait a few microseconds and send next data
        current_dev.timeout = NI_TIMEOUT / 2;
    }

#ifndef DEBUG
    // toggle LED
    if (mLED_In_Timeout >= 2 * MLED_IN_MAX_TIMEOUT) {
        IO_LED_In_Off();
        mLED_In_Timeout = 0;
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Check for data in ring_USART_datain and send complete data to USB.

void USB_send(void) {
    if (master_send_waiting.all > 0)
        check_device_data_to_USB();

    // check for USB ready
    if (!mUSBUSARTIsTxTrfReady())
        return;

    uint8_t len = msg_len(ring_USART_datain, ring_USART_datain.ptr_b);

    if (((ringLength(&ring_USART_datain)) >= 3) && (ringLength(&ring_USART_datain) >= len)) {
        // send message
        ringSerialize(&ring_USART_datain, USB_Out_Buffer, ring_USART_datain.ptr_b, len);
        putUSBUSART(USB_Out_Buffer, len);
        ringRemoveFrame((ring_generic*)&ring_USART_datain, len);
    }
}

////////////////////////////////////////////////////////////////////////////////
/* Receive data from USB and add it to ring_USB_datain.
 * Index of start of last message is in
 * WARNING! This function cannot rely on ring_USB_datain.ptr_b value!
 * ring_USB_datain.ptr_b could be changed in interrupt called at any time!
 * More specifically, USART_receive_interrupt could add data at beginning of
 * the USB buffer. It is necessary to keep this information always in mind.
 */

void USB_receive(void) {
    static uint8_t last_start = 0;

    if ((USBDeviceState != CONFIGURED_STATE) || (USBIsDeviceSuspended()))
        return;

	// ring_USB_datain overflow check
	if (ringFull(&ring_USB_datain)) {
		// delete last message
		ringRewindEnd(&ring_USB_datain, last_start);
		master_send_waiting.bits.usb_usart_overflow = true;
		return;
	}

	uint8_t received_len = getsUSBUSART((ring_generic*)&ring_USB_datain, ringFreeSpace(&ring_USB_datain));
	if (received_len == 0) {
		// check for timeout
		if ((usb_timeout >= USB_MAX_TIMEOUT) && (last_start != ring_USB_datain.ptr_e)) {
			ringRewindEnd(&ring_USB_datain, last_start);
			master_send_waiting.bits.usb_incoming_timeout = true;
			usb_timeout = 0;
		}
		return;
	}

	// some data received ...
	usb_timeout = 0;

	// data received -> parse data
	// at least 3 bytes must be in buffer to start parsing
	// (call byte + header byte + xor)
	while ((ringDistance(&ring_USB_datain, last_start, ring_USB_datain.ptr_e) >= 3)
		&& (USB_last_message_len >= msg_len(ring_USB_datain, last_start))) {
		// while message received

		// check for parity
		if (parity(ring_USB_datain.data[last_start])) {
			// parity error
			ringRewindEnd(&ring_USB_datain, last_start);
			master_send_waiting.bits.usb_parity_error = true;
			return;
		}

		// check xor
		uint8_t xor = 0;
		for (uint8_t i = 0; i < msg_len(ring_USB_datain, last_start) - 1; i++)
			xor ^= ring_USB_datain.data[(i + last_start + 1) % RINGBUF_SIZE];

		if (xor != 0) {
			// xor error
			ringRewindEnd(&ring_USB_datain, last_start);
			master_send_waiting.bits.usb_xor_error = true;
			return;
		}

		// xor ok -> parse data
		if (((ring_USB_datain.data[last_start] >> 5) & 0b11) == 0b01) {
			parse_command_for_master(last_start, msg_len(ring_USB_datain, last_start));
			ringRewindEnd(&ring_USB_datain, last_start);
		} else {
			if (!sense_hist.state) {
				ringRewindEnd(&ring_USB_datain, last_start);
				master_send_waiting.bits.xn_no_power = true;
				return;
			}

			if (!IO_XNPWR_get()) {
				ringRewindEnd(&ring_USB_datain, last_start);
				master_send_waiting.bits.xn_transistor_closed = true;
				return;
			}

			last_start = (last_start + msg_len(ring_USB_datain, last_start)) % RINGBUF_SIZE;
		}
	}

#ifndef DEBUG
	// toggle LED
	if (mLED_Out_Timeout >= 2 * MLED_OUT_MAX_TIMEOUT) {
		IO_LED_Out_On();
		mLED_Out_Timeout = 0;
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////
/* Parse data intended for master.
 */

void parse_command_for_master(uint8_t start, uint8_t len) {
    uint8_t db1 = ring_USB_datain.data[(start + 2) % RINGBUF_SIZE];

    if ((db1 >> 4) == 0xA) {
        // set master status
        const bool xnPwr = db1 & 0b1;
        IO_XNPWR_set(xnPwr);
        if (xnPwr) {
            USARTEnableReceive();
        } else {
            resetBus();
        }
        keep_alive.send = ((db1 >> 3) & 0b1);
        keep_alive.receive = ((db1 >> 2) & 0b1);
        keep_alive.receive_timer = 0;
        keep_alive.send_timer = 0;
        master_send_waiting.bits.status = true;
    } else if (db1 == 0xA2) {
        // transistor status request
        master_send_waiting.bits.status = true;
    } else if (db1 == 0x80) {
        // version request
        master_send_waiting.bits.version = true;
    } else if (db1 == 0x81) {
        // response request
        master_send_waiting.bits.ok = true;
    } else if (db1 == 0x82) {
        // active device list request
        master_send_waiting.bits.active_devices = true;
    } else if (db1 == 0x05) {
        // keep-alive
        keep_alive.receive_timer = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////
/* SEND NEXT DATA TO XPRESSNET DEVICE.
 * This function checks if message is present in USB->USART buffer. If yes,
 * the mesasge is sent to device. Otherwise, next device is requested with
 * normal inquiry.
 * This function must be called from main loop. We must ensure that this
 * function is not called when data are received, but not yet parsed. Otherwise,
 * it could send out data intended for uLI-master!
 */

void USART_send_next_frame(void) {
    uint8_t ring_length = ringDistance(&ring_USB_datain, ring_USB_datain.ptr_b, ring_USB_datain.ptr_e);

    // check if there is a message from PC to be sent to XpressNET
    if ((ring_length >= 3) && (ring_length >= msg_len(ring_USB_datain, ring_USB_datain.ptr_b))) {
        // yes -> send the message
        usart_to_send = (ring_USB_datain.ptr_b + 1) % RINGBUF_SIZE;
        XPRESSNET_DIR = XPRESSNET_OUT;
        current_dev.reacted = false; // we do not want USART timeout to overflow
        current_dev.finished = false;
        sent_callback = &(USART_send_rest_of_message);
        usart_last_byte_sent = 0;
        USARTWriteByte(1, ring_USB_datain.data[ring_USB_datain.ptr_b]);
        PIE1bits.TXIE = 1;
    } else {
        // no -> send normal inquiry to next XpressNET device
        USART_request_next_device();
    }
}

/* SEND REST OF MESSAGE TO USART.
 * This fnction is called as callback (from interrupt!) after a byte is
 * sent to USART. It sends next byte. After last byte is sent,
 * USART_request_next_device is called as callback.
 */
void USART_send_rest_of_message(void) {
    USARTWriteByte(0, ring_USB_datain.data[usart_to_send]);
    usart_to_send = (usart_to_send + 1) % RINGBUF_SIZE;

    if (usart_to_send == ((ring_USB_datain.ptr_b + msg_len(ring_USB_datain, ring_USB_datain.ptr_b)) % RINGBUF_SIZE)) {
        // last byte sending

        ring_USB_datain.ptr_b = usart_to_send; // whole message sent
        if (ring_USB_datain.ptr_b == ring_USB_datain.ptr_e)
			ring_USB_datain.empty = true;

        sent_callback = &(USART_request_next_device);
        usart_last_byte_sent = 1;
        PIE1bits.TXIE = 0;
    } else {
        // other-than-last byte sending
        sent_callback = &(USART_send_rest_of_message);
        usart_last_byte_sent = 0;
        PIE1bits.TXIE = 1;
    }
}

////////////////////////////////////////////////////////////////////////////////

// request next XpressNET device
void USART_request_next_device(void) {
    uint32_t tmp = 0;

    // 1) pick next device
    current_dev.index++;
    if (current_dev.index >= DEVICE_COUNT) {
        current_dev.round++;
        if (current_dev.round >= ROUND_MAX) { current_dev.round = 0; }
        current_dev.index = 1; // 0 == broadcast (not a device)
    }

#ifdef RACK_ENABLE
    // Are we supposed to send request for acknowledgement (RACK)?
    // Which device are we supposed to send RACK to?
    if (IsRACKRound) {
        tmp = active_devices >> current_dev.index;
        if (tmp == 0) {
            // all active devices requested in this round
            current_dev.round = 0;
            current_dev.index = 1;
        } else {
            // at least one active device has not been requested in
            // this round yet -> find it and request it
            while (!(tmp & 0b1)) {
                tmp = tmp >> 1;
                current_dev.index++;
            }
        }
    }
#endif

    // 2) request current device
    USARTEnableReceive();
    current_dev.timeout = 0;
    current_dev.reacted = false;
    current_dev.finished = false;
    XPRESSNET_DIR = XPRESSNET_OUT;
    sent_callback = &(USART_ni_sent);
    PIE1bits.TXIE = 0;
    usart_timeout = 0;
#ifdef RACK_ENABLE
    USARTWriteByte(1, calc_parity(current_dev.index + ((!IsRACKRound) << 6))); // send normal inquiry or request acknowledgement
#else
    USARTWriteByte(1, calc_parity(current_dev.index + (0x40))); // send normal inquiry
#endif
    usart_last_byte_sent = true;
}

////////////////////////////////////////////////////////////////////////////////

// Debug function: dump buffer to USB
void dump_buf_to_USB(ring_generic* buf) {
    for (uint8_t i = 0; i <= RINGBUF_SIZE; i++)
        USB_Out_Buffer[i] = buf->data[i];
    putUSBUSART(USB_Out_Buffer, RINGBUF_SIZE);
}

////////////////////////////////////////////////////////////////////////////////

void init_devices(void) {
    current_dev.index = 0;
    current_dev.timeout = 1; // this will cause the processor to send first normal inquiry after some time
    current_dev.reacted = false;
}

////////////////////////////////////////////////////////////////////////////////

bool parity(uint8_t byte) {
	bool parity = false;
	for (uint8_t i = 0; i < 8; i++, byte >>= 1)
		if (byte & 1)
			parity = !parity;
	return parity;
}

// Calculate parity and return uint8_t with the leftmost parity bit (even parity).
uint8_t calc_parity(uint8_t data) {
	return parity(data) ? (data | 0x80) : data;
}

////////////////////////////////////////////////////////////////////////////////

/* This callback is called after normal inquiry is sent.
 * WARNING: this function is called in high-priority interrupt
 * It could anyhow interleave low-priority interrupt (especially the part
 * working with current_dev.timeout = 0) !!
 */
void USART_ni_sent(void) {
    // device may react before this function is called
    // do not replace this if with ternary operator, it does not behave well
    current_dev.timeout = 1;
    if (current_dev.reacted) current_dev.timeout = 0;
    sent_callback = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/* Send data from uLI-master to PC (not from XpressNET devices).
 */

void check_device_data_to_USB(void) {
    if (!mUSBUSARTIsTxTrfReady())
        return;

    USB_Out_Buffer[0] = 0xA0;
    USB_Out_Buffer[1] = 0x01;

    if (master_send_waiting.bits.usb_incoming_timeout) {
        master_send_waiting.bits.usb_incoming_timeout = false;
        USB_Out_Buffer[2] = 0x01;
        USB_Out_Buffer[3] = 0x00;
        putUSBUSART(USB_Out_Buffer, 4);

    } else if (master_send_waiting.bits.usart_incoming_timeout) {
        master_send_waiting.bits.usart_incoming_timeout = false;
        USB_Out_Buffer[2] = 0x02;
        USB_Out_Buffer[3] = 0x03;
        putUSBUSART(USB_Out_Buffer, 4);

    } else if (master_send_waiting.bits.ok) {
        USB_Out_Buffer[2] = 0x04;
        USB_Out_Buffer[3] = 0x05;
        putUSBUSART(USB_Out_Buffer, 4);

    } else if (master_send_waiting.bits.keep_alive) {
        master_send_waiting.bits.keep_alive = false;
        USB_Out_Buffer[2] = 0x05;
        USB_Out_Buffer[3] = 0x04;
        putUSBUSART(USB_Out_Buffer, 4);

    } else if (master_send_waiting.bits.usb_usart_overflow) {
        master_send_waiting.bits.usb_usart_overflow = false;
        USB_Out_Buffer[2] = 0x06;
        USB_Out_Buffer[3] = 0x07;
        putUSBUSART(USB_Out_Buffer, 4);

    } else if (master_send_waiting.bits.usb_xor_error) {
        master_send_waiting.bits.usb_xor_error = false;
        USB_Out_Buffer[2] = 0x07;
        USB_Out_Buffer[3] = 0x06;
        putUSBUSART(USB_Out_Buffer, 4);

    } else if (master_send_waiting.bits.usb_parity_error) {
        master_send_waiting.bits.usb_parity_error = false;
        USB_Out_Buffer[2] = 0x08;
        USB_Out_Buffer[3] = 0x09;
        putUSBUSART(USB_Out_Buffer, 4);

    } else if (master_send_waiting.bits.xn_no_power) {
        master_send_waiting.bits.xn_no_power = false;
        USB_Out_Buffer[2] = 0x09;
        USB_Out_Buffer[3] = 0x08;
        putUSBUSART(USB_Out_Buffer, 4);

    } else if (master_send_waiting.bits.xn_transistor_closed) {
        master_send_waiting.bits.xn_transistor_closed = false;
        USB_Out_Buffer[2] = 0x0A;
        USB_Out_Buffer[3] = 0x0B;
        putUSBUSART(USB_Out_Buffer, 4);

    } else if (master_send_waiting.bits.status) {
        master_send_waiting.bits.status = false;
        USB_Out_Buffer[1] = 0x11;
        USB_Out_Buffer[2] = (uint8_t)(0xA0 + IO_XNPWR_get() + (sense_hist.state << 1) + ((keep_alive.receive & 0b1) << 2) + ((keep_alive.send & 0b1) << 3));
        USB_Out_Buffer[3] = xor(USB_Out_Buffer+1, 2);
        putUSBUSART(USB_Out_Buffer, 4);

    } else if (master_send_waiting.bits.version) {
        master_send_waiting.bits.version = false;
        USB_Out_Buffer[1] = 0x13;
        USB_Out_Buffer[2] = 0x80;
        USB_Out_Buffer[3] = version_hw;
        USB_Out_Buffer[4] = VERSION_SW;
        USB_Out_Buffer[5] = xor(USB_Out_Buffer+1, 4);
        putUSBUSART(USB_Out_Buffer, 6);

    } else if (master_send_waiting.bits.active_devices) {
        master_send_waiting.bits.active_devices = false;
        USB_Out_Buffer[1] = 0x15;
        USB_Out_Buffer[2] = 0x82;
        USB_Out_Buffer[3] = active_devices >> 24;
        USB_Out_Buffer[4] = (active_devices >> 16) & 0xFF;
        USB_Out_Buffer[5] = (active_devices >> 8) & 0xFF;
        USB_Out_Buffer[6] = active_devices & 0xFF;
        USB_Out_Buffer[7] = xor(USB_Out_Buffer+1, 6);
        putUSBUSART(USB_Out_Buffer, 8);
    }
}

////////////////////////////////////////////////////////////////////////////////

uint8_t detect_hw_version(void) {
    // HW v5.0 contains pull-down on IO_HW_VERSION pin
    // In HW <v5.0 the pin is floating
    IO_HW_VERSION_TRIS = 0; // output
    IO_HW_VERSION_LAT = 1; // output high
    __delay_us(1);
    IO_HW_VERSION_TRIS = 1; // input
    NOP();
    NOP();
    return IO_HW_VERSION_PORT ? VERSION_HW_OLD : VERSION_HW_5;
}

////////////////////////////////////////////////////////////////////////////////

void resetBus(void) {
    current_dev.reacted = false;
    current_dev.timeout = 1;
    current_dev.index = 1;
    active_devices = 0;
    dirty_devices = 0;
    USARTDisableReceive();
}

////////////////////////////////////////////////////////////////////////////////

uint8_t xor(const uint8_t data[], uint8_t len) {
    uint8_t xor = 0;
    for (uint8_t i = 0; i < len; i++)
        xor ^= data[i];
    return xor;
}
