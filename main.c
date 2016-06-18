/********************************************************************
 FileName:		main.c
 Processor:		PIC18F14K50
 Hardware:		uLI4 - JH&MP 2016
 Complier:		Microchip C18
 Author:		Jan Horacek
 * 
/** INCLUDES *******************************************************/
#include "usb.h"
#include "usb_function_cdc.h"
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "usb_device.h"
#include "usb.h"
#include "usart.h"
#include "main.h"
#include "ringBuffer.h"
#include "eeprom.h"

/** CONFIGURATION **************************************************/

// CONFIG1L
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection bits (No CPU System Clock divide)
#pragma config USBDIV = ON		// USB Clock Selection bit (USB clock comes from the OSC1/OSC2 divided by 2)

// CONFIG1H
#pragma config FOSC = HS		// Oscillator Selection bits (HS oscillator)
#pragma config PLLEN = ON		// 4 X PLL Enable bit (Oscillator multiplied by 4)
#pragma config PCLKEN = ON		// Primary Clock Enable bit (Primary clock enabled)
#pragma config FCMEN = OFF		// Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF		// Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON		// Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = SBORDIS	// Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 27		// Brown-out Reset Voltage bits (VBOR set to 2.7 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF		// Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config WDTPS = 32768	// Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config HFOFST = OFF		// HFINTOSC Fast Start-up bit (The system clock is held off until the HFINTOSC is stable.)
#pragma config MCLRE = ON		// MCLR Pin Enable bit (MCLR pin enabled; RA3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON		// Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON			// Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config BBSIZ = ON		// Boot Block Size Select bit (2kW boot block size)
#pragma config XINST = OFF		// Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

#pragma config CP0 = OFF		// Code Protection bit (Block 0 not code-protected)
#pragma config CP1 = OFF		// Code Protection bit (Block 1 not code-protected)
#pragma config CPB = OFF		// Boot Block Code Protection bit (Boot block not code-protected)
#pragma config CPD = OFF		// Data EEPROM Code Protection bit (Data EEPROM not code-protected)
#pragma config WRT0 = OFF		// Table Write Protection bit (Block 0 not write-protected)
#pragma config WRT1 = OFF		// Table Write Protection bit (Block 1 not write-protected)
#pragma config WRTC = OFF		// Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTB = OFF		// Boot Block Write Protection bit (Boot block not write-protected)
#pragma config WRTD = OFF		// Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config EBTR0 = OFF		// Table Read Protection bit (Block 0 not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF		// Table Read Protection bit (Block 1 not protected from table reads executed in other blocks)
#pragma config EBTRB = OFF		// Boot Block Table Read Protection bit (Boot block not protected from table reads executed in other blocks)

/** D E F I N E S ************************************************************/

#define msg_len(buf,start)      (((buf).data[((start)+1) & (buf).max] & 0x0F)+3)

#define USB_last_message_len	ringDistance(ring_USB_datain, last_start, ring_USB_datain.ptr_e)
#define USART_last_message_len	ringDistance(ring_USART_datain, last_start, ring_USART_datain.ptr_e)

#define USB_MAX_TIMEOUT                   10		// 100 ms
#define USART_MAX_TIMEOUT                 50		// 500 us

#define DEVICE_COUNT                      32
#define NI_TIMEOUT                        12        // normal inquiery timeout = 120 us

#define NI_ROUND_COUNT                     5

/** V A R I A B L E S ********************************************************/
#pragma udata
char USB_Out_Buffer[32];

// USART and USB buffers
volatile ring_generic ring_USB_datain;
volatile ring_generic ring_USART_datain;

// currently requested device
volatile current current_dev;

#pragma idata

volatile BYTE usb_timeout = 0;
	// increment every 100 us -> 100 ms timeout = 1 000
volatile WORD usart_timeout = 0;
	// increment every 100 us -> 100 ms timeout = 1 000
volatile WORD ten_ms_counter = 0;
    // 10 ms counter

// callback being called after byte is sent to USART
void (*volatile sent_callback)(void) = NULL;

volatile BYTE usart_to_send = 0;

volatile BOOL usb_configured = FALSE;
volatile BYTE tmp;
volatile BOOL usart_last_byte_sent = FALSE;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();

// general functions
void user_init(void);
BYTE calc_xor(BYTE* data, BYTE len);
void Timer2(void);
void init_EEPROM(void);
void init_devices(void);
BYTE calc_parity(BYTE data);

// USB functions
void USB_send(void);
void USB_receive(void);
void dump_buf_to_USB(ring_generic* buf);
void USBDeviceTasks(void);
void parse_command_for_master(BYTE start, BYTE len);
BOOL USB_send_data(BYTE first, BYTE second, BYTE third);

// USART (XpressNET) functions
void USART_send_next_frame(void);
void USART_send_rest_of_message(void);
void USART_request_next_device(void);
void USART_request_current_device(void);
void USART_ni_sent(void);
void USART_pick_next_device(void);
void USART_send(void);
void USART_receive(void);


/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
		_asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
		_asm goto YourLowPriorityISRCode _endasm
	}
	
	#pragma code
	
	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
		#if defined(USB_INTERRUPT)
			USBDeviceTasks();
		#endif

		// USART send interrupt
		if ((PIE1bits.TXIE) && (PIR1bits.TXIF)) {
			if (sent_callback) { sent_callback(); }
		}
        
	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
		
		// Timer2 on 10 us
		if ((PIE1bits.TMR2IE) && (PIR1bits.TMR2IF)) {
			
            if ((current_dev.timeout > 0) && (current_dev.timeout < NI_TIMEOUT)) { current_dev.timeout++; }
            
			if ((usart_last_byte_sent) && (TXSTAbits.TRMT)) {
                XPRESSNET_DIR  = XPRESSNET_IN;
                RCSTAbits.CREN = 1;    // enable RX
            }
            
            if (!BAUDCONbits.RCIDL) {
                mLED_Out_Toggle();
            }
            
            if ((!BAUDCONbits.RCIDL) && (!current_dev.reacted) && (RCSTAbits.CREN)) {
                // receiver detected start bit -> wait for all data
                current_dev.timeout = 0; // device answered -> provide long window
                current_dev.reacted = TRUE;
                usart_timeout = 0;
                
            }

            // usart receive timeout
            if (usart_timeout < USART_MAX_TIMEOUT) usart_timeout++;

            if (ten_ms_counter < 1000) { ten_ms_counter++; }
            else {
                ten_ms_counter = 0;
                
                // 10 ms overflow:
                
                // usb receive timeout
                if (usb_timeout < USB_MAX_TIMEOUT) usb_timeout++;
	
                // end of 10 ms counter
            }
            
			PIR1bits.TMR2IF = 0;		// reset overflow flag
		}
		
	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 

#endif


/** DECLARATIONS ***************************************************/
#pragma code

void main(void)
{
	InitializeSystem();

	while(1)
	{
		#if defined(USB_INTERRUPT)
			if(USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE))
			{
				USBDeviceAttach();
			}
		#endif

        if (current_dev.timeout >= NI_TIMEOUT) {
            // device did not answer in 120 us
            current_dev.timeout = 0;
            IOCBbits.IOCB5 = 0;     // disable pin interrupts
            USART_send_next_frame();
        }
        // TODO: calling function in interrupt is a problem, is this enough? (latentions)
            
		if ((usart_last_byte_sent) && (TXSTAbits.TRMT)) {
    		usart_last_byte_sent = 0;                
            if (sent_callback) { sent_callback(); }
		}
            
		USB_receive();
		USB_send();
        USART_receive();
		CDCTxService();
	}//end while
}//end main

void InitializeSystem(void)
{
    ADCON1 = 0x0F;   
    ADCON0 = 0;
    
    init_devices();
	user_init();
	init_EEPROM();
	USBDeviceInit();
	USARTInit();
}

void user_init(void)
{
	// init ring buffers
	ringBufferInit(ring_USB_datain, 32);
	ringBufferInit(ring_USART_datain, 32);
 
	// switch off AD convertors (USART is not working when not switched off manually)
	ANSEL = 0x00;
	ANSELH = 0x00;
	
	// Initialize all of the LED pins
	mInitAllLEDs();
	mLED_Pwr_On();
	mLED_In_On();
	mLED_Out_On();
	
	// setup timer2 on 100 us
	T2CONbits.T2CKPS = 0b01;	// prescaler 4x
	PR2 = 30;					// setup timer period register to interrupt every 10 us
	TMR2 = 0x00;				// reset timer counter
	PIR1bits.TMR2IF = 0;		// reset overflow flag
	PIE1bits.TMR2IE = 1;		// enable timer2 interrupts
	IPR1bits.TMR2IP = 0;		// timer2 interrupt low level
	
	RCONbits.IPEN = 1;			// enable high and low priority interrupts
	INTCONbits.PEIE = 1;		  // Enable peripheral interrupts
	INTCONbits.GIE = 1;			// enable global interrupts
	INTCONbits.GIEH = 1;
	INTCONbits.GIEL = 1;

    INTCONbits.RABIE = 0;       // enable port interrupts
    INTCON2bits.RABIP = 1;      // interrupt in high level
                                // interrupt is fired on port change
	
	T2CONbits.TMR2ON = 1;		// enable timer2
}

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events. For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device. In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function. You should modify these callback functions to take appropriate actions for each of these
// conditions. For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

void USBCBSuspend(void)
{
	#if defined(__C30__)
		USBSleepOnSuspend();
	#endif

	usb_configured = FALSE;
	//mLED_Out_On();
	ringClear((ring_generic*)&ring_USART_datain);
	ringClear((ring_generic*)&ring_USB_datain);
}

void USBCBWakeFromSuspend(void)
{
	usb_configured = TRUE;
	//mLED_Out_Off();
}

void USBCB_SOF_Handler(void)
{
	
}

void USBCBErrorHandler(void)
{
	
}

void USBCBCheckOtherReq(void)
{
	USBCheckCDCRequest();
}//end

void USBCBStdSetDscHandler(void)
{
	// Must claim session ownership if supporting this request
}

void USBCBInitEP(void)
{
	CDCInitEP();
	usb_configured = TRUE;
    mLED_Out_Off();
}

void USBCBSendResume(void)
{
	
}

#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
	switch(event)
	{
		case EVENT_CONFIGURED: 
			USBCBInitEP();
			break;
		case EVENT_SET_DESCRIPTOR:
			USBCBStdSetDscHandler();
			break;
		case EVENT_EP0_REQUEST:
			USBCBCheckOtherReq();
			break;
		case EVENT_SOF:
			USBCB_SOF_Handler();
			break;
		case EVENT_SUSPEND:
			USBCBSuspend();
			break;
		case EVENT_RESUME:
			USBCBWakeFromSuspend();
			break;
		case EVENT_BUS_ERROR:
			USBCBErrorHandler();
			break;
		case EVENT_TRANSFER:
			Nop();
			break;
		default:
			break;
	}	   
	return TRUE; 
}

////////////////////////////////////////////////////////////////////////////////

BYTE calc_xor(BYTE* data, BYTE len)
{
	int xor = 0, i;
	for (i = 0; i < len; i++) xor ^= data[i];
	return xor;
}

////////////////////////////////////////////////////////////////////////////////
/* How does USART data receiving look like:
 *	1) We wait for start of mesage for us (9. bit is 1 and address is XPRESSNET_ADDR)
 *	2) We receive data into ring_USART_datain (ring buffer), message start in ring buffer is saved to last_start.
 *	3) Once the message is received we check XOR. When the XOR does not match, we delete the message from ring buffer.
 *	4) When USB is ready to send messages, we send messages from ring buffer.
 *	This function specially does not and MUST NOT care about start of the ring buffer.
 *	Start of the ring buffer is moved by function which sends data to usb.
 */

void USART_receive(void)
{
    // We do not check xor in this function intentionally.
    // We let the PC to solve these xor issues.
    
	static nine_data received = {0, 0};
	static BYTE last_start = 0;
	
    /*if (last_start != ring_USART_datain.ptr_e) {
        mLED_Out_Toggle();
    }*/
    
	// check for (short) timeout
	if (((last_start != ring_USART_datain.ptr_e) || (current_dev.reacted)) && 
            (usart_timeout >= USART_MAX_TIMEOUT)) {
		// delete last incoming message and wait for next message
		ring_USART_datain.ptr_e = last_start;
		if (ring_USART_datain.ptr_e == ring_USART_datain.ptr_b) ring_USART_datain.empty = TRUE;
		usart_timeout = 0;

		// inform PC about timeout
		USB_send_data(0x01, 0x02, 0x03);

        // send next message to XpressNET
        USART_send_next_frame();
		
		return;
	}
	
	if ((XPRESSNET_DIR == XPRESSNET_OUT) || (!USARTInputData())) return;
	usart_timeout = 0;
    current_dev.reacted = TRUE;
    current_dev.timeout = 0;
		
	received = USARTReadByte();
	
    if (ringFreeSpace(ring_USART_datain) < 2) {
		// reset buffer and wait for next message
		ring_USART_datain.ptr_e = last_start;
		if (ring_USART_datain.ptr_e == ring_USART_datain.ptr_b) ring_USART_datain.empty = TRUE;
		return;
	}
    
    if (last_start == ring_USART_datain.ptr_e) {
        // first byte -> add call byte before first byte
        ringAddByte((ring_generic*)&ring_USART_datain, calc_parity(current_dev.index + (0b11 << 5)));
    }
        
	ringAddByte((ring_generic*)&ring_USART_datain, received.data);
	
	if (USART_last_message_len >= msg_len(ring_USART_datain, last_start)) {
		// whole message received -> wait a few microseconds and send next data
        last_start = ring_USART_datain.ptr_e;
        current_dev.timeout = NI_TIMEOUT / 2;
	}
	
}

////////////////////////////////////////////////////////////////////////////////
// Check for data in ring_USART_datain and send complete data to USB.

void USB_send(void)
{
    BYTE len = msg_len(ring_USART_datain, ring_USART_datain.ptr_b);
    
	// check for USB ready
	if (!mUSBUSARTIsTxTrfReady()) return;
    
    if (((ringLength(ring_USART_datain)) >= 3) && (ringLength(ring_USART_datain) >= len)) {
		// send message
		ringSerialize((ring_generic*)&ring_USART_datain, (BYTE*)USB_Out_Buffer, ring_USART_datain.ptr_b, len);
		putUSBUSART(USB_Out_Buffer, len);
		ringRemoveFrame((ring_generic*)&ring_USART_datain, len);
	}
}

////////////////////////////////////////////////////////////////////////////////
/* Receive data from USB and add it to ring_USB_datain.
 * Index of start of last message is in 
 */

void USB_receive(void)
{
	static BYTE last_start = 0;
	BYTE xor, i;
	BYTE received_len;
    BOOL parity;
	
	if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

	if(mUSBUSARTIsTxTrfReady())
	{
		// ring_USB_datain overflow check
		if (ringFull(ring_USB_datain)) {
			// delete last message
			ring_USB_datain.ptr_e = last_start;
			if (ring_USB_datain.ptr_b == ring_USB_datain.ptr_e) ring_USART_datain.empty = TRUE;
			
			// inform PC about full buffer
			USB_send_data(0x01, 0x06, 0x07);
			
			return;
		}
		
		received_len = getsUSBUSART((ring_generic*)&ring_USB_datain, ringFreeSpace(ring_USB_datain));
		if (received_len == 0) {
			// check for timeout
			if ((usb_timeout >= USB_MAX_TIMEOUT) && (last_start != ring_USB_datain.ptr_e)) {
				ring_USB_datain.ptr_e = last_start;
				usb_timeout = 0;
				if (ring_USB_datain.ptr_e == ring_USB_datain.ptr_b) ring_USB_datain.empty = TRUE;
				
				// inform PC about timeout
                USB_send_data(0x01, 0x01, 0x00);
			}
			return;
        }
        
        // some data received ...
		usb_timeout = 0;
		
		// data received -> parse data
        // at least 3 bytes must be in buffer to start parsing
        // (call byte + header byte + xor)
		while ((ringDistance(ring_USB_datain, last_start, ring_USB_datain.ptr_e) >= 3) &&
				(USB_last_message_len >= msg_len(ring_USB_datain, last_start))) {
            // while message received
            
            // check for parity
			for(i = 0, parity = 0; i < 8; i++) if ((ring_USB_datain.data[last_start] >> i) & 1) parity = !parity;
			if (parity != 0) {
				// parity error
                ringRemoveFromMiddle((ring_generic*)&ring_USB_datain, last_start, msg_len(ring_USB_datain, last_start));
				USB_send_data(0x01, 0x08, 0x09);
				return;
			}
            
			// check for xor
			for (i = 0, xor = 0; i < msg_len(ring_USB_datain, last_start)-2; i++)
				xor ^= ring_USB_datain.data[(i+last_start+1) & ring_USB_datain.max];
			
			if (xor != ring_USB_datain.data[(i+last_start+1) & ring_USB_datain.max]) {
				// xor error
				// delete content in the middle of ring buffer
				ringRemoveFromMiddle((ring_generic*)&ring_USB_datain, last_start, msg_len(ring_USB_datain, last_start));
				USB_send_data(0x01, 0x07, 0x06);
				return;
			}
			
            mLED_In_Toggle();
            
			// xor ok -> parse data
            if (((ring_USB_datain.data[last_start] >> 5) & 0b11) == 0b01) {
                parse_command_for_master(last_start, msg_len(ring_USB_datain, last_start));
                
                // remove message from buffer -> do not move last_start
                // (message moves in the buffer itself)
                ringRemoveFromMiddle((ring_generic*)&ring_USB_datain, last_start, msg_len(ring_USB_datain, last_start));                
            } else {
                last_start = (last_start+msg_len(ring_USB_datain, last_start))&ring_USB_datain.max;
            }
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
/* Parse data intended for master.
 * \start and \len reference to ring_USB_datain.
 * \len is WITH header byte, WITH xor byte and WITH call byte
 */ 

void parse_command_for_master(BYTE start, BYTE len)
{
    // TODO
    //mLED_Out_Toggle();
}

////////////////////////////////////////////////////////////////////////////////
/* Send data to USART.
 * How it works:
 *	This functino is called when TX is enabled and sends each byte manually.
 *	It always sends message from beginning of the ring_USB_datain buffer.
 *	\to_send is index of byte in ring_USB_datain to be send next time.
 *	Note: if this function has anything to send, it has to send it,
 *	because this function is called only once after TX gets ready.
 *	Returns: true if any data were send, otherwise false
 *	This function should be called only when there is anything to send and bus is in output state (at least 1 byte).
 */ 
// XpressNET is free and able to send next data (input data or normal inquiry
// to next device)
void USART_send_next_frame(void)
{
    BYTE ring_length = ringDistance(ring_USB_datain, ring_USB_datain.ptr_b, ring_USB_datain.ptr_e);
    RCSTAbits.CREN	= 0;    // disable RX
    
	// check if there is a message from PC to be sent to XpressNET
	if ((ring_length >= 3) && (ring_length >= msg_len(ring_USB_datain, ring_USB_datain.ptr_b))) {
        // yes -> send the message
        usart_to_send = (ring_USB_datain.ptr_b + 1) & ring_USB_datain.max;
        XPRESSNET_DIR = XPRESSNET_OUT;
        sent_callback = &(USART_send_rest_of_message);
        usart_last_byte_sent = 0;
        USARTWriteByte(1, ring_USB_datain.data[ring_USB_datain.ptr_b]);
        PIE1bits.TXIE = 1;
    } else {
        // no -> send normal inquiry to next XpressNET device
        USART_request_next_device();
    }
}

void USART_send_rest_of_message(void)
{    
	USARTWriteByte(0, ring_USB_datain.data[usart_to_send]);
	usart_to_send = (usart_to_send+1)&ring_USB_datain.max;
    
	if (usart_to_send == ((ring_USB_datain.ptr_b + msg_len(ring_USB_datain, ring_USB_datain.ptr_b))&ring_USB_datain.max)) {
        // last byte sending
        
		ring_USB_datain.ptr_b = usart_to_send;  // whole message sent
		if (ring_USB_datain.ptr_b == ring_USB_datain.ptr_e) { ring_USB_datain.empty = TRUE; }
        
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
void USART_request_next_device(void)
{
    USART_pick_next_device();
    USART_request_current_device();
}

////////////////////////////////////////////////////////////////////////////////
// Debug function: dump buffer to USB
void dump_buf_to_USB(ring_generic* buf)
{
	int i;
	for (i = 0; i <= buf->max; i++) USB_Out_Buffer[i] = buf->data[i];
	putUSBUSART(USB_Out_Buffer, buf->max+1);
}

////////////////////////////////////////////////////////////////////////////////

void init_EEPROM(void)
{
    // TODO: read data from EEPROM at boot
}

////////////////////////////////////////////////////////////////////////////////

void init_devices(void)
{
    current_dev.index = 0;
    current_dev.timeout = 1;    // this will cause the processor to send first normal inquiry after some time
    current_dev.reacted = FALSE;
}

////////////////////////////////////////////////////////////////////////////////

// send normal inquiry to current_dev
void USART_request_current_device(void)
{
    current_dev.timeout = 0;
    current_dev.reacted = FALSE;
    XPRESSNET_DIR = XPRESSNET_OUT;
    sent_callback = &(USART_ni_sent);
    PIE1bits.TXIE = 0;
    USARTWriteByte(1, calc_parity(current_dev.index + (0b10 << 5)));
    usart_last_byte_sent = TRUE;
    //mLED_Out_Off();
}

////////////////////////////////////////////////////////////////////////////////

// Calculate parity and return BYTE with the leftmost parity bit (even parity).
BYTE calc_parity(BYTE data)
{
    BYTE i, result, parity;
    parity = 0;
    result = data;
    for (i = 0; i < 7; i++) {
        if ((data & 0x01) == 0x01) { parity = !parity; }
        data = (data >> 1);
    }
    result |= (parity << 7);
    return result;
}

////////////////////////////////////////////////////////////////////////////////

// This callback is called after normal inquiry is sent.
void USART_ni_sent(void)
{    
    XPRESSNET_DIR = XPRESSNET_IN;
    current_dev.timeout = 1;
    tmp = PORTB;
    sent_callback = NULL;
}

////////////////////////////////////////////////////////////////////////////////

// pick next device
void USART_pick_next_device(void)
{
    current_dev.index++;
    if (current_dev.index >= DEVICE_COUNT) {
        current_dev.index = 1;      // 0 == broadcast (not a device)
    }
}

////////////////////////////////////////////////////////////////////////////////

BOOL USB_send_data(BYTE first, BYTE second, BYTE third)
{    
    USB_Out_Buffer[0] = first;
    USB_Out_Buffer[1] = second;
    USB_Out_Buffer[2] = third;
    if (mUSBUSARTIsTxTrfReady()) {
        putUSBUSART(USB_Out_Buffer, 3);
        return TRUE;
    } else {
        return FALSE;
    }
}

////////////////////////////////////////////////////////////////////////////////
