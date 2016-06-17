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
#define USB_msg_len(start)		((ring_USB_datain.data[start] & 0x0F)+2)	  // len WITH header byte and WITH xor byte
#define USB_last_message_len	ringDistance(&ring_USB_datain, last_start, ring_USB_datain.ptr_e)

#define USART_msg_len(start)	((ring_USART_datain.data[(start+1) & ring_USART_datain.max] & 0x0F)+3)	// length of xpressnet message is 4-lower bits in second byte
#define USART_last_message_len	ringDistance(&ring_USART_datain, last_start, ring_USART_datain.ptr_e)
#define USART_msg_to_send		((ringLength(&ring_USB_datain) >= 2) && (ringLength(&ring_USB_datain) >= USB_msg_len(ring_USB_datain.ptr_b)))

#define USB_MAX_TIMEOUT                   10		// 100 ms
#define USART_MAX_TIMEOUT                  2		// 20 ms
#define TIMESLOT_MAX_TIMEOUT             500		// 5 s (according to specification)
#define TIMESLOT_LONG_MAX_TIMEOUT       9000		// 1:30 min (according to specification)
#define FERR_TIMEOUT                    1000		// 10 s

#define MLED_IN_MAX_TIMEOUT                5		// 50 ms
#define MLED_OUT_MAX_TIMEOUT               7        // 70 ms

#define PWR_LED_SHORT_COUNT               15		// 150 ms
#define PWR_LED_LONG_COUNT                40		// 400 ms
#define PWR_LED_FERR_COUNT                10		// status led indicates >10 framing errors

/** V A R I A B L E S ********************************************************/
#pragma udata
char USB_Out_Buffer[32];

ring_generic ring_USB_datain;
ring_generic ring_USART_datain;

#pragma idata

volatile BYTE usb_timeout = 0;
	// increment every 100 us -> 100 ms timeout = 1 000
volatile WORD usart_timeout = 0;
	// increment every 100 us -> 100 ms timeout = 1 000
volatile BYTE ten_ms_counter = 0;
    // 10 ms counter

volatile BOOL usb_configured = FALSE;


/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void BlinkUSBStatus(void);
void UserInit(void);
void USART_receive(void);
void USART_send(void);
void USB_send(void);
void USB_receive(void);
BOOL USB_parse_data(BYTE start, BYTE len);
BYTE calc_xor(BYTE* data, BYTE len);
void Timer2(void);
void ProcessIO(void);
void dumpBufToUSB(ring_generic* buf);
void InitEEPROM(void);

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
			USART_send();
		}

	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
		
		// Timer2 on 100 us
		if ((PIE1bits.TMR2IE) && (PIR1bits.TMR2IF)) {
			
            if (ten_ms_counter < 100) { ten_ms_counter++; }
            else {
                ten_ms_counter = 0;
                
                // 10 ms overflow:
                
                
                // end of 10 ms counter
            }
            
			// XPRESSNET_DIR is set to XPRESSNET_IN after successful tranfer of last byte
			/*if ((usart_last_byte_sent) && (TXSTAbits.TRMT)) {
				XPRESSNET_DIR = XPRESSNET_IN;
				usart_last_byte_sent = 0;
			}*/
			
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

		#if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
		USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
						  // this function periodically.  This function will take care
						  // of processing and responding to SETUP transactions 
						  // (such as during the enumeration process when you first
						  // plug in).	USB hosts require that USB devices should accept
						  // and process SETUP packets in a timely fashion.  Therefore,
						  // when using polling, this function should be called 
						  // frequently (such as once about every 100 microseconds) at any
						  // time that a SETUP packet might reasonably be expected to
						  // be sent by the host to your device.  In most cases, the
						  // USBDeviceTasks() function does not take very long to
						  // execute (~50 instruction cycles) before it returns.
		#endif


		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.

		USART_receive();
		USB_receive();
		USB_send();

		CDCTxService();
	}//end while
}//end main

void InitializeSystem(void)
{
	#if (defined(__18CXX) & !defined(PIC18F87J50_PIM))
		ADCON1 |= 0x0F;					// Default all pins to digital
	#endif

	#if defined(USE_USB_BUS_SENSE_IO)
		tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
	#endif
	
	#if defined(USE_SELF_POWER_SENSE_IO)
		tris_self_power = INPUT_PIN;	// See HardwareProfile.h
	#endif

	UserInit();
	InitEEPROM();
	USBDeviceInit();
	USARTInit();
}

void UserInit(void)
{
	// init ring buffers
	ringBufferInit(ring_USB_datain, 32);
	ringBufferInit(ring_USART_datain, 32);
 
	// switch off AD convertors (USART is not working when not switched off manually)
	ANSEL = 0x00;
	ANSELH = 0x00;
	
	// Initialize all of the LED pins
	mInitAllLEDs();
	mLED_In_Off();
	mLED_Pwr_Off();
	mLED_Out_Off();
	
	// setup timer2 on 100 us
	T2CONbits.T2CKPS = 0b11;	// prescaler 16x
	PR2 = 75;					// setup timer period register to interrupt every 100 us
	TMR2 = 0x00;				// reset timer counter
	PIR1bits.TMR2IF = 0;		// reset overflow flag
	PIE1bits.TMR2IE = 1;		// enable timer2 interrupts
	IPR1bits.TMR2IP = 0;		// timer2 interrupt low level
	
	RCONbits.IPEN = 1;			// enable high and low priority interrupts
	//INTCONbits.PEIE = 1;		  // Enable peripheral interrupts
	INTCONbits.GIE = 1;			// enable global interrupts
	INTCONbits.GIEH = 1;
	INTCONbits.GIEL = 1;
	
	T2CONbits.TMR2ON = 1;		// enable timer2
}//end UserInit

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
	mLED_Out_On();
	ringClear(&ring_USART_datain);
	ringClear(&ring_USB_datain);
}

void USBCBWakeFromSuspend(void)
{
	usb_configured = TRUE;
	mLED_Out_Off();
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
	
}

////////////////////////////////////////////////////////////////////////////////
// Check for data in ring_USART_datain and send complete data to USB.

void USB_send(void)
{	
	// check for USB ready
	if (!mUSBUSARTIsTxTrfReady()) return;

	if (((ringLength(&ring_USART_datain)) >= 1) && (ringLength(&ring_USART_datain) >= USART_msg_len(ring_USART_datain.ptr_b))) {
		// send message
		ringSerialize(&ring_USART_datain, (BYTE*)USB_Out_Buffer, ring_USART_datain.ptr_b, USART_msg_len(ring_USART_datain.ptr_b));
		putUSBUSART(USB_Out_Buffer+1, ((USB_Out_Buffer[1])&0x0F)+2);
		ringRemoveFrame(&ring_USART_datain, ((USB_Out_Buffer[1])&0x0F)+3);
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
	
	if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

	if(mUSBUSARTIsTxTrfReady())
	{
		// ring_USB_datain overflow check
		// 2 bytes in the buffer are always reserved for the Acknowledgement
		// response.
		if (ringFreeSpace(&ring_USB_datain) < 2) {
			// delete last message
			ring_USB_datain.ptr_e = last_start;
			if (ring_USB_datain.ptr_b == ring_USB_datain.ptr_e) ring_USART_datain.empty = TRUE;
			
			// inform PC about full buffer
			//respondBufferFull();
			
			return;
		}
		
		received_len = getsUSBUSART(&ring_USB_datain, ringFreeSpace(&ring_USB_datain));
		if (received_len == 0) {
			// check for timeout
			if ((usb_timeout >= USB_MAX_TIMEOUT) && (last_start != ring_USB_datain.ptr_e)) {
				ring_USB_datain.ptr_e = last_start;
				usb_timeout = 0;
				if (ring_USB_datain.ptr_e == ring_USB_datain.ptr_b) ring_USB_datain.empty = TRUE;
				
				// inform PC about timeout
				USB_Out_Buffer[0] = 0x01;
				USB_Out_Buffer[1] = 0x01;
				USB_Out_Buffer[2] = 0x00;
				if (mUSBUSARTIsTxTrfReady()) putUSBUSART(USB_Out_Buffer, 3);
			}
			return;
		}
		usb_timeout = 0;
				
		// data received -> parse data
		while ((ringDistance(&ring_USB_datain, last_start, ring_USB_datain.ptr_e) > 0) &&
				(USB_last_message_len >= USB_msg_len(last_start))) {
			
			// whole message received -> check for xor
			for (i = 0, xor = 0; i < USB_msg_len(last_start)-1; i++)
				xor ^= ring_USB_datain.data[(i+last_start) & ring_USB_datain.max];
			
			if (xor != ring_USB_datain.data[(i+last_start) & ring_USB_datain.max]) {
				// xor error
				// here, we need to delete content in the middle of ring buffer
				ringRemoveFromMiddle(&ring_USB_datain, last_start, USB_msg_len(last_start));
				//respondXORerror();
				return;
			}
			
			// xor ok -> parse data
			if (USB_parse_data(last_start, USB_msg_len(last_start))) {
				// timeslot not available -> respond "Buffer full"
				/*if (timeslot_err) {
					ringRemoveFromMiddle(&ring_USB_datain, last_start, USB_msg_len(last_start));
					respondBufferFull();
					return;
				}*/
				
				// data waiting for sending to xpressnet -> move last_start
				last_start = (last_start+USB_msg_len(last_start))&ring_USB_datain.max;
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
/* Parse data from USB
 * \start and \len reference to ring_USB_datain.
 * \len is WITH header byte and WITH xor byte
 * Returns:
 *	TRUE if data is waiting for sending to xpressnet
 *	FALSE if data were processed
 * Achtung: you must delete data from ring_USB_datain if data were processed.
 * (otherwise the program will freeze)
 */ 

BOOL USB_parse_data(BYTE start, BYTE len)
{
    // TODO
	return TRUE;
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
void USART_send(void)
{
	static BYTE head = 0, id = 0;
	
	// according to specification, ninth bit is always 0
	/*USARTWriteByte(0, ring_USB_datain.data[usart_to_send]);
	usart_to_send = (usart_to_send+1)&ring_USB_datain.max;
	
	if (usart_to_send == ((ring_USB_datain.ptr_b + USB_msg_len(ring_USB_datain.ptr_b))&ring_USB_datain.max)) {
		// last byte sending
		head = ring_USB_datain.data[ring_USB_datain.ptr_b];
		id = ring_USB_datain.data[(ring_USB_datain.ptr_b+1)&ring_USB_datain.max];
		
		ring_USB_datain.ptr_b = usart_to_send;  // whole message sent
		if (ring_USB_datain.ptr_b == ring_USB_datain.ptr_e) ring_USB_datain.empty = TRUE;
		
		checkResponseToPC(head, id); // send OK response to PC
		
		PIE1bits.TXIE = 0;
		usart_last_byte_sent = 1;
        mLED_Out_Timeout = MLED_OUT_MAX_TIMEOUT;    // we want to switch LED off after timeout
	} else {
		// other-than-last byte sending
		PIE1bits.TXIE = 1;
        mLED_Out_Timeout = 0;   // we do not want to switch the LED off
	}*/
}

////////////////////////////////////////////////////////////////////////////////
// Debug function: dump eing buffer to USB
void dumpBufToUSB(ring_generic* buf)
{
	int i;
	for (i = 0; i <= buf->max; i++) USB_Out_Buffer[i] = buf->data[i];
	putUSBUSART(USB_Out_Buffer, buf->max+1);
}

////////////////////////////////////////////////////////////////////////////////

void InitEEPROM(void)
{
    // TODO: read data from EEPROM at boot
}

////////////////////////////////////////////////////////////////////////////////
