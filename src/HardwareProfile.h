#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

#include "main.h"

/*******************************************************************/
/******** USB stack hardware selection options *********************/
/*******************************************************************/
//This section is the set of definitions required by the MCHPFSUSB
//  framework.  These definitions tell the firmware what mode it is
//  running in, and where it can find the results to some information
//  that the stack needs.
//These definitions are required by every application developed with
//  this revision of the MCHPFSUSB framework.  Please review each
//  option carefully and determine which options are desired/required
//  for your application.

//#define USE_SELF_POWER_SENSE_IO	
#define tris_self_power     TRISAbits.TRISA2    // Input
#if defined(USE_SELF_POWER_SENSE_IO)
#define self_power          PORTAbits.RA2
#else
#define self_power          1
#endif

//#define USE_USB_BUS_SENSE_IO
#define tris_usb_bus_sense  TRISAbits.TRISA1    // Input
#if defined(USE_USB_BUS_SENSE_IO)
#define USB_BUS_SENSE       PORTAbits.RA1
#else
#define USB_BUS_SENSE       1
#endif

/*******************************************************************/
/*******************************************************************/
/*******************************************************************/
/******** Application specific definitions *************************/
/*******************************************************************/
/*******************************************************************/
/*******************************************************************/

//Uncomment the following line to make the output HEX of this 
//  project work with the HID Bootloader
//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER	

/** Board definition ***********************************************/

#define IO_OUT(TRIS, MASK) { TRIS &= ~MASK; }
#define IO_IN(TRIS, MASK) { TRIS |= MASK; }

/** LEDs ***********************************************************/

#define mLED_In_LAT          LATCbits.LC0
#define mLED_In_On()         { mLED_In_LAT = 1; }
#define mLED_In_Off()        { mLED_In_LAT = 0; }
#define mLED_In_Toggle()     { mLED_In_LAT = !mLED_In_LAT; }

#define mLED_Out_LAT         LATCbits.LC1
#define mLED_Out_On()        { mLED_Out_LAT = 1; }
#define mLED_Out_Off()       { mLED_Out_LAT = 0; }
#define mLED_Out_Toggle()    { mLED_Out_LAT = !mLED_Out_LAT; }

#define mLED_PWR_LAT         LATCbits.LC2
#define mLED_Pwr_On()        { mLED_PWR_LAT = 1; }
#define mLED_Pwr_Off()       { mLED_PWR_LAT = 0; }
#define mLED_Pwr_Toggle()    { mLED_PWR_LAT = !mLED_PWR_LAT; }

/** IO ************************************************************/

#define IO_XNPWR_LAT         LATCbits.LC4

static inline void IO_XNPWR_set(bool value) {
    IO_XNPWR_LAT = (value ^ (version_hw == VERSION_HW_5));
}

static inline bool IO_XNPWR_get(void) {
    return (IO_XNPWR_LAT ^ (version_hw == VERSION_HW_5));
}

#define mSense               (!PORTBbits.RB4)

#define IO_HW_VERSION_PORT   PORTCbits.RC7
#define IO_HW_VERSION_TRIS   TRISC
#define IO_HW_VERSION_MASK   0x80

static inline void IO_init(void) {
    TRISBbits.TRISB4 = 1;
    
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC4 = 0;

    IO_XNPWR_set(false);
  	mLED_Pwr_On();
	mLED_In_On();
	mLED_Out_On();
}

#endif  //HARDWARE_PROFILE_H
