# uLI-master

Ultimate LI – master is a XpressNET ↔ USB (CDC) inteface. It bahaves as a
XpressNET master. In this scheme, computer basically replaces the command station.

* Processor: PIC18F14K50
* Programming language: C
* PCB: [uLI-04](https://github.com/kmzbrnoI/uLI-pcb)
* Authors: Jan Horacek (c) 2016
* License: Apache License v2

## Used tools

- MPLAB X IDE v3.45
- C18 LITE v3.47 compiler (version for linux available
  [here](https://github.com/Manouchehri/Microchip-C18-Lite))
- clang-format to format code

## Windows driver

Windows CDC driver could be found in the
[Microchip Libraries for Applications](http://www.microchip.com/mplab/microchip-libraries-for-applications).

However you do not need to download ~ 280 MB of data to get ~ 10 kB driver. We
added custom driver (based on the Microchip driver) to the
[driver_win](driver_win/) directory of this repo.

## COM port specification

* Any speed.
* No flow control.

## EEPROM

EEPROM is not used in this firmware.

## Programming

We use `PICPgm` at RaspberryPi to program the PIC.

Note: When programming the processor for first time, do not forget to include
`-p_cfg` argument to program fuses into the processor. Fuses are stored in main
hex file.

## LEDs

### Input LED (green)
This LED is turned on by default. It turns off for a few miliseconds when a
byte arrives from a XpressNET device to Master.

### Output LED (green)
This LED is turned on before a valid connection with PC is established. After
establishing the connection, this LED turns off and blinks only when a command
is being received from PC.

### Status LED (yellow)
- 2 blinks = normal operations

## Protocols

 * On XpressNET side, Master uses standard XpressNET protocol.
 * On PC side, Master uses [custom protocol](cdc-protocol.md).

## Further reading

- [About XpressNET](http://www.opendcc.de/info/xpressnet/xpressnet_e.html)
