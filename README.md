# uLI-master

Ultimate LI – master is a XpressNET ↔ USB (CDC) inteface. It bahaves as a
XpressNET master. In this scheme, computer basically replaces the command station.

* Processor: PIC18F14K50
* Programming language: C
* PCB: [Ultimate LI](https://github.com/kmzbrnoI/uLI-pcb)
* Author: Jan Malina (ex Horacek)
* License: Apache License v2

## Used tools

- MPLAB X IDE v6.20
- XC8 v3.00 compiler
- clang-format to format code

## Windows driver

You can find Windows CDC driver at
[Microchip Libraries for Applications](http://www.microchip.com/mplab/microchip-libraries-for-applications).

However you do not need to download ~280 MB of data to get ~10 kB driver. We
provide custom driver based on the Microchip driver in the
[driver_win](driver_win/) directory of this repo.

## COM port specification

* Any speed.
* **No flow control**.

## EEPROM

EEPROM is not used in this firmware.

## Programming

Just use any programmer compatible with MPLAB – e.g. PICKit.

## LEDs

### Input LED (green)
Input LED is turned on by default. It turns off for a few miliseconds when a
byte arrives from a XpressNET device.

### Output LED (green)
Output LED is turned on when connection with PC is not established. After
establishment of the connection, the LED turns off. It blinks when a command
is received from PC.

### Status LED (yellow)
- 2 blinks = normal operations

## `RACK_ENABLE`

When code is compiled with `RACK_ENABLE` macro, uLI-master will periodically
ask devices to respond via *Request for Acknowledgement* command. uLI-mastter
uses this command to keep list of active XpressNET devices. Changes in active
devices list are reported to PC, see protocol.

## Protocols

 * On XpressNET side, Master uses standard XpressNET protocol.
 * On PC side, Master uses [custom protocol](cdc-protocol.md).

## Further reading

- [About XpressNET](http://www.opendcc.de/info/xpressnet/xpressnet_e.html)
