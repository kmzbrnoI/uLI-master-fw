# uLI-master
LI boards behaving as XpressNET master. It allows PC to behave as command
station.

* Processor: PIC18F14K50
* Programming language: C
* PCB: [uLI](https://github.com/kmzbrnoI/uLI-pcb)
* Authors: Jan Horacek (c) 2016

## Used tools

- MPLAB X IDE v3.15 for Windows
- C18 LITE v3.47 compiler for Windows

## Programming

We used `PICPgm` at RaspberryPi to program the PIC.

Note: When programming the processor for first time, do not forget to include
`-p_cfg` argument to program fuses into the processor. Fuses are stored in main
hex file.

## Further reading

- [About XpressNET](http://www.opendcc.de/info/xpressnet/xpressnet_e.html)
