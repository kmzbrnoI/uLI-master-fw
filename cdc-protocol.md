# uLI-master ↔ PC communication protocol

## Overview

This protocol allows PC to send and receive:
 * messages for (from) any XpressNET device,
 * broadcast messages and
 * messages for (from) Master.

To achieve this functionality, data are sent (received) from (to) computer
with modified call byte included.

Standard Master ↔ PC message consists of:

|   Call byte   |  Header byte  |  Data byte 1  |  Data byte 2  | ... |  Data byte n  | XOR |
|---------------|---------------|---------------|---------------|-----|---------------|-----|
| `0bPDDA AAAA` | `0bTTTT LLLL` | `0bDDDD DDDD` | `0bDDDD DDDD` | ... | `0bDDDD DDDD` | XOR |

## Call byte:

 - `P` : parity (even)
 - `DD` : type of message:
   * `0b11` : command for XpressNET device
   * `0b01` : command for Master
 - `AAAAA`: address of XpressNET device. When `DD` = `0b01`, `AAAAA` can be
   anything.
   `AAAAA` = `0b00000` = broadcast to all XpressNET devices.

## Header byte

 - `TTTT` : type of message according to XpressNET specification
 - `LLLL` : length of *Data byte 1 .. Data byte n* = n

## XOR

is XOR of *Header byte, Data byte 1 .. Data byte n*.

## Master → PC commands

| Call byte | Header byte | Data byte 1 | Data 2 | Data 3 | Data 4 |                 Meaning                  |
|-----------|-------------|-------------|--------|--------|--------|------------------------------------------|
| 0xA0      | 0x01        | 0x01        | XOR    | -      | -      | USB incoming data timeout                |
| 0xA0      | 0x01        | 0x02        | XOR    | -      | -      | USART incoming data timeout              |
| 0xA0      | 0x01        | 0x03        | XOR    | -      | -      | Unknown command                          |
| 0xA0      | 0x01        | 0x04        | XOR    | -      | -      | OK                                       |
| 0xA0      | 0x01        | 0x05        | XOR    | -      | -      | Keep-alive                               |
| 0xA0      | 0x01        | 0x06        | XOR    | -      | -      | USB→USART buffer overflow                |
| 0xA0      | 0x01        | 0x07        | XOR    | -      | -      | USB XOR error                            |
| 0xA0      | 0x01        | 0x08        | XOR    | -      | -      | USB parity error                         |
| 0xA0      | 0x01        | 0x09        | XOR    | -      | -      | XpressNET power source turned off        |
| 0xA0      | 0x01        | 0x0A        | XOR    | -      | -      | XpressNET power transistor closed        |
|-----------|-------------|-------------|--------|--------|--------|------------------------------------------|
| 0xA0      | 0x11        | 0xA 0b00ST  | XOR    | -      | -      | Master status (S = sense, T = transistor)|
| 0xA0      | 0x13        | 0x80        | 0xHW   | 0xSW   | XOR    | Master SW and HW version response        |

## PC → Master commands

| Call byte | Header byte | Data byte 1 | Data 2 | Data 3 | Data 4 |                 Meaning                  |
|-----------|-------------|-------------|--------|--------|--------|------------------------------------------|
| 0xA0      | 0x01        | 0x05        | XOR    | -      | -      | Keep-alive                               |
| 0xA0      | 0x11        | 0xA 0b000T  | XOR    | -      | -      | Set transistor to T                      |
| 0xA0      | 0x11        | 0xA2        | XOR    | -      | -      | Transistor status request                |
| 0xA0      | 0x11        | 0xB2        | XOR    | -      | -      | Sense status request                     |
| 0xA0      | 0x11        | 0x80        | XOR    | -      | -      | Master`s version request                 |
| 0xA0      | 0x11        | 0x81        | XOR    | -      | -      | Response request                         |
