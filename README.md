# Modbus for Arduino

This library provides a Modbus implementation for Arduino.

The library enables industrial communication for the Arduino by linking it to industrial devices such as HMIs, CNCs, PLCs, temperature regulators or speed drives.

The library supports software serial as well as hardware serial. The initial changes from Helium6072 are generalized so that you can use any object with Serial semantics.

[![GitHub release](https://img.shields.io/github/release/mcci-catena/Modbus-for-Arduino/all.svg)](https://github.com/mcci-catena/Modbus-for-Arduino/releases/latest) ![GitHub commits](https://img.shields.io/github/commits-since/mcci-catena/Modbus-for-Arduino/latest.svg)

**Contents:**

<!--
  This TOC uses the VS Code markdown TOC extension AlanWalk.markdown-toc.
  We strongly recommend updating using VS Code, the markdown-toc extension and the
  bierner.markdown-preview-github-styles extension.  Note that if you are using
  VS Code 1.29 and Markdown TOC 1.5.6, https://github.com/AlanWalk/markdown-toc/issues/65
  applies -- you must change your line-ending to some non-auto value in Settings>
  Text Editor>Files.  `\n` works for me.
-->

<!-- markdownlint-disable MD033 MD004 -->
<!-- markdownlint-capture -->
<!-- markdownlint-disable -->
<!-- TOC depthFrom:2 updateOnSave:true -->

- [Terminology](#terminology)
- [Library Contents](#library-contents)
	- [Files](#files)
	- [examples/advanced_device](#examplesadvanced_device)
	- [examples/RS485_device](#examplesrs485_device)
	- [examples/simple_host](#examplessimple_host)
	- [examples/simple_device](#examplessimple_device)
	- [examples/software_serial_simple_host](#examplessoftware_serial_simple_host)
	- [examples/catena_simple_host](#examplescatena_simple_host)
- [Installation Procedure](#installation-procedure)
- [Known Issues](#known-issues)
- [To-Do List](#to-do-list)
- [Serial port, baud rate, configuration](#serial-port-baud-rate-configuration)
- [The `ModbusSerial<>` Template Class](#the-modbusserial-template-class)
- [Legacy names](#legacy-names)
- [Queueing datagrams (host only)](#queueing-datagrams-host-only)
- [Meta](#meta)
	- [License](#license)
	- [Contributors](#contributors)
	- [Support Open Source Hardware and Software](#support-open-source-hardware-and-software)
	- [Trademarks](#trademarks)

<!-- /TOC -->
<!-- markdownlint-restore -->
<!-- Due to a bug in Markdown TOC, the table is formatted incorrectly if tab indentation is set other than 4. Due to another bug, this comment must be *after* the TOC entry. -->

## Terminology

Modbus literature uses "host" and "slave". Time has moved on, so in this library, we use the terms "initiator" and "responder".

## Library Contents

### Files

This list is not exhaustive, but highlights some of the more important files.

File | Description
-----|------------
`LICENSE.txt` | GNU License file
`keywords.txt` | Arduino IDE coloring syntax
`documentation/*` | Library documentation generated with Doxygen.
`examples/*` | Sample sketches to implement miscellaneous settings.
`src/ModbusRtuV2.h` | The library header file
`src/ModbusRtu.h` | A wrapper header file that declares additional names for compatibility with older versions of the library.
`src/Catena_ModbusRtuHost.h` | Slightly higher abstraction layer for Modbus host use. Includes ability to queue datagrams for sequential processing, with callbacks on completion. Integrates with the Catena polling system.
`src/lib/ModbusRtu.cpp` | The main source file for the library.

### examples/advanced_device

Modbus device node, which links Arduino pins to the Modbus port.

### examples/RS485_device

Modbus device adapted to the RS485 port.

### examples/simple_host

Modbus host node with a single query.

### examples/simple_device

Modbus device node with a link array.

### examples/software_serial_simple_host

Modbus host node that works via software serial.

### examples/catena_simple_host

Modbus host node implementation demonstrating the use of the Catena polling system.

## Installation Procedure

Refer to this documentation to install this library:

[`arduino.cc/en/Guide/Libraries`](https://arduino.cc/en/Guide/Libraries)

Starting with IDE version 1.0.5, you can install 3rd party libraries in the IDE.

Do not unzip the downloaded library, leave it as is.

In the Arduino IDE, navigate to Sketch > Import Library. At the top of the drop down list, select the option to "Add Library".

You will be prompted to select this zipped library.

Return to the Sketch > Import Library menu. You should now see the library at the bottom of the drop-down menu. It is ready to be used in your sketch.

The zip file will have been expanded in the libraries folder in your Arduino sketches directory.

Note: the library will be available to use in sketches, but examples for the library will not be exposed in the `File > Examples` until after the IDE has restarted.

## Known Issues

The original library was not compatible with ARDUINO LEONARDO. This library has not been tested under ARDUINO DUE and newer boards. It has been tested primarily with Adafruit Feather M0 boards.

## To-Do List

Common to host and device:

1. End frame delay, also known as T35

2. Test it with several Arduino boards: UNO, Mega, etc..

3. Extend it to Leonardo

Host:

1. Function code 1 and 2 still not implemented

2. Function code 15 still not implemented

3. Other codes under development

## Serial port, baud rate, configuration

The `Modbus::begin()` method, in its complete form, takes three arguments:

- `ModbusPort *pPort`: this object maps from  abstract serial port semantics onto a concrete serial port.

- `unsigned long baudRate`: specifies the baud rate in bits per second.

- `uint16_t config`: specifies the configuration of the serial port. Normally this should be `SERIAL_8N2`(8 data bits, no parity, two stop bits), but other settings may be chosen. NB: `SoftwareSerial` doesn't yet honor these settings.

## The `ModbusSerial<>` Template Class

Earlier versions of this library did not compile with `SoftwareSerial`, nor with `USBSerial` ports. The error was because a polymorphic pointer is needed at the top level to the "port" -- although Serial, UART, SoftwareSerial share a common interface, there is no common abstract class in the standard Arduino library; and their methods aren't virtual.

This variant of the library introduces the `ModbusPort` class, which has all the proper abstract semantics, and the `ModbusSerial<T>` template class, which maps the abstract semantics of `ModbusPort` onto the concrete semantics of `T` (whatever `T` happens to be; provided that `T` conforms to the Serial API).

Declaring a Modbus instance takes two steps (which can be done in any order).

1. Declare a `Modbus` instance that represents the host or device:

   ```c++
   Modbus host(0, kTxPin);
   ```

   This declares a variable named `host`, which represents a Modbus controller. `kTxPin`, if non-zero, specifies the pin to be used to enable/disable TX.

2. Declare an object derived from `ModbusPort` to serve as the interface to the serial port. The easy way to do this is using the `ModbusSerial<>` template type.

   ```c++
   // declare modbusSerial as a ModbusSerial<> object
   // and map it onto Serial1:
   ModbusSerial<decltype(Serial1)> modbusSerial(&Serial1);
   ```

   Of course, if you know that the type of `Serial` is `UART`, you can also write:

   ```c++
   // declare modbusSerial as a ModbusSerial<> object
   // and map it onto Serial1:
   ModbusSerial<UART> modbusSerial(&Serial1);
   ```

   We tend to prefer the former form, as it makes the examples more portable.

## Legacy names

For compatibility with earlier versions of this library, we supply the header file `ModbusRtu.h`.  This file includes `ModbusRtuV2.h`, and then exposes a number of names and types in the root namespace.

Similarly, the legacy header file `Catena_ModbusRtu.h` calls `Catena_ModbusRtuHost.h` and defines a few names for backward compatibility. It also exposes all the same names as `ModbusRtu.h`.

## Queueing datagrams (host only)

Starting with v0.4.0, this library supports queued datagram operations with callbacks. To take advantage of this, declare your datagrams as `McciCatena::cModbusDatagram` objects (rather than `modbus_datagram_t` objects).  Create one or more callback functions of type `McciCatena::CatenaModbusRtu_DatagramCb_t`.
Then use `cCatenaModbusRtuHost::queue()` to submit your datagrams; they'll be processed in FIFO order.

## Meta

### License

This repository is released under the [LGPL 2.1](./LICENSE.md) license.

### Contributors

Samuel Marco i Armengol wrote the original [Modbus for Arduino]https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino) library. Terry Moore refactored and made various minor functional changes for portability, compatibility with more Serial-like objects, and type safety.

### Support Open Source Hardware and Software

MCCI invests time and resources providing this open source code, please support MCCI and open-source hardware by purchasing products from MCCI, Adafruit and other open-source hardware/software vendors!

For information about MCCI's products, please visit [mcci.com](https://mcci.com/) and [store.mcci.com](https://store.mcci.com/).

### Trademarks

MCCI and MCCI Catena are registered trademarks of MCCI Corporation. LoRaWAN is a registered trademark of the LoRa Alliance. LoRa is a registered trademark of Semtech Corporation. All other marks are the property of their respective owners.
