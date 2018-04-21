# Modbus for Aruduino

This library provides a Serial Modbus implementation for Arduino.

A primary goal was to enable industrial communication for the Arduino in order to link it to industrial devices such as HMIs, CNCs, PLCs, temperature regulators or speed drives.

It supports software serial as well as hardware serial. The initial changes from Helium6072 are generalized so that you can use any object with Serial semantics.

[![GitHub release](https://img.shields.io/github/release/mcci-catena/Modbus-for-Arduino.svg)](https://github.com/mcci-catena/Modbus-for-Arudino/releases/latest) [![GitHub commits](https://img.shields.io/github/commits-since/mcci-catena/Modbus-for-Arduino/latest.svg)](https://github.com/mcci-catena/Modbus-for-Arduino/compare/v0.2.0-beta1...master)

**Contents:**

<!--
  This TOC uses the VS Code markdown TOC extension AlanWalk.markdown-toc.
  We strongly recommend updating using VS Code, the markdown-toc extension and the
  bierner.markdown-preview-github-styles extension.
-->

<!-- TOC depthFrom:2 updateOnSave:true -->

- [Terminology](#terminology)
- [Library Contents](#library-contents)
	- [Examples](#examples)
		- [examples/advanced_device](#examplesadvanced_device)
		- [examples/RS485_device](#examplesrs485_device)
		- [examples/simple_host](#examplessimple_host)
		- [examples/simple_device](#examplessimple_device)
		- [examples/software_serial_simple_host](#examplessoftware_serial_simple_host)
- [Installation Procedure](#installation-procedure)
- [Known Issues](#known-issues)
- [To-Do List](#to-do-list)
- [Parity, stop bits, etc](#parity-stop-bits-etc)
- [The `ModbusSerial<>` Template Class](#the-modbusserial-template-class)

<!-- /TOC -->

## Terminology

Modbus literature uses "host" and "slave". Time has moved on, so in this library, we use the terms "initiator" and "responder".

## Library Contents

File | Description
-----|------------
ModbusRTU.h | The library
LICENSE.txt | GNU Licence file
keywords.txt | Arduino IDE colouring syntax
documentation/* | Library documentation generated with Doxygen.
examples/* | Sample sketches to implement miscellaneous settings.

### Examples

#### examples/advanced_device

Modbus device node, which links Arduino pins to the Modbus port.

#### examples/RS485_device

Modbus device adapted to the RS485 port.

#### examples/simple_host

Modbus host node with a single query.

#### examples/simple_device

Modbus device node with a link array.

#### examples/software_serial_simple_host

Modbus host node that works via software serial.

## Installation Procedure

Refer to this documentation to install this library:

http://arduino.cc/en/Guide/Libraries

Starting with version 1.0.5, you can install 3rd party libraries in the IDE.

Do not unzip the downloaded library, leave it as is.

In the Arduino IDE, navigate to Sketch > Import Library. At the top of the drop down list, select the option to "Add Library".

You will be prompted to select this zipped library.

Return to the Sketch > Import Library menu. You should now see the library at the bottom of the drop-down menu. It is ready to be used in your sketch.

The zip file will have been expanded in the libraries folder in your Arduino sketches directory.

NB : the library will be available to use in sketches, but examples for the library will not be exposed in the File > Examples until after the IDE has restarted.

## Known Issues

The original library was not compatible with ARDUINO LEONARDO. This library has not been tested under ARDUINO DUE and newer boards. It has been tested primarily with Adafruit Feather M0 boards.

## To-Do List

Common to host and device:

1) End frame delay, also known as T35

2) Test it with several Arduino boards: UNO, Mega, etc..

3) Extend it to Leonardo

Host:

1) Function code 1 and 2 still not implemented

2) Function code 15 still not implemented

3) Other codes under development

## Parity, stop bits, etc

The `Modbus::begin()` method, in its complete form, takes three arguments:

- `ModbusPort *pPort`: this object maps from  abstract serial port semantics onto a concrete serial port.

- `unsigned long baudRate`: specifies the baudrate in bits per second.

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
