/*

Module:  catena_simple_host.ino

Function:
        simple_host.ino example converted to use Catena-Arduino-Platform
        framework, and pollable interface.

Copyright notice and License:
        See LICENSE file accompanying this project.

Author:
        Terry Moore, MCCI Corporation   July 2018

*/

// to compile this, you must install the Catena-Arduino-Platform library:
// see https://github.com/mcci-catena/Catena-Arduino-Platform

#include <Catena.h>
#include <Catena_ModbusRtu.h>

using namespace McciCatena;

// the framework object.
Catena gCatena;

// data array for modbus network sharing
uint16_t au16data[16];
uint8_t u8state;
uint8_t u8addr;
uint8_t u8lastaddr;

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for host, = 1..247 for device
 *         In this case, we're the host.
 *  u8txenpin : 0 for RS-232 and USB-FTDI
 *               or any pin number > 1 for RS-485
 *
 *  We also need a serial port object, based on a UART;
 *  we default to serial port 1.  Since the type of Serial1
 *  varies from platform to platform, we use decltype() to
 *  drive the template based on the type of Serial1,
 *  eliminating a complex and never-exhaustive series of
 *  #ifs.
 */
cCatenaModbusRtu host(0, A4); // this is host and RS-232 or USB-FTDI
ModbusSerial<decltype(Serial1)> mySerial(&Serial1);

#define kPowerOn        A3

static inline void powerOn(void)
{
        pinMode(kPowerOn, OUTPUT);
        digitalWrite(kPowerOn, HIGH);
}

/**
 * This is a struct which contains a message to a device
 */
modbus_datagram_t datagram;

unsigned long u32wait;

void setup() {
  gCatena.begin();                      // set up the framework.

  powerOn();                            // turn on the transceiver.
  host.begin(&mySerial, 19200);         // baud-rate at 19200
  host.setTimeOut( 1000 );              // if there is no answer in 1000 ms, roll over
  host.setTxEnableDelay(100);           // wait 100ms before each tx
  gCatena.registerObject(&host);        // enroll the host object in the poll list.

  // set up the fsm
  u32wait = millis() + 1000;            // when to exit state 0
  u8state = 0;                          // current state: state 0
  u8addr = 1;                           // current target device: 1
  u8lastaddr = 2;                       // last device to scan before starting over at 1.
}

void loop() {
  gCatena.poll(); // check incoming messages & drive things along.

  switch( u8state ) {
  case 0:
    if (long(millis() - u32wait) > 0) u8state++; // wait state
    break;
  case 1:
    datagram.u8id = u8addr; // device address
    datagram.u8fct = 3; // function code (this one is registers read)
    datagram.u16RegAdd = 1700; // start address in device
    datagram.u16CoilsNo = 4; // number of elements (coils or registers) to read
    datagram.au16reg = au16data; // pointer to a memory array in the Arduino

    host.setLastError(ERR_SUCCESS);
    host.query( datagram ); // send query (only once)
    u8state++;
    break;
  case 2:
    if (host.getState() == COM_IDLE) {
      u8state = 0;
      ERR_LIST lastError = host.getLastError();
      Serial.print(millis());
      Serial.print(": addr=");
      Serial.print(u8addr, 16);

      if (host.getLastError() != ERR_SUCCESS) {
	Serial.print(": Error ");
	Serial.print(int(lastError));
      } else {
        Serial.print(": Registers: ");
        for (int i=0; i < 4; ++i)
          {
          Serial.print(" ");
          Serial.print(au16data[i], 16);
          }
      }
      Serial.println("");

      // move to next
      if (u8addr == u8lastaddr)
        u8addr = 1;
      else
        ++u8addr;

      u32wait = millis() + 100;
    }
    break;
  }
}

