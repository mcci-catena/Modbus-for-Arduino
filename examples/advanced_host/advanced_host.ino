/**
 *  Modbus host example 2:
 *  The purpose of this example is to query several sets of data
 *  from an external Modbus device. 
 *  The link media can be USB or RS232.
 *
 *  Recommended Modbus device: 
 *  diagslave http://www.modbusdriver.com/diagslave.html
 *
 *  In a Linux box, run 
 *  "./diagslave /dev/ttyUSB0 -b 19200 -d 8 -s 1 -p none -m rtu -a 1"
 * 	This is:
 * 		serial port /dev/ttyUSB0 at 19200 baud 8N1
 *		RTU mode and address @1
 */

#include <ModbusRtu.h>

uint16_t au16data[16]; //!< data array for modbus network sharing
uint8_t u8state; //!< machine state
uint8_t u8query; //!< pointer to message query

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for host, = 1..247 for device
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
ModbusSerial<decltype(Serial1)> mySerial(&Serial1);
Modbus host(0, 0); // this is host and RS-232 or USB-FTDI

/**
 * This is a struct which contains a query to a device
 */
modbus_t datagram[2];

unsigned long u32wait;

void setup() {
  // datagram 0: read registers
  datagram[0].u8id = 1; // device address
  datagram[0].u8fct = 3; // function code (this one is registers read)
  datagram[0].u16RegAdd = 0; // start address in device
  datagram[0].u16CoilsNo = 4; // number of elements (coils or registers) to read
  datagram[0].au16reg = au16data; // pointer to a memory array in the Arduino

  // datagram 1: write a single register
  datagram[1].u8id = 1; // device address
  datagram[1].u8fct = 6; // function code (this one is write a single register)
  datagram[1].u16RegAdd = 4; // start address in device
  datagram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
  datagram[1].au16reg = au16data+4; // pointer to a memory array in the Arduino
	
  host.begin(&mySerial, 19200 ); // baud-rate at 19200
  host.setTimeOut( 5000 ); // if there is no answer in 5000 ms, roll over
  u32wait = millis() + 1000;
  u8state = u8query = 0; 
}

void loop() {
  switch( u8state ) {
  case 0: 
    if (millis() > u32wait) u8state++; // wait state
    break;
  case 1: 
    host.query( datagram[u8query] ); // send query (only once)
    u8state++;
	u8query++;
	if (u8query > 2) u8query = 0;
    break;
  case 2:
    host.poll(); // check incoming messages
    if (host.getState() == COM_IDLE) {
      u8state = 0;
      u32wait = millis() + 1000; 
    }
    break;
  }

  au16data[4] = analogRead( 0 );
  
}

