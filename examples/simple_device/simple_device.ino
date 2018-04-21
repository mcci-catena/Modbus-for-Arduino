/**
 *  Modbus device example 1:
 *  The purpose of this example is to link a data array
 *  from the Arduino to an external device.
 *
 *  Recommended Modbus host: QModbus
 *  http://qmodbus.sourceforge.net/
 */

#include <ModbusRtu.h>

// data array for modbus network sharing
uint16_t au16data[16] = {
  3, 1415, 9265, 4, 2, 7182, 28182, 8, 0, 0, 0, 0, 0, 0, 1, -1 };

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for host, = 1..247 for device
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
ModbusSerial<decltype(Serial1)> mySerial(&Serial1);

Modbus device(1, 0); // this is device @1 and RS-232 or USB-FTDI

void setup() {
  device.begin( &mySerial, 19200 ); // baud-rate at 19200
}

void loop() {
  device.poll( au16data, 16 );
}
