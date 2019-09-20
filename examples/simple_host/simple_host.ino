/**
 *  Modbus host example 1:
 *  The purpose of this example is to query an array of data
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

// data array for modbus network sharing
uint16_t au16data[16];
uint8_t u8state;
uint8_t u8addr;
uint8_t u8lastaddr;

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for host, = 1..247 for device
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus host(0, A4); // this is host and RS-232 or USB-FTDI
ModbusSerial<decltype(Serial1)> mySerial(&Serial1);

#define kPowerOn        A3

static inline void powerOn(void)
{
        pinMode(kPowerOn, OUTPUT);
        digitalWrite(kPowerOn, HIGH);
}

/**
 * This is a struct which contains a query to a device
 */
modbus_t datagram;

unsigned long u32wait;

void setup() {
  powerOn();
  host.begin(&mySerial, 19200); // baud-rate at 19200
  host.setTimeOut( 1000 ); // if there is no answer in 1000 ms, roll over
  host.setTxEnableDelay(100);
  u32wait = millis() + 1000;
  u8state = 0; 
  u8addr = 1;
  u8lastaddr = 2;
}

void loop() {
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
    host.poll(); // check incoming messages
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

