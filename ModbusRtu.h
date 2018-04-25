/**
 * @file 	ModbusRtu.h
 * @version     1.21
 * @date        2016.02.21
 * @author 	Samuel Marco i Armengol
 * @contact     sammarcoarmengol@gmail.com
 * @contribution Helium6072
 *
 * @description
 *  Arduino library for communicating with Modbus devices
 *  over RS232/USB/485 via RTU protocol.
 *
 *  Further information:
 *  http://modbus.org/
 *  http://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
 *
 * @license
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; version
 *  2.1 of the License.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * @defgroup setup Modbus Object Instantiation/Initialization
 * @defgroup loop Modbus Object Management
 * @defgroup buffer Modbus Buffer Management
 * @defgroup discrete Modbus Function Codes for Discrete Coils/Inputs
 * @defgroup register Modbus Function Codes for Holding/Input Registers
 *
 */

#include <inttypes.h>
#include "Arduino.h"
#include "Print.h"
#include "ModbusPort.h"

/**
 * @struct modbus_t
 * @brief
 * host query structure:
 * This includes all the necessary fields to make the host generate a Modbus query.
 * A host may keep several of these structures and send them cyclically or
 * use them according to program needs.
 */
typedef struct
{
    uint8_t u8id;          /*!< device address between 1 and 247. 0 means broadcast */
    uint8_t u8fct;         /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
    uint16_t u16RegAdd;    /*!< Address of the first register to access at device/s */
    uint16_t u16CoilsNo;   /*!< Number of coils or registers to access */
    uint16_t *au16reg;     /*!< Pointer to memory image in host */
}
modbus_t;

enum
{
    RESPONSE_SIZE = 6,
    EXCEPTION_SIZE = 3,
    CHECKSUM_SIZE = 2
};

/**
 * @enum MESSAGE
 * @brief
 * Indexes to telegram frame positions
 */
enum MESSAGE
{
    ID = 0, //!< Index of ID field
    FUNC, //!< Index of Function code
    ADD_HI, //!< Index of Address high byte
    ADD_LO, //!< Index of Address low byte
    NB_HI, //!< Index of Number of coils or registers high byte
    NB_LO, //!< Index of Number of coils or registers low byte
    BYTE_CNT,  //!< Index of byte counter

    EXCEPTION = FUNC + 1, //!< Index in exception response of exception code.
};

/**
 * @enum MB_FC
 * @brief
 * Modbus function codes summary.
 * These are the implement function codes either for host or for device.
 *
 * @see also fctsupported
 * @see also modbus_t
 */
enum MB_FC
{
    MB_FC_NONE                     = 0,   /*!< null operator */
    MB_FC_READ_COILS               = 1,	/*!< FCT=1 -> read coils or digital outputs */
    MB_FC_READ_DISCRETE_INPUT      = 2,	/*!< FCT=2 -> read digital inputs */
    MB_FC_READ_REGISTERS           = 3,	/*!< FCT=3 -> read registers or analog outputs */
    MB_FC_READ_INPUT_REGISTER      = 4,	/*!< FCT=4 -> read analog inputs */
    MB_FC_WRITE_COIL               = 5,	/*!< FCT=5 -> write single coil or output */
    MB_FC_WRITE_REGISTER           = 6,	/*!< FCT=6 -> write single register */
    MB_FC_WRITE_MULTIPLE_COILS     = 15,	/*!< FCT=15 -> write multiple coils or outputs */
    MB_FC_WRITE_MULTIPLE_REGISTERS = 16	/*!< FCT=16 -> write multiple registers */
};

enum COM_STATES
{
    COM_IDLE                     = 0,
    COM_WAITING                  = 1

};

enum ERR_LIST : int16_t
{
    ERR_SUCCESS                   = 0,
    ERR_NOT_HOST                  = -1,
    ERR_POLLING                   = -2,
    ERR_BUFF_OVERFLOW             = -3,
    ERR_BAD_CRC                   = -4,
    ERR_EXCEPTION                 = -5,
    // TODO(tmm@mcci.com) rename NO_REPLY -> ERR_NO_REPLY
    NO_REPLY			  = -6,
    ERR_RUNT_PACKET		  = -7,
};

// TODO(tmm@mcci.com) use values from MB_EXCEPTION instead
enum
{
    EXC_FUNC_CODE = 1,
    EXC_ADDR_RANGE = 2,
    EXC_REGS_QUANT = 3,
    EXC_EXECUTE = 4,
};

enum MB_EXCEPTION: uint8_t
{
    MB_EXC_ILLEGAL_FUNCTION = 1,
    MB_EXC_ILLEGAL_DATA_ADDRESS = 2,
    MB_EXC_ILLEGAL_DATA_VALUE = 3,
    MB_EXC_SERVER_DEVICE_FAILURE = 4,
    MB_EXC_ACKNOWLEDGE = 5,
    MB_EXC_SERVER_DEVICE_BUSY = 6,
    MB_MEMORY_PARITY_ERROR = 8,
    MB_GATEWAY_PATH_UNAVAILABLE = 0x0A,
    MB_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0B,
};

const unsigned char fctsupported[] =
{
    MB_FC_READ_COILS,
    MB_FC_READ_DISCRETE_INPUT,
    MB_FC_READ_REGISTERS,
    MB_FC_READ_INPUT_REGISTER,
    MB_FC_WRITE_COIL,
    MB_FC_WRITE_REGISTER,
    MB_FC_WRITE_MULTIPLE_COILS,
    MB_FC_WRITE_MULTIPLE_REGISTERS
};

#define T35  5
#define  MAX_BUFFER  64	//!< maximum size for the communication buffer in bytes

/**
 * @class Modbus
 * @brief
 * Arduino class library for communicating with Modbus devices over
 * USB/RS232/485 (via RTU protocol).
 */
class Modbus
{
private:
    ModbusPort *port; //!< pointer to the "port" wrapper object.
    uint16_t u16txenDelay;
    uint8_t u8id; //!< 0=host, 1..247=device number
    uint8_t u8txenpin; //!< flow control pin: 0=USB or RS-232 mode, >0=RS-485 mode
    uint8_t u8state;
    uint8_t au8Buffer[MAX_BUFFER];
    uint8_t u8BufferSize;
    uint8_t lastRec;
    uint16_t *au16regs;
    uint16_t u16InCnt, u16OutCnt, u16errCnt;
    uint16_t u16timeOut;
    uint32_t u32time, u32timeOut;
    ERR_LIST lastError;
    uint8_t u8regsize;

    void init(uint8_t u8id, uint8_t u8txenpin);
    void init(uint8_t u8id);
    void sendTxBuffer();
    uint8_t getRxBuffer(ERR_LIST &errcode);
    uint16_t calcCRC(uint8_t u8length);
    ERR_LIST validateAnswer();
    ERR_LIST validateRequest();
    void get_FC1();
    void get_FC3();
    int8_t process_FC1( uint16_t *regs, uint8_t u8size );
    int8_t process_FC3( uint16_t *regs, uint8_t u8size );
    int8_t process_FC5( uint16_t *regs, uint8_t u8size );
    int8_t process_FC6( uint16_t *regs, uint8_t u8size );
    int8_t process_FC15( uint16_t *regs, uint8_t u8size );
    int8_t process_FC16( uint16_t *regs, uint8_t u8size );
    void buildException( uint8_t u8exception ); // build exception message

public:
    Modbus();
    Modbus(uint8_t u8id);
    Modbus(uint8_t u8id, uint8_t u8txenpin);
    void begin(ModbusPort *pPort, unsigned long u32speed);
    void begin(ModbusPort *pPort, unsigned long u32speed, uint16_t u8config);
    void setTimeOut( uint16_t u16timeout); //!<write communication watch-dog timer
    void setTxEnableDelay(uint16_t u16txen_us); //!<set tx enable delay in us
    uint16_t getTimeOut(); //!<get communication watch-dog timer value
    boolean getTimeOutState(); //!<get communication watch-dog timer state
    int8_t query( modbus_t telegram ); //!<only for host
    int8_t poll(); //!<cyclic poll for host
    int8_t poll( uint16_t *regs, uint8_t u8size ); //!<cyclic poll for device
    uint16_t getInCnt(); //!<number of incoming messages
    uint16_t getOutCnt(); //!<number of outcoming messages
    uint16_t getErrCnt(); //!<error counter
    uint8_t getID(); //!<get device ID between 1 and 247
    uint8_t getState();
    ERR_LIST getLastError(); //!<get last error value.
    void setLastError(ERR_LIST errcode); //!<set last error value.
    void setID( uint8_t u8id ); //!<write new ID for the device
    void end(); //!<finish any communication and release serial communication port
};

/* _____PUBLIC FUNCTIONS_____________________________________________________ */

/**
 * @brief
 * Default Constructor: make a host, no rx/tx control
 *
 * @ingroup setup
 */
Modbus::Modbus()
{
    init(0, 0);
}

/**
 * @brief
 * Full constructor for a host/device through USB/RS232C, no rx/tx control
 *
 * @param u8id   node address 0=host, 1..247=device
 * @ingroup setup
 * @overload Modbus::Modbus(uint8_t u8id)
 * @overload Modbus::Modbus()
 */
Modbus::Modbus(uint8_t u8id)
{
    init(u8id, 0);
}

/**
 * @brief
 * Full constructor for a host/device through USB/RS232C/RS485
 * It needs a pin for flow control only for RS485 mode
 *
 * @param u8id   node address 0=host, 1..247=device
 * @param u8txenpin pin for txen RS-485 (=0 means USB/RS232C mode)
 * @ingroup setup
 * @overload Modbus::Modbus(uint8_t u8id, uint8_t u8txenpin)
 * @overload Modbus::Modbus(uint8_t u8id)
 * @overload Modbus::Modbus()
 */
Modbus::Modbus(uint8_t u8id, uint8_t u8txenpin)
{
    init(u8id, u8txenpin);
}

/**
 * @brief
 * Start class object.
 *
 * Sets up the serial port using specified baud rate.
 * Call once class has been instantiated, typically within setup().
 *
 * @see http://arduino.cc/en/Serial/Begin#.Uy4CJ6aKlHY
 * @param speed   baud rate, in standard increments (300..115200)
 * @ingroup setup
 */
void Modbus::begin(ModbusPort *pPort, unsigned long u32speed)
{
    this->begin(pPort, u32speed, SERIAL_8N2);
}

/**
 * @brief
 * Initialize class object.
 *
 * Sets up the serial port using specified baud rate.
 * Call once class has been instantiated, typically within setup().
 *
 * @see http://arduino.cc/en/Serial/Begin#.Uy4CJ6aKlHY
 * @param speed   baud rate, in standard increments (300..115200)
 * @param config  data frame settings (data length, parity and stop bits)
 * @ingroup setup
 */
void Modbus::begin(ModbusPort *pPort, unsigned long u32speed, uint16_t config)
{

    port = pPort;

    port->begin(u32speed, config);

    if (u8txenpin > 1)   // pin 0 & pin 1 are reserved for RX/TX
    {
        // return RS485 transceiver to receive mode
        pinMode(u8txenpin, OUTPUT);
        digitalWrite(u8txenpin, LOW);
    }

    port->drainRead();
    lastRec = u8BufferSize = 0;
    u16InCnt = u16OutCnt = u16errCnt = 0;
    this->u16txenDelay = 100;
}

/**
 * @brief
 * Method to write a new device ID address
 *
 * @param 	u8id	new device address between 1 and 247
 * @ingroup setup
 */
void Modbus::setID( uint8_t u8id)
{
    if (( u8id != 0) && (u8id <= 247))
    {
        this->u8id = u8id;
    }
}

/**
 * @brief
 * Method to read current device ID address
 *
 * @return u8id	current device address between 1 and 247
 * @ingroup setup
 */
uint8_t Modbus::getID()
{
    return this->u8id;
}

/**
 * @brief
 * Initialize time-out parameter
 *
 * Call once class has been instantiated, typically within setup().
 * The time-out timer is reset each time that there is a successful communication
 * between host and device. It works for both.
 *
 * @param time-out value (ms)
 * @ingroup setup
 */
void Modbus::setTimeOut( uint16_t u16timeOut)
{
    this->u16timeOut = u16timeOut;
}

/**
 * @brief
 * Initialize the tx enable delay, in microseconds
 *
 * Called after the class has been created. The txEnable controls how long the
 * system will wait from enabling TX before sending the first byte.
 *
 * @param transmit delay value (microseconds)
 * @ingroup setup
*/
void Modbus::setTxEnableDelay(uint16_t u16txEnableUsec)
{
    this->u16txenDelay = u16txEnableUsec;
}

/**
 * @brief
 * Return communication Watchdog state.
 * It could be usefull to reset outputs if the watchdog is fired.
 *
 * @return TRUE if millis() > u32timeOut
 * @ingroup loop
 */
boolean Modbus::getTimeOutState()
{
    return ((int32_t)(millis() - u32timeOut) > 0);
}

/**
 * @brief
 * Get input messages counter value
 * This can be useful to diagnose communication
 *
 * @return input messages counter
 * @ingroup buffer
 */
uint16_t Modbus::getInCnt()
{
    return u16InCnt;
}

/**
 * @brief
 * Get transmitted messages counter value
 * This can be useful to diagnose communication
 *
 * @return transmitted messages counter
 * @ingroup buffer
 */
uint16_t Modbus::getOutCnt()
{
    return u16OutCnt;
}

/**
 * @brief
 * Get errors counter value
 * This can be useful to diagnose communication
 *
 * @return errors counter
 * @ingroup buffer
 */
uint16_t Modbus::getErrCnt()
{
    return u16errCnt;
}

/**
 * Get modbus host state
 *
 * @return = 0 IDLE, = 1 WAITING FOR ANSWER
 * @ingroup buffer
 */
uint8_t Modbus::getState()
{
    return u8state;
}

/**
 * Get the last error in the protocol processor
 *
 * @return   last reported error, if non zero. Negative value means a protocol error, positive is a Modbus exception code
 * @ingroup buffer
 */
ERR_LIST Modbus::getLastError()
{
    return this->lastError;
}

void Modbus::setLastError(ERR_LIST errorValue)
{
	this->lastError = errorValue;
}

/**
 * @brief
 * *** Only Modbus host ***
 * Generate a query to a device with a modbus_t telegram structure
 * The host must be in COM_IDLE mode. After it, its state would be COM_WAITING.
 * This method has to be called only in loop() section.
 *
 * @see modbus_t
 * @param modbus_t  modbus telegram structure (id, fct, ...)
 * @ingroup loop
 * @todo finish function 15
 */
int8_t Modbus::query( modbus_t telegram )
{
    uint8_t u8regsno, u8bytesno;
    if (u8id!=0) return -2;
    if (u8state != COM_IDLE) return -1;

    if ((telegram.u8id==0) || (telegram.u8id>247)) return -3;

    au16regs = telegram.au16reg;

    // telegram header
    au8Buffer[ ID ]         = telegram.u8id;
    au8Buffer[ FUNC ]       = telegram.u8fct;
    au8Buffer[ ADD_HI ]     = highByte(telegram.u16RegAdd );
    au8Buffer[ ADD_LO ]     = lowByte( telegram.u16RegAdd );

    switch( telegram.u8fct )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
    case MB_FC_READ_REGISTERS:
    case MB_FC_READ_INPUT_REGISTER:
        au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
        au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_COIL:
        au8Buffer[ NB_HI ]      = ((au16regs[0] > 0) ? 0xff : 0);
        au8Buffer[ NB_LO ]      = 0;
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_REGISTER:
        au8Buffer[ NB_HI ]      = highByte(au16regs[0]);
        au8Buffer[ NB_LO ]      = lowByte(au16regs[0]);
        u8BufferSize = 6;
        break;
    case MB_FC_WRITE_MULTIPLE_COILS: // TODO: implement "sending coils"
        u8regsno = telegram.u16CoilsNo / 16;
        u8bytesno = u8regsno * 2;
        if ((telegram.u16CoilsNo % 16) != 0)
        {
            u8bytesno++;
            u8regsno++;
        }

        au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
        au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
        au8Buffer[ NB_LO+1 ]    = u8bytesno;
        u8BufferSize = 7;

        u8regsno = u8bytesno = 0; // now auxiliary registers
        for (uint16_t i = 0; i < telegram.u16CoilsNo; i++)
        {


        }
        break;

    case MB_FC_WRITE_MULTIPLE_REGISTERS:
        au8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
        au8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
        au8Buffer[ NB_LO+1 ]    = (uint8_t) ( telegram.u16CoilsNo * 2 );
        u8BufferSize = 7;

        for (uint16_t i=0; i< telegram.u16CoilsNo; i++)
        {
            au8Buffer[ u8BufferSize ] = highByte( au16regs[ i ] );
            u8BufferSize++;
            au8Buffer[ u8BufferSize ] = lowByte( au16regs[ i ] );
            u8BufferSize++;
        }
        break;
    }

    sendTxBuffer();
    u8state = COM_WAITING;
    return 0;
}

/**
 * @brief *** Only for Modbus host ***
 * This method checks if there is any incoming answer if pending.
 * If there is no answer, it would change host state to COM_IDLE.
 * This method must be called only at loop section.
 * Avoid any delay() function.
 *
 * Any incoming data would be redirected to au16regs pointer,
 * as defined in its modbus_t query telegram.
 *
 * @params	nothing
 * @return 0 if no progress, -1 if error, +1 if operation completed.
 * @ingroup loop
 */
int8_t Modbus::poll()
{
    // check if there is any incoming frame
    int current;
    current = port->available();

    if (this->getTimeOutState())
    {
        u8state = COM_IDLE;
        lastError = NO_REPLY;
        u16errCnt++;
        return -1;
    }

    if (current == 0) return 0;

    // check T35 after frame end or still no frame end
    if (current != lastRec)
    {
        lastRec = current;
        u32time = millis() + T35;
        return 0;
    }
    if (millis() < u32time) return 0;

    // transfer Serial buffer frame to auBuffer
    lastRec = 0;
    uint8_t u8Len;
    ERR_LIST errcode;

    u8Len = getRxBuffer(errcode);
    if (errcode != 0)
    {
	this->lastError = errcode;
        u8state = COM_IDLE;
        u16errCnt++;
        return errcode;
    }

    // validate message: id, CRC, FCT, exception
    errcode = validateAnswer();
    if (errcode != 0)
    {
	this->lastError = errcode;
        u8state = COM_IDLE;
        return -1;
    }

    // process answer
    switch( au8Buffer[ FUNC ] )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
        // call get_FC1 to transfer the incoming message to au16regs buffer
        get_FC1( );
        break;
    case MB_FC_READ_INPUT_REGISTER:
    case MB_FC_READ_REGISTERS :
        // call get_FC3 to transfer the incoming message to au16regs buffer
        get_FC3( );
        break;
    case MB_FC_WRITE_COIL:
    case MB_FC_WRITE_REGISTER :
    case MB_FC_WRITE_MULTIPLE_COILS:
    case MB_FC_WRITE_MULTIPLE_REGISTERS :
        // nothing to do
        break;
    default:
        break;
    }
    u8state = COM_IDLE;
    return 1;
}

/**
 * @brief
 * *** Only for Modbus device ***
 * This method checks if there is any incoming query
 * Afterwards, it would shoot a validation routine plus a register query
 * Avoid any delay() function !!!!
 * After a successful frame between the host and the device, the time-out timer is reset.
 *
 * @param *regs  register table for communication exchange
 * @param u8size  size of the register table
 * @return 0 if no query, -1 if any error, 1 if correct query processed
 * @ingroup loop
 */
int8_t Modbus::poll( uint16_t *regs, uint8_t u8size )
{

    au16regs = regs;
    u8regsize = u8size;
    int current;


    // check if there is any incoming frame
    current = port->available();

    if (current == 0) return 0;

    // check T35 after frame end or still no frame end
    // We allow the ring buffer to accumulate the data; this
    // limts the maximum message size to the size of the ring
    // buffer.
    if (current != lastRec)
    {
        lastRec = current;
        u32time = millis() + T35;
        return 0;
    }
    if (millis() < u32time) return 0;

    lastRec = 0;
    uint8_t u8Len;
    ERR_LIST errcode;

    u8Len = getRxBuffer(errcode);

    if (errcode != 0)
	{
	lastError = errcode;
	return -1;
	}

    // check device id -- since port->available() was non-zero, we can assume 
    // that at least one byte was received.  We check the address to save
    // CRC calculations, in case the message is not for us.
    if (au8Buffer[ ID ] != u8id)
    {
        // TODO(tmm@mcci.com) increment a statistic
        return 0;
    }

    // validate message: CRC, FCT, address and size
    errcode = validateRequest();
    if (errcode != 0)
    {
        if (errcode > 0)
        {
            buildException( errcode );
            sendTxBuffer();
        }
        lastError = errcode;
        return -1;
    }

    u32timeOut = millis() + long(u16timeOut);
    lastError = ERR_SUCCESS;

    // process message
    switch( au8Buffer[ FUNC ] )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
        return process_FC1( regs, u8size );
        break;
    case MB_FC_READ_INPUT_REGISTER:
    case MB_FC_READ_REGISTERS :
        return process_FC3( regs, u8size );
        break;
    case MB_FC_WRITE_COIL:
        return process_FC5( regs, u8size );
        break;
    case MB_FC_WRITE_REGISTER :
        return process_FC6( regs, u8size );
        break;
    case MB_FC_WRITE_MULTIPLE_COILS:
        return process_FC15( regs, u8size );
        break;
    case MB_FC_WRITE_MULTIPLE_REGISTERS :
        return process_FC16( regs, u8size );
        break;
    default:
        break;
    }
    return 1;
}

/* _____PRIVATE FUNCTIONS_____________________________________________________ */

void Modbus::init(uint8_t u8id, uint8_t u8txenpin)
{
    this->u8id = u8id;
    this->u8txenpin = u8txenpin;
    this->u16timeOut = 1000;
}

void Modbus::init(uint8_t u8id)
{
    this->u8id = u8id;
    this->u8txenpin = 0;
    this->u16timeOut = 1000;
}

/**
 * @brief
 * This method moves Serial buffer data to the Modbus au8Buffer.
 *
 * The ring buffer is drained, and is assumed to represent the
 * entire message.
 *
 * @return buffer size if OK, ERR_BUFF_OVERFLOW if u8BufferSize >= MAX_BUFFER
 * @ingroup buffer
 */
uint8_t Modbus::getRxBuffer(ERR_LIST &errcode)
{
    boolean bBuffOverflow = false;

    if (u8txenpin > 1) digitalWrite( u8txenpin, LOW );

    u8BufferSize = 0;
    while ( port->available() )
        {
	if (u8BufferSize >= MAX_BUFFER)
		bBuffOverflow = true;
	else
		{
		au8Buffer[ u8BufferSize++ ] = port->read();
		}
        }
    u16InCnt++;

    if (bBuffOverflow)
    {
        u16errCnt++;
	errcode = ERR_BUFF_OVERFLOW;
    }
    else
    {
	errcode = ERR_SUCCESS;
    }
    return u8BufferSize;
}

/**
 * @brief
 * This method transmits au8Buffer to Serial line.
 * Only if u8txenpin != 0, there is a flow handling in order to keep
 * the RS485 transceiver in output state as long as the message is being sent.
 * This is done with UCSRxA register.
 * The CRC is appended to the buffer before starting to send it.
 *
 * @param nothing
 * @return nothing
 * @ingroup buffer
 */
void Modbus::sendTxBuffer()
{
    uint8_t i = 0;

    // append CRC to message
    uint16_t u16crc = calcCRC( u8BufferSize );
    au8Buffer[ u8BufferSize ] = u16crc >> 8;
    u8BufferSize++;
    au8Buffer[ u8BufferSize ] = u16crc & 0x00ff;
    u8BufferSize++;

    // set RS485 transceiver to transmit mode
    if (u8txenpin > 1)
    {
        digitalWrite( u8txenpin, HIGH );
        delayMicroseconds(this->u16txenDelay);
    }

    // transfer buffer to serial line
    port->write( au8Buffer, u8BufferSize );

    // keep RS485 transceiver in transmit mode as long as sending
    if (u8txenpin > 1)
    {
        port->drainWrite();

        // return RS485 transceiver to receive mode
        digitalWrite( u8txenpin, LOW );
    }
    port->drainRead();

    u8BufferSize = 0;

    // set time-out for host
    u32timeOut = millis() + (uint32_t) u16timeOut;

    // increase message counter
    u16OutCnt++;
}

/**
 * @brief
 * This method calculates CRC
 *
 * @return uint16_t calculated CRC value for the message
 * @ingroup buffer
 */
uint16_t Modbus::calcCRC(uint8_t u8length)
{
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < u8length; i++)
    {
        temp = temp ^ au8Buffer[i];
        for (unsigned char j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>=1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
}

/**
 * @brief
 * This method validates device incoming messages
 *
 * @return 0 if OK, non-zero error code if anything fails
 * @ingroup buffer
 */
ERR_LIST Modbus::validateRequest()
{
    // check message crc vs calculated crc
    uint16_t u16MsgCRC =
        ((au8Buffer[u8BufferSize - 2] << 8)
         | au8Buffer[u8BufferSize - 1]); // combine the crc Low & High bytes
    if ( calcCRC( u8BufferSize-2 ) != u16MsgCRC )
    {
        u16errCnt ++;
        return ERR_BAD_CRC;
    }

    // check fct code
    boolean isSupported = false;
    for (uint8_t i = 0; i< sizeof( fctsupported ); i++)
    {
        if (fctsupported[i] == au8Buffer[FUNC])
        {
            isSupported = 1;
            break;
        }
    }
    if (!isSupported)
    {
        u16errCnt ++;
        return ERR_LIST(EXC_FUNC_CODE);
    }

    // check start address & nb range
    uint16_t u16regs = 0;
    uint8_t u8regs;
    switch ( au8Buffer[ FUNC ] )
    {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE_INPUT:
    case MB_FC_WRITE_MULTIPLE_COILS:
        u16regs = word( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ]) / 16;
        u16regs += word( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ]) /16;
        u8regs = (uint8_t) u16regs;
        if (u8regs > u8regsize) return ERR_LIST(EXC_ADDR_RANGE);
        break;
    case MB_FC_WRITE_COIL:
        u16regs = word( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ]) / 16;
        u8regs = (uint8_t) u16regs;
        if (u8regs > u8regsize) return ERR_LIST(EXC_ADDR_RANGE);
        break;
    case MB_FC_WRITE_REGISTER :
        u16regs = word( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ]);
        u8regs = (uint8_t) u16regs;
        if (u8regs > u8regsize) return ERR_LIST(EXC_ADDR_RANGE);
        break;
    case MB_FC_READ_REGISTERS :
    case MB_FC_READ_INPUT_REGISTER :
    case MB_FC_WRITE_MULTIPLE_REGISTERS :
        u16regs = word( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ]);
        u16regs += word( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ]);
        u8regs = (uint8_t) u16regs;
        if (u8regs > u8regsize) return ERR_LIST(EXC_ADDR_RANGE);
        break;
    }
    return ERR_SUCCESS; // OK, no exception code thrown
}

/**
 * @brief
 * This method validates host incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
ERR_LIST Modbus::validateAnswer()
{
    if (u8BufferSize < 4)
    {
	    u16errCnt ++;
	    return ERR_RUNT_PACKET;
    }
    // check message crc vs calculated crc
    uint16_t u16MsgCRC =
        ((au8Buffer[u8BufferSize - 2] << 8)
         | au8Buffer[u8BufferSize - 1]); // combine the crc Low & High bytes
    if ( calcCRC( u8BufferSize-2 ) != u16MsgCRC )
    {
        u16errCnt ++;
        return ERR_BAD_CRC;
    }

    // check exception
    if ((au8Buffer[ FUNC ] & 0x80) != 0)
    {
        u16errCnt ++;
        return ERR_LIST(au8Buffer[EXCEPTION]);
    }

    // check fct code
    boolean isSupported = false;
    for (uint8_t i = 0; i< sizeof( fctsupported ); i++)
    {
        if (fctsupported[i] == au8Buffer[FUNC])
        {
            isSupported = 1;
            break;
        }
    }
    if (!isSupported)
    {
        u16errCnt ++;
        return ERR_LIST(EXC_FUNC_CODE);
    }

    return ERR_SUCCESS; // OK, no exception code thrown
}

/**
 * @brief
 * This method builds an exception message
 *
 * @ingroup buffer
 */
void Modbus::buildException( uint8_t u8exception )
{
    uint8_t u8func = au8Buffer[ FUNC ];  // get the original FUNC code

    au8Buffer[ ID ]      = u8id;
    au8Buffer[ FUNC ]    = u8func + 0x80;
    au8Buffer[ 2 ]       = u8exception;
    u8BufferSize         = EXCEPTION_SIZE;
}

/**
 * This method processes functions 1 & 2 (for host)
 * This method puts the device answer into host data buffer
 *
 * @ingroup register
 * TODO: finish its implementation
 */
void Modbus::get_FC1()
{
    uint8_t u8byte, i;
    u8byte = 0;

    //  for (i=0; i< au8Buffer[ 2 ] /2; i++) {
    //    au16regs[ i ] = word(
    //    au8Buffer[ u8byte ],
    //    au8Buffer[ u8byte +1 ]);
    //    u8byte += 2;
    //  }
}

/**
 * This method processes functions 3 & 4 (for host)
 * This method puts the device answer into host data buffer
 *
 * @ingroup register
 */
void Modbus::get_FC3()
{
    uint8_t u8byte, i;
    u8byte = 3;

    for (i=0; i< au8Buffer[ 2 ] /2; i++)
    {
        au16regs[ i ] = word(
                            au8Buffer[ u8byte ],
                            au8Buffer[ u8byte +1 ]);
        u8byte += 2;
    }
}

/**
 * @brief
 * This method processes functions 1 & 2
 * This method reads a bit array and transfers it to the host
 *
 * @return u8BufferSize Response to host length
 * @ingroup discrete
 */
int8_t Modbus::process_FC1( uint16_t *regs, uint8_t u8size )
{
    uint8_t u8currentRegister, u8currentBit, u8bytesno, u8bitsno;
    uint8_t u8CopyBufferSize;
    uint16_t u16currentCoil, u16coil;

    // get the first and last coil from the message
    uint16_t u16StartCoil = word( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ] );
    uint16_t u16Coilno = word( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ] );

    // put the number of bytes in the outcoming message
    u8bytesno = (uint8_t) (u16Coilno / 8);
    if (u16Coilno % 8 != 0) u8bytesno ++;
    au8Buffer[ ADD_HI ]  = u8bytesno;
    u8BufferSize         = ADD_LO;

    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;

    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++)
    {
        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (uint8_t) (u16coil / 16);
        u8currentBit = (uint8_t) (u16coil % 16);

        bitWrite(
            au8Buffer[ u8BufferSize ],
            u8bitsno,
            bitRead( regs[ u8currentRegister ], u8currentBit ) );
        u8bitsno ++;

        if (u8bitsno > 7)
        {
            u8bitsno = 0;
            u8BufferSize++;
        }
    }

    // send outcoming message
    if (u16Coilno % 8 != 0) u8BufferSize ++;
    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();
    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes functions 3 & 4
 * This method reads a word array and transfers it to the host
 *
 * @return u8BufferSize Response to host length
 * @ingroup register
 */
int8_t Modbus::process_FC3( uint16_t *regs, uint8_t u8size )
{

    uint8_t u8StartAdd = word( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ] );
    uint8_t u8regsno = word( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ] );
    uint8_t u8CopyBufferSize;
    uint8_t i;

    au8Buffer[ 2 ]       = u8regsno * 2;
    u8BufferSize         = 3;

    for (i = u8StartAdd; i < u8StartAdd + u8regsno; i++)
    {
        au8Buffer[ u8BufferSize ] = highByte(regs[i]);
        u8BufferSize++;
        au8Buffer[ u8BufferSize ] = lowByte(regs[i]);
        u8BufferSize++;
    }
    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();

    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 5
 * This method writes a value assigned by the host to a single bit
 *
 * @return u8BufferSize Response to host length
 * @ingroup discrete
 */
int8_t Modbus::process_FC5( uint16_t *regs, uint8_t u8size )
{
    uint8_t u8currentRegister, u8currentBit;
    uint8_t u8CopyBufferSize;
    uint16_t u16coil = word( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ] );

    // point to the register and its bit
    u8currentRegister = (uint8_t) (u16coil / 16);
    u8currentBit = (uint8_t) (u16coil % 16);

    // write to coil
    bitWrite(
        regs[ u8currentRegister ],
        u8currentBit,
        au8Buffer[ NB_HI ] == 0xff );


    // send answer to host
    u8BufferSize = 6;
    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();

    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 6
 * This method writes a value assigned by the host to a single word
 *
 * @return u8BufferSize Response to host length
 * @ingroup register
 */
int8_t Modbus::process_FC6( uint16_t *regs, uint8_t u8size )
{

    uint8_t u8add = word( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ] );
    uint8_t u8CopyBufferSize;
    uint16_t u16val = word( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ] );

    regs[ u8add ] = u16val;

    // keep the same header
    u8BufferSize         = RESPONSE_SIZE;

    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();

    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 15
 * This method writes a bit array assigned by the host
 *
 * @return u8BufferSize Response to host length
 * @ingroup discrete
 */
int8_t Modbus::process_FC15( uint16_t *regs, uint8_t u8size )
{
    uint8_t u8currentRegister, u8currentBit, u8frameByte, u8bitsno;
    uint8_t u8CopyBufferSize;
    uint16_t u16currentCoil, u16coil;
    boolean bTemp;

    // get the first and last coil from the message
    uint16_t u16StartCoil = word( au8Buffer[ ADD_HI ], au8Buffer[ ADD_LO ] );
    uint16_t u16Coilno = word( au8Buffer[ NB_HI ], au8Buffer[ NB_LO ] );


    // read each coil from the register map and put its value inside the outcoming message
    u8bitsno = 0;
    u8frameByte = 7;
    for (u16currentCoil = 0; u16currentCoil < u16Coilno; u16currentCoil++)
    {

        u16coil = u16StartCoil + u16currentCoil;
        u8currentRegister = (uint8_t) (u16coil / 16);
        u8currentBit = (uint8_t) (u16coil % 16);

        bTemp = bitRead(
                    au8Buffer[ u8frameByte ],
                    u8bitsno );

        bitWrite(
            regs[ u8currentRegister ],
            u8currentBit,
            bTemp );

        u8bitsno ++;

        if (u8bitsno > 7)
        {
            u8bitsno = 0;
            u8frameByte++;
        }
    }

    // send outgoing message
    // it's just a copy of the incomping message until 6th byte
    u8BufferSize         = 6;
    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();
    return u8CopyBufferSize;
}

/**
 * @brief
 * This method processes function 16
 * This method writes a word array assigned by the host
 *
 * @return u8BufferSize Response to host length
 * @ingroup register
 */
int8_t Modbus::process_FC16( uint16_t *regs, uint8_t u8size )
{
    uint8_t u8func = au8Buffer[ FUNC ];  // get the original FUNC code
    uint8_t u8StartAdd = au8Buffer[ ADD_HI ] << 8 | au8Buffer[ ADD_LO ];
    uint8_t u8regsno = au8Buffer[ NB_HI ] << 8 | au8Buffer[ NB_LO ];
    uint8_t u8CopyBufferSize;
    uint8_t i;
    uint16_t temp;

    // build header
    au8Buffer[ NB_HI ]   = 0;
    au8Buffer[ NB_LO ]   = u8regsno;
    u8BufferSize         = RESPONSE_SIZE;

    // write registers
    for (i = 0; i < u8regsno; i++)
    {
        temp = word(
                   au8Buffer[ (BYTE_CNT + 1) + i * 2 ],
                   au8Buffer[ (BYTE_CNT + 2) + i * 2 ]);

        regs[ u8StartAdd + i ] = temp;
    }
    u8CopyBufferSize = u8BufferSize +2;
    sendTxBuffer();

    return u8CopyBufferSize;
}
