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

#pragma once

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
    ERR_NO_REPLY                  = -6,
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
#define  MAX_BUFFER  256	//!< maximum size for the communication buffer in bytes

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

