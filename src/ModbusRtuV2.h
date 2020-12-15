/**
 * @file        ModbusRtuV2.h
 * @author      Samuel Marco i Armengol
 * @contact     sammarcoarmengol@gmail.com
 * @contribution Helium6072, tmm@mcci.com
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

#ifndef _ModbusRtuV2_h_
# define _ModbusRtuV2_h_

#include <inttypes.h>
#include "Arduino.h"
#include "Print.h"
#include "ModbusPort.h"

// Versioning information.

/// @brief generate a 32-bit version
#define MCCI_MODBUS_RTU_ARDUINO_VERSION_CALC(major, minor, patch, local)	\
    (((major) << 24u) | ((minor) << 16u) | ((patch) << 8u) | (local))

/// @brief the version of this library - increment on every addition/deletion
#define	MCCI_MODBUS_RTU_ARDUINO_VERSION	MCCI_MODBUS_RTU_ARDUINO_VERSION_CALC(0, 5, 0, 0)        /* v0.5.0.0 */

/// @brief get major code
#define	MCCI_MODBUS_RTU_ARDUINO_VERSION_GET_MAJOR(v)	\
    (((v) >> 24u) & 0xFFu)

/// @brief get minor code
#define	MCCI_MODBUS_RTU_ARDUINO_VERSION_GET_MINOR(v)	\
    (((v) >> 16u) & 0xFFu)

/// @brief get patch number (from semantic version)
#define	MCCI_MODBUS_RTU_ARDUINO_VERSION_GET_PATCH(v)	\
    (((v) >> 8u) & 0xFFu)

/// @brief get local version -- used in between official releases.
#define	MCCI_MODBUS_RTU_ARDUINO_VERSION_GET_LOCAL(v)	\
    ((v) & 0xFFu)

namespace McciCatena {

/**
 * @struct modbus_datagram_t
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
modbus_datagram_t;

enum class ModbusMessageSize
{
    RESPONSE = 6,
    EXCEPTION = 3,
    CHECKSUM = 2
};

/**
 * @enum ModbusMessageOffset
 * @brief
 * Indices to datagram frame positions
 */
enum class ModbusMessageOffset : uint8_t
{
    ID = 0, //!< Index of ID field
    FUNC, //!< Index of Function code
    ADD_HI, //!< Index of Address high byte
    ADD_LO, //!< Index of Address low byte
    NB_HI, //!< Index of Number of coils or registers high byte
    NB_LO, //!< Index of Number of coils or registers low byte
    BYTE_CNT,  //!< Index of byte count

    EXCEPTION = ModbusMessageOffset::FUNC + 1, //!< Index in exception response of exception code.
};

/**
 * @enum ModbusFunction
 * @brief
 * Modbus function codes.
 * These are the implemented function codes either for host or for device.
 *
 * @see also fctsupported
 * @see also modbus_datagram_t
 */
enum class ModbusFunction : uint8_t
{
    NONE                     = 0,     /*!< null operator */
    READ_COILS               = 1,     /*!< FCT=1 -> read coils or digital outputs */
    READ_DISCRETE_INPUT      = 2,     /*!< FCT=2 -> read digital inputs */
    READ_REGISTERS           = 3,     /*!< FCT=3 -> read registers or analog outputs */
    READ_INPUT_REGISTER      = 4,     /*!< FCT=4 -> read analog inputs */
    WRITE_COIL               = 5,     /*!< FCT=5 -> write single coil or output */
    WRITE_REGISTER           = 6,     /*!< FCT=6 -> write single register */
    WRITE_MULTIPLE_COILS     = 15,    /*!< FCT=15 -> write multiple coils or outputs */
    WRITE_MULTIPLE_REGISTERS = 16     /*!< FCT=16 -> write multiple registers */
};

/**
 * @enum ModbusException_t
 * @brief
 * Modbus exception codes used in datagrams
 *
 * @see also modbus_datagram_t
 */
enum class ModbusException_t: uint8_t
    {
    ILLEGAL_FUNCTION = 1,
    ILLEGAL_DATA_ADDRESS = 2,
    ILLEGAL_DATA_VALUE = 3,
    SERVER_DEVICE_FAILURE = 4,
    ACKNOWLEDGE = 5,
    SERVER_DEVICE_BUSY = 6,
    MEMORY_PARITY_ERROR = 8,
    GATEWAY_PATH_UNAVAILABLE = 0x0A,
    GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0B,
    };

/**
 * @class McciCatena::Modbus
 * @brief
 * Arduino class library for communicating with Modbus devices over
 * USB/RS232/485 (via RTU protocol). Can represent either host or device.
 */
class Modbus
{
public:
    enum class CommState : uint8_t
        {
        IDLE                     = 0,
        WAITING                  = 1
        };

    enum class Error : int16_t
        {
        ILLEGAL_MB_EXECUTION      = 4,
        ILLEGAL_MB_COUNT          = 3,
        ILLEGAL_MB_ADDRESS        = 2,
        ILLEGAL_MB_FUNCTION       = 1,
        SUCCESS                   = 0,
        NOT_HOST                  = -1,
        POLLING                   = -2,
        BUFF_OVERFLOW             = -3,
        BAD_CRC                   = -4,
        EXCEPTION                 = -5,
        NO_REPLY                  = -6,
        RUNT_PACKET               = -7,
        ILLEGAL_DEVICE_ADDRESS    = -8,
        };


    // timeout between characters before we assume end of message.
    static constexpr unsigned kT35 = 5;
    static constexpr unsigned kMaxBuffer = 256; //!< maximum size for the communication buffer in bytes

    /// @brief check whether address is a valid device address
    static constexpr bool isDeviceAddress(uint8_t id)
        {
        return 1 <= id && id <= 247;
        }

private:
    uint16_t getMessageWord(unsigned offset) const
        {
        return (this->au8Buffer[offset] << 8u) + this->au8Buffer[offset + 1];
        }

    uint16_t getMessageAddress(void) const
        {
        return this->getMessageWord(unsigned(ModbusMessageOffset::ADD_HI));
        }

    uint16_t getMessageCount(void) const
        {
        return this->getMessageWord(unsigned(ModbusMessageOffset::NB_HI));
        }

    void putMessageWord(unsigned offset, uint16_t v)
        {
        auto p = &this->au8Buffer[offset];

        *p++ = uint8_t(v >> 8u);
        *p = uint8_t(v);
        }

    void putMessageAddress(uint16_t v)
        {
        this->putMessageWord(unsigned(ModbusMessageOffset::ADD_HI), v);
        }

    void putMessageCount(uint16_t v)
        {
        this->putMessageWord(unsigned(ModbusMessageOffset::NB_HI), v);
        }

private:
    ModbusPort *port; //!< pointer to the "port" wrapper object.
    uint16_t u16txenDelay;
    uint8_t u8id; //!< 0=host, 1..247=device number
    uint8_t u8txenpin; //!< flow control pin: 0=USB or RS-232 mode, >0=RS-485 mode
    uint8_t u8rxenpin; //!< flow control pin: 0=USB or RS-232 mode, >0=RS-485 mode
    CommState u8state;
    uint8_t au8Buffer[kMaxBuffer];
    uint8_t u8BufferSize;
    uint8_t lastRec;
    uint16_t *au16regs;
    uint16_t u16InCnt, u16OutCnt, u16errCnt;
    uint16_t u16timeOut;
    uint32_t u32time, u32timeOut;
    Error lastError;
    uint8_t u8regsize;

    void init(uint8_t u8id, uint8_t u8txenpin, uint8_t u8rxenpin);
    void init(uint8_t u8id, uint8_t u8txenpin);
    void init(uint8_t u8id);
    void sendTxBuffer();
    uint8_t getRxBuffer(Error &errcode);
    uint16_t calcCRC(uint8_t u8length);
    Error validateAnswer();
    Error validateRequest();
    void get_FC1();
    void get_FC3();
    int8_t process_FC1( uint16_t *regs, uint8_t u8size );
    int8_t process_FC3( uint16_t *regs, uint8_t u8size );
    int8_t process_FC5( uint16_t *regs, uint8_t u8size );
    int8_t process_FC6( uint16_t *regs, uint8_t u8size );
    int8_t process_FC15( uint16_t *regs, uint8_t u8size );
    int8_t process_FC16( uint16_t *regs, uint8_t u8size );
    void buildException( ModbusException_t u8exception ); // build exception message

    static const ModbusFunction fctsupported[];

public:
    /**
     * @brief
     * Default Constructor: make a host, no rx/tx control
     *
     * @ingroup setup
     */
    Modbus() : u8id(0), u8txenpin(0), u8rxenpin(0), u16timeOut(1000) {};

    /**
     * @brief
     * Full constructor for a host/device through USB/RS232C, no rx/tx control
     *
     * @param u8id   node address 0=host, 1..247=device
     * @ingroup setup
     * @overload Modbus::Modbus(uint8_t u8id)
     * @overload Modbus::Modbus()
     */
    Modbus(uint8_t u8id) : u8id(u8id), u8txenpin(0), u8rxenpin(0), u16timeOut(1000) {};

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
    Modbus(uint8_t u8id, uint8_t u8txenpin) : u8id(u8id), u8txenpin(u8txenpin), u8rxenpin(0), u16timeOut(1000) {};

    /**
     * @brief
     * Full constructor for a host/device through USB/RS232C/RS485
     * It needs a pin for flow control only for RS485 mode
     *
     * @param u8id   node address 0=host, 1..247=device
     * @param u8txenpin pin for txen RS-485 (=0 means USB/RS232C mode)
     * @param u8rxenpin pin for rxen RS-485 (=0 means USB/RS232C mode)
     * @ingroup setup
     * @overload Modbus::Modbus(uint8_t u8id, uint8_t u8txenpin, uint8_t u8rxenpin)
     * @overload Modbus::Modbus(uint8_t u8id, uint8_t u8txenpin)
     * @overload Modbus::Modbus(uint8_t u8id)
     * @overload Modbus::Modbus()
     */
    Modbus(uint8_t u8id, uint8_t u8txenpin, uint8_t u8rxenpin) : u8id(u8id), u8txenpin(u8txenpin), u8rxenpin(u8rxenpin), u16timeOut(1000) {};

    void begin(ModbusPort *pPort, unsigned long u32speed);
    void begin(ModbusPort *pPort, unsigned long u32speed, uint16_t u8config);
    void setTimeOut( uint16_t u16timeout); //!<write communication watch-dog timer
    void setTxEnableDelay(uint16_t u16txen_us); //!<set tx enable delay in us
    uint16_t getTimeOut(); //!<get communication watch-dog timer value
    boolean getTimeOutState(); //!<get communication watch-dog timer state
    Error query( modbus_datagram_t &datagram ); //!<only for host
    Error poll(); //!<cyclic poll for host
    int8_t poll( uint16_t *regs, uint8_t u8size ); //!<cyclic poll for device
    uint16_t getInCnt(); //!<number of incoming messages
    uint16_t getOutCnt(); //!<number of outcoming messages
    uint16_t getErrCnt(); //!<error counter
    uint8_t getID(); //!<get device ID between 1 and 247
    CommState getState();
    Error getLastError(); //!<get last error value.
    void setLastError(Error errcode); //!<set last error value.
    void setID( uint8_t u8id ); //!<write new ID for the device
    void end(); //!<finish any communication and release serial communication port
    bool isHost(void) const { return this->u8id == 0; }
    bool isIdle(void) const { return this->u8state == CommState::IDLE; }
};

} // namespace McciCatena

#endif // _ModbusRtuV2_h_
