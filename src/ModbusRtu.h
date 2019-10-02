/**
 * @file        ModbusRtu.h
 * @version     0.3.0
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

#include "ModbusRtuV2.h"

// Versioning information.

/**
 * @struct modbus_t
 * @brief
 * host query structure:
 * This includes all the necessary fields to make the host generate a Modbus query.
 * A host may keep several of these structures and send them cyclically or
 * use them according to program needs.
 */
using modbus_t = McciCatena::modbus_datagram_t;

enum
{
    RESPONSE_SIZE = unsigned(McciCatena::ModbusMessageSize::RESPONSE),
    EXCEPTION_SIZE = unsigned(McciCatena::ModbusMessageSize::EXCEPTION),
    CHECKSUM_SIZE = unsigned(McciCatena::ModbusMessageSize::CHECKSUM),
};

/**
 * @enum MESSAGE
 * @brief
 * Indices to datagram frame positions
 */
enum MESSAGE
{
    ID          = int(McciCatena::ModbusMessageOffset::ID),           //!< Index of ID field
    FUNC        = int(McciCatena::ModbusMessageOffset::FUNC),         //!< Index of Function code
    ADD_HI      = int(McciCatena::ModbusMessageOffset::ADD_HI),       //!< Index of Address high byte
    ADD_LO      = int(McciCatena::ModbusMessageOffset::ADD_LO),       //!< Index of Address low byte
    NB_HI       = int(McciCatena::ModbusMessageOffset::NB_HI),        //!< Index of Number of coils or registers high byte
    NB_LO       = int(McciCatena::ModbusMessageOffset::NB_LO),        //!< Index of Number of coils or registers low byte
    BYTE_CNT    = int(McciCatena::ModbusMessageOffset::BYTE_CNT),     //!< Index of byte counter
    EXCEPTION   = int(McciCatena::ModbusMessageOffset::EXCEPTION),    //!< Index in exception response of exception code.
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
    MB_FC_NONE                     = int(McciCatena::ModbusFunction::NONE),                 /*!< null operator */
    MB_FC_READ_COILS               = int(McciCatena::ModbusFunction::READ_COILS),           /*!< FCT=1 -> read coils or digital outputs */
    MB_FC_READ_DISCRETE_INPUT      = int(McciCatena::ModbusFunction::READ_DISCRETE_INPUT),  /*!< FCT=2 -> read digital inputs */
    MB_FC_READ_REGISTERS           = int(McciCatena::ModbusFunction::READ_REGISTERS),       /*!< FCT=3 -> read registers or analog outputs */
    MB_FC_READ_INPUT_REGISTER      = int(McciCatena::ModbusFunction::READ_INPUT_REGISTER),  /*!< FCT=4 -> read analog inputs */
    MB_FC_WRITE_COIL               = int(McciCatena::ModbusFunction::WRITE_COIL),           /*!< FCT=5 -> write single coil or output */
    MB_FC_WRITE_REGISTER           = int(McciCatena::ModbusFunction::WRITE_REGISTER),       /*!< FCT=6 -> write single register */
    MB_FC_WRITE_MULTIPLE_COILS     = int(McciCatena::ModbusFunction::WRITE_MULTIPLE_COILS), /*!< FCT=15 -> write multiple coils or outputs */
    MB_FC_WRITE_MULTIPLE_REGISTERS = int(McciCatena::ModbusFunction::WRITE_MULTIPLE_REGISTERS),/*!< FCT=16 -> write multiple registers */
};

enum COMM_STATES
{
    COM_IDLE                     = int(McciCatena::Modbus::CommState::IDLE),
    COM_WAITING                  = int(McciCatena::Modbus::CommState::WAITING),
};

enum ERR_LIST : int16_t
{
    ERR_SUCCESS                   = int16_t(McciCatena::Modbus::Error::SUCCESS),
    ERR_NOT_HOST                  = int16_t(McciCatena::Modbus::Error::NOT_HOST),
    ERR_POLLING                   = int16_t(McciCatena::Modbus::Error::POLLING),
    ERR_BUFF_OVERFLOW             = int16_t(McciCatena::Modbus::Error::BUFF_OVERFLOW),
    ERR_BAD_CRC                   = int16_t(McciCatena::Modbus::Error::BAD_CRC),
    ERR_EXCEPTION                 = int16_t(McciCatena::Modbus::Error::EXCEPTION),
    ERR_NO_REPLY                  = int16_t(McciCatena::Modbus::Error::NO_REPLY),
    ERR_RUNT_PACKET               = int16_t(McciCatena::Modbus::Error::RUNT_PACKET),
    ERR_ILLEGAL_DEVICE_ADDRESS    = int16_t(McciCatena::Modbus::Error::ILLEGAL_DEVICE_ADDRESS),
};

enum
{
    EXC_FUNC_CODE                 = int(McciCatena::ModbusException_t::ILLEGAL_FUNCTION),
    EXC_ADDR_RANGE                = int(McciCatena::ModbusException_t::ILLEGAL_DATA_ADDRESS),
    EXC_REGS_QUANT                = int(McciCatena::ModbusException_t::ILLEGAL_DATA_VALUE),
    EXC_EXECUTE                   = int(McciCatena::ModbusException_t::SERVER_DEVICE_FAILURE),
};

enum MB_EXCEPTION: uint8_t
{
    MB_EXC_ILLEGAL_FUNCTION        = uint8_t(McciCatena::ModbusException_t::ILLEGAL_FUNCTION),
    MB_EXC_ILLEGAL_DATA_ADDRESS    = uint8_t(McciCatena::ModbusException_t::ILLEGAL_DATA_ADDRESS),
    MB_EXC_ILLEGAL_DATA_VALUE      = uint8_t(McciCatena::ModbusException_t::ILLEGAL_DATA_VALUE),
    MB_EXC_SERVER_DEVICE_FAILURE   = uint8_t(McciCatena::ModbusException_t::SERVER_DEVICE_FAILURE),
    MB_EXC_ACKNOWLEDGE             = uint8_t(McciCatena::ModbusException_t::ACKNOWLEDGE),
    MB_EXC_SERVER_DEVICE_BUSY      = uint8_t(McciCatena::ModbusException_t::SERVER_DEVICE_BUSY),
    MB_MEMORY_PARITY_ERROR         = uint8_t(McciCatena::ModbusException_t::MEMORY_PARITY_ERROR),
    MB_GATEWAY_PATH_UNAVAILABLE    = uint8_t(McciCatena::ModbusException_t::GATEWAY_PATH_UNAVAILABLE),
    MB_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = uint8_t(McciCatena::ModbusException_t::GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND),
};

#define T35         (McciCatenea::Modbus::kT35)
#define MAX_BUFFER  (McciCatena::Modbus::kMaxBuffer)    //!< maximum size for the communication buffer in bytes

class Modbus : public McciCatena::Modbus
    {
private:
    using Super = McciCatena::Modbus;

public:
    // constructors
    Modbus() {}
    Modbus(uint8_t u8id)
        : Super::Modbus(u8id) 
        {}
    Modbus(uint8_t u8id, uint8_t u8txenpin)
        : Super::Modbus(u8id, u8txenpin) 
        {}

//  void begin(ModbusPort *pPort, unsigned long u32speed)
//      { this->Super::begin(port, )};
//  void begin(ModbusPort *pPort, unsigned long u32speed, uint16_t u8config);
//  void setTimeOut( uint16_t u16timeout); //!<write communication watch-dog timer
//  void setTxEnableDelay(uint16_t u16txen_us); //!<set tx enable delay in us
//  uint16_t getTimeOut(); //!<get communication watch-dog timer value
//  boolean getTimeOutState(); //!<get communication watch-dog timer state
    ERR_LIST query( modbus_t telegram ) //!<only for host
        {
        return ERR_LIST(this->Super::query(telegram));
        }
    int16_t poll() //!<cyclic poll for host
        {
        return int16_t(this->Super::poll());
        }
    int8_t poll( uint16_t *regs, uint8_t u8size ) //!<cyclic poll for device
        {
        return this->Super::poll(regs, u8size);
        }
//  uint16_t getInCnt(); //!<number of incoming messages
//  uint16_t getOutCnt(); //!<number of outcoming messages
//  uint16_t getErrCnt(); //!<error counter
//  uint8_t getID(); //!<get device ID between 1 and 247
    uint8_t getState()
        {
        return uint8_t(this->Super::getState());
        }
    ERR_LIST getLastError() //!<get last error value.
        {
        return ERR_LIST(this->Super::getLastError());
        }
    void setLastError(ERR_LIST errcode) //!<set last error value.
        {
        this->Super::setLastError(Error(errcode));
        }
//  void setID( uint8_t u8id ); //!<write new ID for the device
//  void end(); //!<finish any communication and release serial communication port
    };
