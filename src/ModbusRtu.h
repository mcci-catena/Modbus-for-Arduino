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

constexpr auto    RESPONSE_SIZE = unsigned(McciCatena::ModbusMessageSize::RESPONSE);
constexpr auto    EXCEPTION_SIZE = unsigned(McciCatena::ModbusMessageSize::EXCEPTION);
constexpr auto    CHECKSUM_SIZE = unsigned(McciCatena::ModbusMessageSize::CHECKSUM);


/**
 * @enum MESSAGE
 * @brief
 * Indexes to telegram frame positions
 */
using MESSAGE = McciCatena::ModbusMessageOffset;

constexpr auto ID          = McciCatena::ModbusMessageOffset::ID;            //!< Index of ID field
constexpr auto FUNC        = McciCatena::ModbusMessageOffset::FUNC;          //!< Index of Function code
constexpr auto ADD_HI      = McciCatena::ModbusMessageOffset::ADD_HI;        //!< Index of Address high byte
constexpr auto ADD_LO      = McciCatena::ModbusMessageOffset::ADD_LO;        //!< Index of Address low byte
constexpr auto NB_HI       = McciCatena::ModbusMessageOffset::NB_HI;         //!< Index of Number of coils or registers high byte
constexpr auto NB_LO       = McciCatena::ModbusMessageOffset::NB_LO;         //!< Index of Number of coils or registers low byte
constexpr auto BYTE_CNT    = McciCatena::ModbusMessageOffset::BYTE_CNT;      //!< Index of byte counter
constexpr auto EXCEPTION   = McciCatena::ModbusMessageOffset::EXCEPTION;     //!< Index in exception response of exception code.

/**
 * @enum MB_FC
 * @brief
 * Modbus function codes summary.
 * These are the implement function codes either for host or for device.
 *
 * @see also fctsupported
 * @see also modbus_t
 */
using MB_FC = McciCatena::ModbusFunction;

constexpr auto MB_FC_NONE                     = McciCatena::ModbusFunction::NONE;                  /*!< null operator */
constexpr auto MB_FC_READ_COILS               = McciCatena::ModbusFunction::READ_COILS;            /*!< FCT=1 -> read coils or digital outputs */
constexpr auto MB_FC_READ_DISCRETE_INPUT      = McciCatena::ModbusFunction::READ_DISCRETE_INPUT;   /*!< FCT=2 -> read digital inputs */
constexpr auto MB_FC_READ_REGISTERS           = McciCatena::ModbusFunction::READ_REGISTERS;        /*!< FCT=3 -> read registers or analog outputs */
constexpr auto MB_FC_READ_INPUT_REGISTER      = McciCatena::ModbusFunction::READ_INPUT_REGISTER;   /*!< FCT=4 -> read analog inputs */
constexpr auto MB_FC_WRITE_COIL               = McciCatena::ModbusFunction::WRITE_COIL;            /*!< FCT=5 -> write single coil or output */
constexpr auto MB_FC_WRITE_REGISTER           = McciCatena::ModbusFunction::WRITE_REGISTER;        /*!< FCT=6 -> write single register */
constexpr auto MB_FC_WRITE_MULTIPLE_COILS     = McciCatena::ModbusFunction::WRITE_MULTIPLE_COILS;  /*!< FCT=15 -> write multiple coils or outputs */
constexpr auto MB_FC_WRITE_MULTIPLE_REGISTERS = McciCatena::ModbusFunction::WRITE_MULTIPLE_REGISTERS; /*!< FCT=16 -> write multiple registers */

using COMM_STATES = McciCatena::Modbus::CommState;

constexpr auto COM_IDLE                     = McciCatena::Modbus::CommState::IDLE;
constexpr auto COM_WAITING                  = McciCatena::Modbus::CommState::WAITING;

using ERR_LIST = McciCatena::Modbus::Error;

constexpr auto ERR_SUCCESS                   = McciCatena::Modbus::Error::SUCCESS;
constexpr auto ERR_NOT_HOST                  = McciCatena::Modbus::Error::NOT_HOST;
constexpr auto ERR_POLLING                   = McciCatena::Modbus::Error::POLLING;
constexpr auto ERR_BUFF_OVERFLOW             = McciCatena::Modbus::Error::BUFF_OVERFLOW;
constexpr auto ERR_BAD_CRC                   = McciCatena::Modbus::Error::BAD_CRC;
constexpr auto ERR_EXCEPTION                 = McciCatena::Modbus::Error::EXCEPTION;
constexpr auto ERR_NO_REPLY                  = McciCatena::Modbus::Error::NO_REPLY;
constexpr auto ERR_RUNT_PACKET               = McciCatena::Modbus::Error::RUNT_PACKET;
constexpr auto ERR_ILLEGAL_DEVICE_ADDRESS    = McciCatena::Modbus::Error::ILLEGAL_DEVICE_ADDRESS;

constexpr auto EXC_FUNC_CODE        = McciCatena::ModbusException_t::ILLEGAL_FUNCTION;
constexpr auto EXC_ADDR_RANGE       = McciCatena::ModbusException_t::ILLEGAL_DATA_ADDRESS;
constexpr auto EXC_REGS_QUANT       = McciCatena::ModbusException_t::ILLEGAL_DATA_VALUE;
constexpr auto EXC_EXECUTE          = McciCatena::ModbusException_t::SERVER_DEVICE_FAILURE;

using MB_EXCEPTION = McciCatena::ModbusException_t;

constexpr auto MB_EXC_ILLEGAL_FUNCTION        = McciCatena::ModbusException_t::ILLEGAL_FUNCTION;
constexpr auto MB_EXC_ILLEGAL_DATA_ADDRESS    = McciCatena::ModbusException_t::ILLEGAL_DATA_ADDRESS;
constexpr auto MB_EXC_ILLEGAL_DATA_VALUE      = McciCatena::ModbusException_t::ILLEGAL_DATA_VALUE;
constexpr auto MB_EXC_SERVER_DEVICE_FAILURE   = McciCatena::ModbusException_t::SERVER_DEVICE_FAILURE;
constexpr auto MB_EXC_ACKNOWLEDGE             = McciCatena::ModbusException_t::ACKNOWLEDGE;
constexpr auto MB_EXC_SERVER_DEVICE_BUSY      = McciCatena::ModbusException_t::SERVER_DEVICE_BUSY;
constexpr auto MB_MEMORY_PARITY_ERROR         = McciCatena::ModbusException_t::MEMORY_PARITY_ERROR;
constexpr auto MB_GATEWAY_PATH_UNAVAILABLE    = McciCatena::ModbusException_t::GATEWAY_PATH_UNAVAILABLE;
constexpr auto MB_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = McciCatena::ModbusException_t::GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND;

#define T35         (McciCatenea::Modbus::kT35)
#define MAX_BUFFER  (McciCatena::Modbus::kMaxBuffer)    //!< maximum size for the communication buffer in bytes

using Modbus = McciCatena::Modbus;
