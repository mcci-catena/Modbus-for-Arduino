/*

Module:  Catena_ModbusRtu.h

Function:
        Wrapper for ModbusRtu object that is pollable.

Copyright notice and License:
        See LICENSE file accompanying this project.

Author:
        Terry Moore, MCCI Corporation   July 2018

*/

#pragma once

#ifndef _CATENA_MODBUSRTU_H_
#define _CATENA_MODBUSRTU_H_

#include "Catena_ModbusRtuHost.h"
#include "ModbusRtu.h"

namespace McciCatena {

//
class cCatenaModbusRtu : public cCatenaModbusRtuHost
    {
private:
    using Super = cCatenaModbusRtuHost;

public:
    // constructors
    cCatenaModbusRtu() {}
    cCatenaModbusRtu(uint8_t u8id)
        : Super(u8id) 
        {}
    cCatenaModbusRtu(uint8_t u8id, uint8_t u8txenpin)
        : Super(u8id, u8txenpin) 
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
    int8_t poll( uint16_t *regs, uint8_t u8size ); //!<cyclic poll for device
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

}; // end namespace McciCatena

#endif /* _CATENA_MODBUSRTU_H_ */
