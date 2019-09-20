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

using cCatenaModbusRtu = cCatenaModbusRtuHost;

}; // end namespace McciCatena

#endif /* _CATENA_MODBUSRTU_H_ */
