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

#include <Catena_PollableInterface.h>
#include "ModbusRtu.h"

namespace McciCatena {

class cCatenaModbusRtu : public Modbus,
			 public cPollableObject
	{
	using Super = Modbus;
public:
	cCatenaModbusRtu() {};
	cCatenaModbusRtu(uint8_t u8id) : Super(u8id) {};
	cCatenaModbusRtu(uint8_t u8id, uint8_t u8txenpin) : Super(u8id, u8txenpin) {};

	// the polling interface
	// we save the poll() results for the background, or
	// we might just want to look for completion other ways.
	virtual void poll() { lastPollResult = this->Super::poll(); };
	Error getPollResult() const { return this->lastPollResult; }

private:
	Error lastPollResult = Error(0);
	};

// remember to register this with catena framework at startup, e.g.:
//	gCatena.registerObject(&myCatenaModbusRtu);
//
//	After that, gCatena.poll() will include the object in the poll sequence.
//
}; // end namespace McciCatena

#endif /* _CATENA_MODBUSRTU_H_ */
