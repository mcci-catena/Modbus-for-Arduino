/*

Module: cCatenaModbusRtuHost_queue.cpp

Function:
    Home of cCatenaModubusRtu::queue()

Copyright notice and License:
    See LICENSE file accompanying this project.

Author:
    Terry Moore, MCCI Corporation   September 2019

*/


#include "../Catena_ModbusRtuHost.h"

using namespace McciCatena;

/**
 * @brief
 * Submit a datagram for asynchronous processing by the Modbus
 * stack.
 * 
 * The datagram is enqueued for processing by the Modbus stack. The library
 * guarantees that the call-back function is called when processing of the
 * datagram is complete.
 *
 * @param pDatagram datagram to be submitted. If null, an invalid parameter
 *                  error will be reported.
 * @param pCb       function to be called when processing is complete. If this is
 *                  null, then this function call is completely ignored.
 * @param pClientInfo  arbitrary data passed to the callback function.
 */

void
cCatenaModbusRtuHost::queue(
    cModbusDatagram *pDatagram, 
    CatenaModbusRtu_DatagramCb_t *pCb, 
    void *pClientInfo
    )
    {
    // check the parameter.
    if (pCb == nullptr)
        return;

    // make sure we're configured as a host.
    if (! this->isHost())
        {
        (pCb)(pClientInfo, pDatagram, Error::NOT_HOST);
        return;
        }

    // if there's anybody ahead of us, wait in line.
    if (this->m_PendingQueue.peekFirst() != nullptr)
        {
        this->m_PendingQueue.putTail(pDatagram, pCb, pClientInfo);
        return;
        }

    // try to immediately submit.
    auto const eSubmit = this->query(*pDatagram);

    // it might have worked, or it might have been rejected.
    switch (eSubmit)
        {
    // it worked.
    case Error::SUCCESS:
        // put the datagram in the active "queue"
        this->m_ActiveQueue.putTail(pDatagram, pCb, pClientInfo);
        break;

    // rejected because the lower level is busy.
    case Error::POLLING:
        // put the datagram in the pending queue
        this->m_PendingQueue.putTail(pDatagram, pCb, pClientInfo);
        break;

    // rejected for some other reason (invalid address, etc.)
    default:
        (*pCb)(pClientInfo, pDatagram, eSubmit);
        break;
        }
    }
