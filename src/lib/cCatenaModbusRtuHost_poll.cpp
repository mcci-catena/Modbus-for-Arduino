/*

Module: cCatenaModbusRtuHost_poll.cpp

Function:
    Home of cCatenaModubusRtuHost::poll()

Copyright notice and License:
    See LICENSE file accompanying this project.

Author:
    Terry Moore, MCCI Corporation   September 2019

*/


#include "../Catena_ModbusRtuHost.h"

using namespace McciCatena;


/**
 * @brief
 * Handle polling for cCatenaModbusRtuHost objects.
 * 
 * We call the low-level host poll routine. If the poll result is other
 * than Error::SUCCESS, and there's a datagram pending, we complete it.
 * If the low-level host is now idle, and there's a 
 */

void
cCatenaModbusRtuHost::poll(void)
    {
    auto const pollResult = this->m_lastPollResult = this->Super::poll();

    // now... deal with promoting the queue.
    // the poll result is != 0 for completion of the active datagram.
    if (pollResult != Error::SUCCESS)
        {
        // if the current datagram represents a queued datagram, 
        // pull the datagram off the queue and complete.
        auto const pDatagram = this->m_ActiveQueue.getFirst();

        if (pDatagram != NULL)
            pDatagram->complete(pollResult < Error::SUCCESS ? pollResult : Error::SUCCESS);
        }

    // now.... if not busy, and a datagram is pending, post it.
    if (this->isIdle())
        {
        auto const pDatagram = this->m_PendingQueue.peekFirst();
        if (pDatagram != nullptr)
            {
            auto const eSubmit = this->query(* dynamic_cast<modbus_datagram_t *>(pDatagram));
            switch (eSubmit)
                {
            case Error::POLLING:
                break;
            case Error::SUCCESS:
                this->m_PendingQueue.getFirst();
                this->m_ActiveQueue.putTail(pDatagram);
                break;
            default:
                this->m_PendingQueue.getFirst();
                pDatagram->complete(eSubmit);
                }
            }
        }
    }
