/*

Module:  Catena_ModbusRtuHost.h

Function:
        Wrapper for ModbusRtu object (as host) that is pollable.

Copyright notice and License:
        See LICENSE file accompanying this project.

Author:
        Terry Moore, MCCI Corporation   July 2018

*/

#pragma once

#ifndef _Catena_ModbusRtuHost_h_
#define _Catena_ModbusRtuHost_h_

#include <Catena_PollableInterface.h>
#include "ModbusRtuV2.h"

namespace McciCatena {

class cModbusDatagram;

extern "C" {
    typedef void (CatenaModbusRtu_DatagramCb_t)(void *pClientInfo, cModbusDatagram *pDatagram, Modbus::Error status);
}

class cModbusDatagram : public modbus_datagram_t
    {
public:
    // neither copyable nor movable.
    cModbusDatagram(const cModbusDatagram&) = delete;
    cModbusDatagram& operator=(const cModbusDatagram&) = delete;
    cModbusDatagram(const cModbusDatagram&&) = delete;
    cModbusDatagram& operator=(const cModbusDatagram&&) = delete;

    // default constructor
    cModbusDatagram() {}
    /// set the callback info.
    inline void setDatagramCallback(CatenaModbusRtu_DatagramCb_t *pDoneFn, void *pDoneInfo)
        {
        this->m_pDoneFn = pDoneFn;
        this->m_pDoneInfo = pDoneInfo;
        }

    /// insert a datagram into a queue.
    inline void queue(cModbusDatagram * &pHead, CatenaModbusRtu_DatagramCb_t *pDoneFn, void *pDoneInfo)
        {
        this->setDatagramCallback(pDoneFn, pDoneInfo);
        this->queue(pHead);
        }

    inline void queue(cModbusDatagram * &pHead)
        {
        if (pHead == nullptr)
            {
            pHead = this->m_pNext = this->m_pLast = this;
            }
        else
            {
            auto pTail = pHead->m_pLast;
            this->m_pNext = pHead;
            this->m_pLast = pTail;
            pTail->m_pNext = pHead->m_pLast = this;
            }
        }

    /// remove a datagram from a queue
    inline void unlink(cModbusDatagram * &pHead)
        {
        auto const pNext = this->m_pNext;
        if (pNext == this)
            pHead = nullptr;
        else
            {
            auto pLast = this->m_pLast;
            pLast->m_pNext = pNext;
            pNext->m_pLast = pLast;
            pHead = pNext;
            this->m_pNext = this->m_pLast = this;
            }
        }

    inline void complete(Modbus::Error status)
        {
        this->m_status = status;
        (*this->m_pDoneFn)(this->m_pDoneInfo, this, status);
        }

private:
    cModbusDatagram *m_pNext;
    cModbusDatagram *m_pLast;
    CatenaModbusRtu_DatagramCb_t *m_pDoneFn;
    void *m_pDoneInfo;
    Modbus::Error    m_status;
    };

class cModbusDatagramQueue
    {
public:
    // neither copyable nor movable.
    cModbusDatagramQueue(const cModbusDatagramQueue&) = delete;
    cModbusDatagramQueue& operator=(const cModbusDatagramQueue&) = delete;
    cModbusDatagramQueue(const cModbusDatagramQueue&&) = delete;
    cModbusDatagramQueue& operator=(const cModbusDatagramQueue&&) = delete;

    // default constructor
    cModbusDatagramQueue() {}

    inline void putTail(cModbusDatagram *pDatagram)
        {
        pDatagram->queue(this->m_pHead);
        }

    inline void putTail(cModbusDatagram *pDatagram, CatenaModbusRtu_DatagramCb_t *pDoneFn, void *pDoneInfo)
        {
        pDatagram->queue(this->m_pHead, pDoneFn, pDoneInfo);
        }

    inline cModbusDatagram *getFirst()
        {
        auto const pResult = this->m_pHead;
        if (pResult != nullptr)
            pResult->unlink(this->m_pHead);
        return pResult;
        }

    inline cModbusDatagram *peekFirst() const
        {
        return this->m_pHead;
        }

private:
    cModbusDatagram *m_pHead = nullptr;
    };

/*

Class:  cCatenaModbusRtuHost

Function:
    Event-driven, queued wrapper for ModbusRtu.

Description:
    This object encapsulates a Modbus object, adding:

    * polling in the Catena framework.
    * Queued host operations

    This object also uses enhanced Modbus datagram
    objects, which extend the base datagram type
    by adding queueing links, slots for callbacks
    and callback info. (But these are only useful
    if the Modbus instance is configured as a host.)

    Remember to register this with catena framework at startup, e.g.:

        gCatena.registerObject(&myCatenaModbusRtuHost);

    After that, gCatena.poll() will include the object
    in the poll sequence.

*/

class cCatenaModbusRtuHost : public Modbus,
             public cPollableObject
    {
    using Super = Modbus;
public:
    // neither copyable nor movable.
    cCatenaModbusRtuHost(const cCatenaModbusRtuHost&) = delete;
    cCatenaModbusRtuHost& operator=(const cCatenaModbusRtuHost&) = delete;
    cCatenaModbusRtuHost(const cCatenaModbusRtuHost&&) = delete;
    cCatenaModbusRtuHost& operator=(const cCatenaModbusRtuHost&&) = delete;

    cCatenaModbusRtuHost() {};
    cCatenaModbusRtuHost(uint8_t u8id) : Super(u8id) {};
    cCatenaModbusRtuHost(uint8_t u8id, uint8_t u8txenpin) : Super(u8id, u8txenpin) {};

    /**
     * @brief
     * Handle polling for cCatenaModbusRtuHost objects.
     *
     * We call the low-level host poll routine. If the poll result is other
     * than Error::SUCCESS, and there's a datagram pending, we complete it.
     * If the low-level host is now idle, and there's a datagram pending,
     * promote it.
     *
     * @note This is in the header file due to Arduino build-system
     * limitations.
     */
    virtual void poll()
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

    Error getPollResult() const { return this->m_lastPollResult; }

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
     *
     * @note This is implemented in the header file to work around Arduino
     *      build-system limitations.
     */
    void queue(
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

private:
    // the queue of datagrams waiting to be serviced.
    cModbusDatagramQueue m_PendingQueue;
    // the queue; if not empty, this is the current datagram.
    cModbusDatagramQueue m_ActiveQueue;
    // saved result from calling lower layer.
    Error m_lastPollResult = Error(0);
    };

}; // end namespace McciCatena

#endif /* _Catena_ModbusRtuHost_h_ */
