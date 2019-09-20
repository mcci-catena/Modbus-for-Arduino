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

#ifndef _Catena_ModbusRtuHostV2_h_
#define _Catena_ModbusRtuHostV2_h_

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

    // the polling interface
    // we save the poll() results for the background, or
    // we might just want to look for completion other ways.
    virtual void poll();
    Error getPollResult() const { return this->m_lastPollResult; }

    // enqueue a request, and then start IO if needed.
    void queue(cModbusDatagram *pDatagram, CatenaModbusRtu_DatagramCb_t *pCb, void *pClientInfo);

private:
    cModbusDatagramQueue m_PendingQueue;
    cModbusDatagramQueue m_ActiveQueue;
    Error m_lastPollResult = Error(0);
    };

}; // end namespace McciCatena

#endif /* _Catena_ModbusRtuHostV2_h_ */
