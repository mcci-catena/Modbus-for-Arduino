/**
* @file         ModbusPort.h
* @version      0
* @date         2018-04-21
* @author       Terry Moore
* @contact      tmm@mcci.com
*
* @description
*       This file defines the ModbusPort object, which abstracts the
*       various serial port types for the ModbusRtu library.
*
*/

#pragma once

#ifndef _MODBUSPORT_H_
# define _MODBUSPORT_H_

#include <inttypes.h>

class ModbusPort
        {
public:
        // constructor
        ModbusPort() {};

        // neither copyable nor movable.
        ModbusPort(const ModbusPort&) = delete;
        ModbusPort& operator=(const ModbusPort&) = delete;
        ModbusPort(const ModbusPort&&) = delete;
        ModbusPort& operator=(const ModbusPort&&) = delete;
        
        virtual void begin(unsigned long baudrate) const = 0;
        virtual void begin(unsigned long baudrate, uint16_t config) const = 0;
        virtual int available() const = 0;
        virtual int read() const = 0;
        virtual size_t write(const uint8_t *buffer, size_t size) const = 0;

        virtual void drainRead() const
                {
                while (this->read() >= 0)
                        /* discard */;
                }
        virtual void drainWrite() const = 0;
        };

template <class T> class ModbusSerial : public ModbusPort
        {
public:
        // constructor
        ModbusSerial(T *pPort) : m_pPort(pPort) {};

        // neither copyable nor movable.
        ModbusSerial(const ModbusSerial&) = delete;
        ModbusSerial& operator=(const ModbusSerial&) = delete;
        ModbusSerial(const ModbusSerial&&) = delete;
        ModbusSerial& operator=(const ModbusSerial&&) = delete;

        virtual int available() const override
                {
                return this->m_pPort->available();
                }

        virtual void begin(unsigned long ulBaudRate) const override
                {
                this->m_pPort->begin(ulBaudRate);
                }

        virtual void begin(unsigned long ulBaudRate, uint16_t config) const override
                {
                this->m_pPort->begin(ulBaudRate, config);
                }

        virtual void drainWrite() const override
                {
                this->m_pPort->flush();
                }

        virtual int read() const override
                {
                return this->m_pPort->read();
                }

        virtual size_t write(const uint8_t *buffer, size_t size) const override
                {
                return this->m_pPort->write(buffer, size);
                }

private:
        T *m_pPort;
        };

#endif /* _MODBUSPORT_H_ */
