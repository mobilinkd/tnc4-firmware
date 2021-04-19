// Copyright 2016-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#include "PortInterface.hpp"

namespace mobilinkd { namespace tnc {

/**
 * This interface defines the semi-asynchronous interface used for reading
 * and writing
 */
struct SerialPort : PortInterface
{
    virtual ~SerialPort() {}
    virtual bool open();
    virtual bool isOpen() const { return open_; }
    virtual void close();
    virtual osMessageQueueId_t queue() const { return queue_; }
    virtual bool write(const uint8_t* data, uint32_t size, uint8_t type,
        uint32_t timeout);
    virtual bool write(const uint8_t* data, uint32_t size, uint32_t timeout);
    virtual bool write(hdlc::IoFrame* frame, uint32_t timeout = osWaitForever);

    void init();

private:
    bool open_{false};                  // opened/closed
    osMutexId_t mutex_{0};                // TX Mutex
    osMessageQueueId_t queue_{0};             // ISR read queue
    osThreadId_t serialTaskHandle_{0};

    bool abort_tx(hdlc::IoFrame* frame);
};

SerialPort* getSerialPort();

}} // mobilinkd::tnc
