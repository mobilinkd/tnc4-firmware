// Copyright 2016-2021 Rob Riggs <rob@mobilinkd.com>
// All rights reserved.

#pragma once

#include "PortInterface.hpp"

#include <atomic>

namespace mobilinkd { namespace tnc {

/**
 * This interface defines the semi-asynchronous interface used for reading
 * and writing
 */
struct NullPort : PortInterface
{
    virtual ~NullPort() {}
    virtual bool open()
    {
        if (open_) return open_;
        open_ = true;
        return open_;
    }

    virtual bool isOpen() const { return open_; }

    virtual void close() {
        open_ = false;
    }
    virtual osMessageQueueId_t queue() const { return 0; }
    virtual bool write(const uint8_t*, uint32_t, uint8_t, uint32_t)
    {
        return true;
    }
    virtual bool write(const uint8_t*, uint32_t, uint32_t)
    {
        return true;
    }
    virtual bool write(hdlc::IoFrame* frame, uint32_t = osWaitForever)
    {
        hdlc::release(frame);
        return true;
    }

    void init() {}

    std::atomic<bool> open_{false};
};

NullPort* getNullPort();

}} // mobilinkd::tnc
