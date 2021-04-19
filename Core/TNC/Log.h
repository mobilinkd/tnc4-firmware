// Copyright 2015 Mobilinkd LLC <rob@mobilinkd.com>
// All rights reserved.

#ifndef MOBILINKD__TNC_LOG_HPP_
#define MOBILINKD__TNC_LOG_HPP_

#ifdef __cplusplus
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

extern "C" {
#else
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#endif

void log_(int level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

#ifndef KISS_LOG_LEVEL
#define KISS_LOG_LEVEL 1
#endif

#ifdef KISS_LOGGING

#define LOG(level, ...) if(level >= KISS_LOG_LEVEL) log_(level, __VA_ARGS__);

#define TNC_DEBUG(...)    LOG(0, __VA_ARGS__)
#define INFO(...)     LOG(1, __VA_ARGS__)
#define WARN(...)     LOG(2, __VA_ARGS__)
#define ERROR(...)    LOG(3, __VA_ARGS__)
#define SEVERE(...)   LOG(4, __VA_ARGS__)
#else
#define TNC_DEBUG(...)
#define INFO(...)
#define WARN(...)
#define ERROR(...)
#define SEVERE(...)
#endif

#ifdef __cplusplus
}

namespace mobilinkd { namespace tnc {

#define APP_TX_DATA_SIZE 64

struct Log {
    enum Level {debug = 0, info, warn, error, severe};

    Level level_;

    Log()
    : level_(Level::debug)
    {}

    Log(Level level)
    : level_(level)
    {}

    void setLevel(Level level) {level_ = level;}

    void log(Level level, const char *fmt, ...);
};

Log& log(void);

}} // mobilinkd::tnc

#endif // __cplusplus

#endif // MOBILINKD__TNC_LOG_HPP_
