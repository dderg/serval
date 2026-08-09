// Helper functions for C / Python interface
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <errno.h> // errno
#include <stdarg.h> // va_start
#include <stdio.h> // fprintf
#include <string.h> // strerror
#include <time.h> // struct timespec
#include "compiler.h" // __visible
#include "pyhelper.h" // get_monotonic

// Return the monotonic system time as a double
double __visible
get_monotonic(void)
{
    struct timespec ts;
    int ret = clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    if (ret) {
        report_errno("clock_gettime", ret);
        return 0.;
    }
    return (double)ts.tv_sec + (double)ts.tv_nsec * .000000001;
}

static void
default_logger(const char *msg)
{
    fprintf(stderr, "%s\n", msg);
}

static void (*python_logging_callback)(const char *msg) = default_logger;

void __visible
set_python_logging_callback(void (*func)(const char *))
{
    python_logging_callback = func;
}

// Log an error message
void
errorf(const char *fmt, ...)
{
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    buf[sizeof(buf)-1] = '\0';
    python_logging_callback(buf);
}

// Report 'errno' in a message written to stderr
void
report_errno(char *where, int rc)
{
    int e = errno;
    errorf("Got error %d in %s: (%d)%s", rc, where, e, strerror(e));
}
