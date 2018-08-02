// Copyright 2017 Chris McKinney, Sam Dauchert, and University of South Carolina.
// All rights reserved.

#include "MaestroSerial.hpp"
#include <errno.h>
#include <error.h>
#include <fcntl.h>

#include <cstdio>

// PUBLIC //

void MaestroSerial::open(const char *filename) {
    fd = ::open(filename, O_RDWR | O_NOCTTY | O_SYNC
            | O_DSYNC | O_RSYNC);
    if (fd == -1) {
        error(0, errno, "Could not open %s", filename);
    }
    is_open = true;
    int fl = fcntl(fd, F_GETFL);
    printf("FD: 0%o, FL: 0%o, ACC: 0%o, O_ACCMODE: 0%o\n",
            fcntl(fd, F_GETFD), fl, fl & O_ACCMODE, O_ACCMODE);
    printf("APPEND: 0%o, DSYNC: 0%o, NONBLOCK: 0%o, RSYNC: 0%o, SYNC: 0%o\n",
            O_APPEND, O_DSYNC, O_NONBLOCK, O_RSYNC, O_SYNC);
}

void MaestroSerial::write(const char *bytes, ssize_t count) {
    if (is_open) {
        ssize_t actual = ::write(fd, bytes, count);
        error_code = errno;
        if (!error_code && actual != count) {
            error_code = EIO;
        }
    } else {
        error_code = EBADF;
    }
}

void MaestroSerial::read(char *bytes, ssize_t count) {
    if (is_open) {
        ssize_t actual = ::read(fd, bytes, count);
        error_code = errno;
        if (!error_code && actual != count) {
            error_code = EIO;
        }
    } else {
        error_code = EBADF;
    }
}

int MaestroSerial::getError() {
    return error_code;
}

void MaestroSerial::printError() {
    if (error_code) {
        error(0, error_code, "Serial Error");
    } else {
        error(0, 0, "OK");
    }
}

void MaestroSerial::clearError() {
    error_code = 0;
}
