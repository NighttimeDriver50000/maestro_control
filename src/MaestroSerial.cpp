// Copyright 2017 Chris McKinney, Sam Dauchert, and University of South Carolina.
// All rights reserved.

#include "MaestroSerial.hpp"
#include <errno.h>
#include <error.h>
#include <fcntl.h>
#include <unistd.h>

// PUBLIC //

void MaestroSerial::open(const char *filename) {
    fd = ::open(filename, O_RDWR | O_NOCTTY | O_SYNC);
    is_open = true;
}

void MaestroSerial::write(const char *bytes, size_t count) {
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

void MaestroSerial::read(char *bytes, size_t count) {
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
