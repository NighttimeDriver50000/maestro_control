// Copyright 2017 Chris McKinney, Sam Dauchert, and University of South Carolina.
// All rights reserved.

#ifndef MAESTRO_SERIAL_HPP
#define MAESTRO_SERIAL_HPP
#include <stddef.h>
#include <unistd.h>

using namespace std;

class MaestroSerial {
public:
    void open(const char *filename);
    void write(const char *bytes, ssize_t count);
    void read(char *bytes, ssize_t count);
    int getError();
    void printError();
    void clearError();
private:
    int fd;
    bool is_open;
    int error_code;
};

#endif
