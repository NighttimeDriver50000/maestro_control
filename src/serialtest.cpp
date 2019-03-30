// Copyright 2017 Chris McKinney, Sam Dauchert, and University of South Carolina.
// All rights reserved.

#include <fstream>
#include <iostream>
#include <stdint.h>
#include <stdio.h>

using namespace std;

fstream serial;

void printSerialFlags() {
    cerr << "Serial state: good: " << serial.good() << ", eof: " << serial.eof()
            << ", fail: " << serial.fail() << ", bad: " << serial.bad() << "\n";
}

int main() {
    serial.open("/dev/ttyACM0", ios_base::in | ios_base::out | ios_base::binary);
    printSerialFlags();
    serial.clear();
    uint8_t geb[] = {0xA1};
    serial.write(reinterpret_cast<const char *>(geb), 1);
    printSerialFlags();
    char buffer[2];
    serial.read(buffer, 2);
    printSerialFlags();
    uint8_t *little_endian = reinterpret_cast<uint8_t *>(buffer);
    cerr << (static_cast<uint16_t>(little_endian[0]) |
            (static_cast<uint16_t>(little_endian[1]) << 8)) << "\n";
    uint8_t stb[] = {0x84, 0, 0, 0};
    printSerialFlags();
    serial.write(reinterpret_cast<const char *>(stb), 4);
    printSerialFlags();
}
