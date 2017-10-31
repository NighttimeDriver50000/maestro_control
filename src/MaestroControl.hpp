#ifndef MAESTRO_CONTROL_HPP
#define MAESTRO_CONTROL_HPP
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <vector>

using namespace std;

struct Target {
    uint8_t channel;
    float target_usec;
};

bool operator<(const struct Target &a, const struct Target &b);

void toLowerCase(string &s);

class MaestroControl {
public:
    // Connect to the device file for the Maestro
    void open(const char *filename);
    // Set the target for a single servo
    // For digital outputs, >=1500 is high, <1500 is low
    void setTarget(uint8_t channel, float target_usec);
    void setTarget(struct Target target);
    // Set the target for multiple servos
    void setMultiTarget(vector<struct Target> targets);
    void setRangeTargets(uint8_t start_channel, uint8_t channel_bound,
            float target);
    // Set all channels to zero
    void zeroTargets(uint8_t channel_count);
    // Set the speed with which a servo moves to targets
    void setSpeed(uint8_t channel, float speed_usec_per_10ms);
    // Set the acceleration with which a servo moves to targets
    void setAcceleration(uint8_t channel, float speed_usec_per_800sqms);
    // Set the PWM output for a Maestro Mini 12, 18, or 24
    void setPWM(float on_time_usec, float period_usec);
    // Get the current position of a servo in microseconds
    // For digital outputs, >=1500 is high, <1500 is low
    // For analog inputs, value ranges from 0 to 255.75
    // For digital inputs, value is either 0 or 255.75
    float getPosition(uint8_t channel);
    // Get whether servos are moving
    bool isMoving();
    // Move all servos to home positions
    void goHome();
    // Get the error bits
    uint16_t getErrors();
    // Print a list of errors
    void printErrors(uint16_t bits, ostream &out);
    // Execute commands from a script or interactively
    void shell(istream &in, ostream &out, bool interactive);
    // Execute a single command
    void executeShellCommand(string command, vector<string> arguments,
            ostream &out);
    // Flush commands to the Maestro
    void flush();

private:
    fstream serial;

    // Prints the error flags of the serial object
    void printSerialFlags();
    // Sends bytes to the Maestro
    inline void rawWrite(const uint8_t bytes_array[]);
    void rawWrite(const vector<uint8_t> &bytes);
    // Receives queued bytes from the Maestro
    vector<uint8_t> *rawRead(size_t count);
    // Convert a floating-point microseconds value to a 14-bit unsigned integer
    // quarter-microseconds value, stored little-endian in two 7-bit bytes
    vector<uint8_t> *formatQuarterUSec(float usec);
    // Convert two little-endian bytes to an unsigned 16-bit integer
    uint16_t deformatUint16(vector<uint8_t> &little_endian);
    // Unfiltered method for setMultiTarget (unsafe)
    void setMultiTarget(vector<struct Target> targets, size_t start_index,
            size_t index_bound);
    // Base method for setMultiTarget (all others call this)
    void setMultiTarget(uint8_t start_channel, vector<float> targets_usec);
};
#endif
