// Copyright 2017 Chris McKinney, Sam Dauchert, and University of South Carolina.
// All rights reserved.

#include "MaestroControl.hpp"
#include <algorithm>
#include <unistd.h>
#include <string>

// FUNCTIONS //

bool operator<(const struct Target &a, const struct Target &b) {
    return a.channel < b.channel;
}

void toLowerCase(string &s) {
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
}

// PUBLIC //

void MaestroControl::open(const char *filename) {
    //serial.exceptions(fstream::badbit | fstream::failbit | fstream::eofbit);
    //serial.open(filename, ios_base::in | ios_base::out | ios_base::binary);
    serial.open(filename);
    printSerialFlags();
    //serial.clear();
    serial.clearError();
    //cerr << "Cleared serial state.\n";
}

void MaestroControl::setTarget(uint8_t channel, float target_usec) {
    vector<uint8_t> &target = *formatQuarterUSec(target_usec);
    uint8_t b[] = {0x84, channel, target[0], target[1]};
    rawWrite(b);
    delete &target;
}

void MaestroControl::setTarget(struct Target target) {
    setTarget(target.channel, target.target_usec);
}

void MaestroControl::setMultiTarget(vector<struct Target> targets) {
    // Sort the targets so that we can most efficiently split them
    sort(targets.begin(), targets.end());
    // Split the targets on breaks in continuity into separate commands
    uint8_t previous_channel = 255;
    uint8_t channel;
    size_t start_index = 0;
    for (size_t i = 0; i < targets.size(); ++i) {
        channel = targets[i].channel;
        if (channel != static_cast<uint16_t>(previous_channel) + 1) {
            // There is a break, send out a batch
            setMultiTarget(targets, start_index, i);
            start_index = i;
        }
        previous_channel = channel;
    }
    // Send whatever is left
    setMultiTarget(targets, start_index, targets.size());
}

void MaestroControl::setRangeTargets(uint8_t start_channel,
        uint8_t channel_bound, float target) {
    vector<float> targets_usec(channel_bound - start_channel, target);
    setMultiTarget(start_channel, targets_usec);
}

void MaestroControl::zeroTargets(uint8_t channel_count) {
    setRangeTargets(0, channel_count, 0);
}

void MaestroControl::setSpeed(uint8_t channel, float speed_usec_per_10ms) {
    vector<uint8_t> &speed = *formatQuarterUSec(speed_usec_per_10ms);
    uint8_t b[] = {0x87, channel, speed[0], speed[1]};
    rawWrite(b);
    delete &speed;
}

void MaestroControl::setAcceleration(uint8_t channel,
        float accel_usec_per_800sqms) {
    vector<uint8_t> &accel = *formatQuarterUSec(accel_usec_per_800sqms);
    uint8_t b[] = {0x89, channel, accel[0], accel[1]};
    rawWrite(b);
    delete &accel;
}

void MaestroControl::setPWM(float on_time_usec, float period_usec) {
    vector<uint8_t> &on_time = *formatQuarterUSec(on_time_usec * 12);
    vector<uint8_t> &period = *formatQuarterUSec(period_usec * 12);
    uint8_t b[] = {0x8A, on_time[0], on_time[1], period[0], period[1]};
    rawWrite(b);
    delete &on_time;
    delete &period;
}

float MaestroControl::getPosition(uint8_t channel) {
    uint8_t b[] = {0x90, channel};
    rawWrite(b);
    vector<uint8_t> &response = *rawRead(2);
    uint16_t position = deformatUint16(response);
    delete &response;
    return static_cast<float>(position) / 4;
}

bool MaestroControl::isMoving() {
    uint8_t b[] = {0x93};
    rawWrite(b);
    vector<uint8_t> &response = *rawRead(1);
    bool moving = (response[0] != 0x00);
    delete &response;
    return moving;
}

void MaestroControl::goHome() {
    uint8_t b[] = {0xA2};
    rawWrite(b);
}

uint16_t MaestroControl::getErrors() {
    uint8_t b[] = {0xA1};
    rawWrite(b);
    vector<uint8_t> &response = *rawRead(2);
    uint16_t errors = deformatUint16(response);
    delete &response;
    return errors;
}

void MaestroControl::printErrors(uint16_t bits, ostream &out) {
    uint8_t error_count = 0;
    for (uint8_t i = 0; i < 16; ++i) {
        if ((bits >> i) & 1) {
            ++error_count;
        }
    }
    out << "0x" << hex << bits << "\n";
    if (error_count == 0) {
        out << "No errors.\n";
    } else if (error_count == 1) {
        out << "The following error occurred:\n";
    } else {
        out << "The following errors occurred:\n";
    }
    if (bits & 1)
        out << "    0: Serial signal error\n";
    if (bits & (1 << 1))
        out << "    1: Serial overrun error\n";
    if (bits & (1 << 2))
        out << "    2: Serial buffer full\n";
    if (bits & (1 << 3))
        out << "    3: Serial CRC error\n";
    if (bits & (1 << 4))
        out << "    4: Serial protocol error\n";
    if (bits & (1 << 5))
        out << "    5: Serial timeout\n";
    if (bits & (1 << 6))
        out << "    6: Script stack error\n";
    if (bits & (1 << 7))
        out << "    7: Script call stack error\n";
    if (bits & (1 << 8))
        out << "    8: Script program counter error\n";
}

void MaestroControl::shell(istream &in, ostream &out, bool interactive) {
    vector<string> raw_command;
    string word;
    // Show the interactive prompt
    if (interactive) {
        uint16_t startup_errors = getErrors();
        if (startup_errors) {
            out << "The following errors occurred before or during startup:\n";
            printErrors(startup_errors, out);
        }
        out << "> ";
        out.flush();
    }
    // Use standard cin splitting
    while(in >> word) {
        if (word == ";") {
            // Execute the entered command on semicolon
            if (raw_command.size() == 0)
                continue;
            // Preprocess command
            vector<string>::iterator it = raw_command.begin();
            while (it != raw_command.end()) {
                // Count backslashes at end of token
                unsigned end_backslash_count = 0;
                for (size_t i = it->size() - 1; i >= 0; --i) {
                    if ((*it)[i] == '\\') {
                        ++end_backslash_count;
                    } else {
                        break;
                    }
                }
                if (end_backslash_count % 2 == 1) {
                    // Ends with unescaped backslash: merge
                    (*it)[it->size() - 1] = ' ';
                    vector<string>::iterator next_arg = it + 1;
                    it->append(*next_arg);
                    raw_command.erase(next_arg);
                } else {
                    // Otherwise, token complete: unescape backslashes
                    size_t pos = 0;
                    while ((pos = it->find("\\\\", pos)) != string::npos) {
                        it->replace(pos, 2, "\\");
                        ++pos;
                    }
                    // Begin work on next token
                    ++it;
                }
            }
            // Extract command
            string command = raw_command[0];
            toLowerCase(command);
            if (command == "exit")
                break;
            // Remove command from argument list
            raw_command.erase(raw_command.begin());
            // Execute the command
            executeShellCommand(command, raw_command, out);
            // Clear the command and show the prompt
            raw_command.clear();
            if (interactive) {
                uint16_t command_errors = getErrors();
                if (command_errors) {
                    printErrors(getErrors(), out);
                }
                out << "> ";
                out.flush();
            }
        } else {
            // If not semicolon, just add to command
            raw_command.push_back(word);
        }
    }
}

// executeShellCommand is defined at the end of the file due to its length.

void MaestroControl::flush() {
    //serial.flush();
}

// PRIVATE //

void MaestroControl::printSerialFlags() {
    //cerr << "Serial state: good: " << serial.good() << ", eof: " << serial.eof()
    //        << ", fail: " << serial.fail() << ", bad: " << serial.bad() << "\n";
    if (serial.getError()) {
        serial.printError();
    }
}

inline void MaestroControl::rawWrite(const uint8_t bytes_array[]) {
    vector<uint8_t> bytes(bytes_array, bytes_array
            + sizeof(bytes_array) / sizeof(uint8_t));
    rawWrite(bytes);
}

void MaestroControl::rawWrite(const vector<uint8_t> &bytes) {
    serial.write(reinterpret_cast<const char *>(&bytes[0]), bytes.size());
    printSerialFlags();
}

vector<uint8_t> *MaestroControl::rawRead(size_t count) {
    char *buffer = new char[count];
    serial.read(buffer, count);
    printSerialFlags();
    vector<uint8_t> *vec = new vector<uint8_t>(
            reinterpret_cast<uint8_t *>(buffer),
            reinterpret_cast<uint8_t *>(buffer + count));
    delete[] buffer;
    return vec;
}

vector<uint8_t> *MaestroControl::formatQuarterUSec(float usec) {
    if (usec == 0) {
        usec = 0;
    } else if (usec < 1000) {
        usec = 1000;
    } else if (usec > 2000) {
        usec = 2000;
    }
    uint16_t quarter_usec = static_cast<uint16_t>(4 * usec);
    vector<uint8_t> *vec = new vector<uint8_t>(2,
            static_cast<uint8_t>(quarter_usec & 0x7F));
    (*vec)[1] = static_cast<uint8_t>((quarter_usec >> 7) & 0x7F);
    return vec;
}

uint16_t MaestroControl::deformatUint16(vector<uint8_t> &little_endian) {
    return static_cast<uint16_t>(little_endian[0]) |
            (static_cast<uint16_t>(little_endian[1]) << 8);
}

void MaestroControl::setMultiTarget(vector<struct Target> targets,
        size_t start_index, size_t index_bound) {
    if (targets.size() == 0)
        return;
    vector<float> targets_usec;
    for (size_t j = start_index; j < index_bound; ++j) {
        targets_usec.push_back(targets[j].target_usec);
    }
    setMultiTarget(targets[start_index].channel, targets_usec);
}

void MaestroControl::setMultiTarget(uint8_t start_channel,
        vector<float> targets_usec) {
    size_t channel_bound = start_channel + targets_usec.size();
    if (channel_bound >= 256) {
        channel_bound = 256;
    }
    size_t length = channel_bound - start_channel;
    vector<uint8_t> bytes(3, 0x9F);
    bytes[1] = targets_usec.size();
    bytes[2] = start_channel;
    for (size_t i = 0; i < length; ++i) {
        vector<uint8_t> &target = *formatQuarterUSec(targets_usec[i]);
        bytes.push_back(target[0]);
        bytes.push_back(target[1]);
        delete &target;
    }
    rawWrite(bytes);
}

int main(int argc, char **argv) {
    MaestroControl ctl;
    if (argc == 2 && argv[1] != "-h") {
        ctl.open(argv[1]);
    } else if (argc != 1) {
        cout << "usage: " << argv[0] << " [device file]\n"
            << "    Run a shell to send commands to the Maestro.\n"
            << "    If [device file] is specified, the shell will\n"
            << "    connect to the Maestro on startup.\n";
        return 1;
    }
    //cerr << ctl.getErrors() << "\n";
    //ctl.setTarget(0, 0);
    //ctl.zeroTargets(5);
    ctl.shell(cin, cout, true);
    return 0;
}

void MaestroControl::executeShellCommand(string command,
        vector<string> arguments, ostream &out) {
    if (command == "help") {
        out << "Shell commands:\n"
            << "    exit        Exit the shell.\n"
            << "    help        Display this help.\n"
            << "    showargs    Display the passed arguments.\n"
            << "Maestro commands:\n"
            << "    acceleration    errors  home        moving\n"
            << "    multitarget     open    position    pwm\n"
            << "    rangetargets    speed   target      zerotargets\n"
            << "Utility commands:\n"
            << "    sleep   usleep\n"
            << "For the mastro and utility commands, use '<command> -h ;'\n"
            << "for more information.\n";
    } else if (command == "showargs") {
        for (size_t i = 0; i < arguments.size(); ++i) {
            out << "[" << i << "]: '" << arguments[i] << "'\n";
        }
    } else if (command == "open") {
        if (arguments.size() == 1 && arguments[0] != "-h") {
            cout << arguments[0].c_str() << "\n";
            open(arguments[0].c_str());
        } else {
            out << "usage: open <device file>\n"
                << "    Connect to the Maestro\n";
        }
    } else if (command == "target") {
        if (arguments.size() == 2) {
            setTarget(static_cast<uint8_t>(atoi(arguments[0].c_str())),
                    static_cast<float>(atof(arguments[1].c_str())));
        } else {
            out << "usage: target <channel> <target>\n"
                << "    Set the target for a servo.\n"
                << "    <target> is in microseconds.\n";
        }
    } else if (command == "multitarget") {
        if (arguments.size() % 2 == 0) {
            vector<struct Target> targets;
            for (size_t i = 0; i < arguments.size(); i += 2) {
                Target t;
                t.channel = static_cast<uint8_t>(atoi(arguments[i].c_str()));
                t.target_usec = static_cast<float>(atof(arguments[i+1].c_str()));
                targets.push_back(t);
            }
            setMultiTarget(targets);
        } else {
            out << "usage: multitarget [<channel> <target>]...\n"
                << "    Set the targets for multiple servos.\n"
                << "    <target> is in microseconds.\n";
        }
    } else if (command == "rangetargets") {
        if (arguments.size() == 3) {
            setRangeTargets(static_cast<uint8_t>(atoi(arguments[0].c_str())),
                    static_cast<uint8_t>(atoi(arguments[1].c_str())),
                    static_cast<float>(atof(arguments[2].c_str())));
        } else {
            out << "usage: rangetargets <start> <bound> <target>\n"
                << "    Set the target for a range of servos.\n"
                << "    <start> is inclusive; <bound> is exclusive.\n"
                << "    <target> is in microseconds.\n";
        }
    } else if (command == "zerotargets") {
        if (arguments.size() == 1 && arguments[0] != "-h") {
            zeroTargets(static_cast<uint8_t>(atoi(arguments[0].c_str())));
        } else {
            out << "usage: zerotargets <count>\n"
                << "    Alias for rangetargets 0 <count> 0.\n";
        }
    } else if (command == "speed") {
        if (arguments.size() == 2) {
            setSpeed(static_cast<uint8_t>(atoi(arguments[0].c_str())),
                    static_cast<float>(atof(arguments[1].c_str())));
        } else {
            out << "usage: speed <channel> <speed>\n"
                << "    Set the speed for a servo.\n"
                << "    <speed> is in microseconds per 10 ms.\n";
        }
    } else if (command == "acceleration") {
        if (arguments.size() == 2) {
            setAcceleration(static_cast<uint8_t>(atoi(arguments[0].c_str())),
                    static_cast<float>(atof(arguments[1].c_str())));
        } else {
            out << "usage: acceleration <channel> <accel>\n"
                << "    Set the accleration for a servo.\n"
                << "    <accel> is in microseconds per 800 ms^2.\n";
        }
    } else if (command == "pwm") {
        if (arguments.size() == 2) {
            setPWM(static_cast<float>(atof(arguments[0].c_str())),
                    static_cast<float>(atof(arguments[1].c_str())));
        } else {
            out << "usage: pwm <on time> <period>\n"
                << "    Set the PWM output (Mini 12, 18, and 24 only).\n"
                << "    Arguments are in microseconds.\n";
        }
    } else if (command == "position") {
        if (arguments.size() == 1 && arguments[0] != "-h") {
            out << getPosition(static_cast<uint8_t>(atoi(arguments[0].c_str())))
                << "us\n";
        } else {
            out << "usage: position <channel>\n"
                << "    Get the current position of a servo in microseconds.\n"
                << "    For digital outputs, >=1500 is high, <1500 is low.\n"
                << "    For analog inputs, value ranges from 0 to 255.75.\n"
                << "    For digital inputs, value is either 0 or 255.75.\n";
        }
    } else if (command == "moving") {
        if (arguments.size() == 0) {
            if (isMoving()) {
                out << "Servos are moving.\n";
            } else {
                out << "Servos are not moving.\n";
            }
        } else {
            out << "usage: moving\n"
                << "    Get whether servos are moving.\n";
        }
    } else if (command == "home") {
        if (arguments.size() == 0) {
            goHome();
        } else {
            out << "usage: home\n"
                << "    Move all servos to home positions.\n";
        }
    } else if (command == "errors") {
        if (arguments.size() == 0) {
            printErrors(getErrors(), out);
        } else {
            out << "usage: errors\n"
                << "    Print and clear the error register.\n"
                << "    Does nothing in interactive mode, as errors are\n"
                << "    always reported.\n";
        }
    } else if (command == "sleep") {
        if (arguments.size() == 1 && arguments[0] != "-h") {
            sleep(static_cast<unsigned>(atoi(arguments[0].c_str())));
        } else {
            out << "usage: sleep <sec>\n"
                << "    Sleep for <sec> seconds.\n";
        }
    } else if (command == "usleep") {
        if (arguments.size() == 1 && arguments[0] != "-h") {
            usleep(static_cast<useconds_t>(atoi(arguments[0].c_str())));
        } else {
            out << "usage: usleep <usec>\n"
                << "    Sleep for <usec> microseconds.\n";
        }
    } else {
        out << "'" << command << "' is not a valid command.\n"
            << "Type 'help ;' for help.\n";
    }
    flush();
}

