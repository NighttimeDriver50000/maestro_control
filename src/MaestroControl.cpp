#include "MaestroControl.hpp"
#include <algorithm>
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
    serial.open(filename, ios_base::in | ios_base::out | ios_base::binary);
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

void MaestroControl::shell(istream &in, ostream &out, bool interactive) {
    vector<string> raw_command;
    string word;
    // Show the interactive prompt
    if (interactive) {
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
                out << "> ";
                out.flush();
            }
        } else {
            // If not semicolon, just add to command
            raw_command.push_back(word);
        }
    }
}

void MaestroControl::executeShellCommand(string command,
        vector<string> arguments, ostream &out) {
    if (command == "help") {
        out << "Shell commands:\n"
            << "    exit        Exit the shell.\n"
            << "    help        Display this help.\n"
            << "    showargs    Display the passed arguments.\n"
            << "Maestro commands:\n"
            << "    open    target\n"
            << "For these, use '<command> -h ;' for more information.\n";
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
    } else {
        out << "'" << command << "' is not a valid command.\n"
            << "Type 'help ;' for help.\n";
    }
    flush();
}

void MaestroControl::flush() {
    serial.flush();
}

// PRIVATE //

inline void MaestroControl::rawWrite(const uint8_t bytes_array[]) {
    vector<uint8_t> bytes(bytes_array, bytes_array
            + sizeof(bytes_array) / sizeof(uint8_t));
    rawWrite(bytes);
}

void MaestroControl::rawWrite(const vector<uint8_t> &bytes) {
    serial.write(reinterpret_cast<const char *>(&bytes[0]), bytes.size());
}

vector<uint8_t> *MaestroControl::rawRead(size_t count) {
    char *buffer = new char[count];
    serial.read(buffer, count);
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
    //ctl.zeroTargets(5);
    ctl.shell(cin, cout, true);
    return 0;
}
