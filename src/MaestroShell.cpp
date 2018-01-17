#include "MaestroControl.hpp"
#include <string.h>

int main(int argc, char **argv) {
    MaestroControl ctl;
    if (argc == 2 && strcmp(argv[1], "-h") != 0) {
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
    cout << "Type `help ;` to see a list of commands.\n";
    ctl.shell(cin, cout, true);
    return 0;
}
