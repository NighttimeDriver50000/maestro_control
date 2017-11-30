#include "MaestroControl.hpp"

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
