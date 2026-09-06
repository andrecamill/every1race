#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "../AnalogEmulator/AnalogEmulator.h"
#include "../AnalogEmulator/AnalogEmulatorDevice.h"

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

static double now_seconds() {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec/1e9;
}

static bool configure_serial(int fd, int baud) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return false;
    cfmakeraw(&tty);
    speed_t sp = B9600;
    cfsetospeed(&tty, sp);
    cfsetispeed(&tty, sp);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE; tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB; tty.c_cflag &= ~CSTOPB;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 10;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;
    return true;
}

int main(int argc, char **argv) {
    const char *dev = nullptr;
    if (argc > 1) dev = argv[1];
    const char *cdevs[] = { "/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1" };

    int fd = -1;
    if (dev) fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    else {
        for (auto d : cdevs) {
            fd = open(d, O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd >= 0) { dev = d; break; }
        }
    }
    if (fd < 0) { std::cerr << "Unable to open serial device\n"; return 1; }
    if (!configure_serial(fd, 9600)) { std::cerr << "Failed to configure serial port\n"; close(fd); return 1; }
    std::cout << "Opened serial device: " << dev << "\n";

    // device: expose throttle, brake, clutch axes
    enum { TRG_THRO = 0, TRG_BRAKE, TRG_CLUTCH, TRG_COUNT };
    std::vector<int> abs_codes = { ABS_GAS, ABS_BRAKE, ABS_RZ };
    std::vector<int> btns; // none
    AnalogEmulatorDevice aedev;
    if (!aedev.open_device("Pedals Device", abs_codes, btns)) { close(fd); return 1; }
    for (int i = 0; i < (int)abs_codes.size(); ++i) aedev.setAxisRange(abs_codes[i], 0, 32767);

    AnalogEmulator emu(TRG_COUNT);
    emu.setGlobalRamp(0.15);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    double last_t = now_seconds();
    std::string line;
    // use FILE* for line-based reads
    FILE *f = fdopen(fd, "r+");
    if (!f) { std::cerr << "fdopen failed\n"; close(fd); return 1; }

    char buf[256];
    while (!g_stop) {
        double t = now_seconds(); double dt = t - last_t; last_t = t;
        // read available lines
        fd_set rfds; FD_ZERO(&rfds); FD_SET(fd, &rfds);
        struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 50000; // 50ms
        int sel = select(fd+1, &rfds, NULL, NULL, &tv);
        if (sel > 0 && FD_ISSET(fd, &rfds)) {
            if (fgets(buf, sizeof(buf), f) != NULL) {
                line = std::string(buf);
                // strip \r\n
                while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) line.pop_back();
                if (line.find("PEDAL_THROTTLE:") == 0) {
                    int v = atoi(line.c_str() + strlen("PEDAL_THROTTLE:"));
                    emu.setDigital(TRG_THRO, v != 0);
                } else if (line.find("PEDAL_BRAKE:") == 0) {
                    int v = atoi(line.c_str() + strlen("PEDAL_BRAKE:"));
                    emu.setDigital(TRG_BRAKE, v != 0);
                } else if (line.find("PEDAL_CLUTCH:") == 0) {
                    int v = atoi(line.c_str() + strlen("PEDAL_CLUTCH:"));
                    emu.setDigital(TRG_CLUTCH, v != 0);
                }
            }
        }

        emu.update(dt);
        for (int i = 0; i < TRG_COUNT; ++i) {
            double v = emu.getValue(i);
            aedev.setAxisValue(abs_codes[i], v);
        }
        aedev.syn_report();
    }

    std::cout << "Shutting down\n";
    fclose(f);
    aedev.close_device();
    return 0;
}
