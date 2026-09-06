#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <array>

#include <fcntl.h>
#include <getopt.h>
#include <linux/uinput.h>
#include <poll.h>
#include <time.h>
#include <unistd.h>

#include "../AnalogEmulator/AnalogEmulatorDevice.h"

#define AXIS_MIN        (-32767)
#define AXIS_MAX        ( 32767)
#define TRIGGER_MIN     0
#define TRIGGER_MAX     32767

// Tunables (kept as globals for parity)
static double COMMON_RAMP_TIME_SEC = 0.15; // lower = faster
static double MAX_STEER_DEGREES = 40.0;
static double STEERING_DEADZONE_DEGREES = 1.5;
static double CALIBRATION_OFFSET = 0.0;
static bool WIIMOTE_INVERT = false;
static double STEERING_CENTER_DEADZONE_DEGREES = 0.35;
#define CALIBRATION_SAMPLES 40
static double STEERING_SMOOTHING_ALPHA = 0.25;
static const int UPDATE_HERTZ = 125;

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int sig) { (void)sig; g_stop = 1; }

// small utilities
static double now_seconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Trigger structure
struct Trigger {
    const char *name;
    int abs_code;
    bool pressed{false};
    double value{0.0}; // 0..1
    double ramp_sec{0.15};
    Trigger() = default;
    Trigger(const char *n, int code, bool p, double v, double r)
        : name(n), abs_code(code), pressed(p), value(v), ramp_sec(r) {}
};

enum { TRG_GAS = 0, TRG_BRAKE, TRG_HANDBRAKE, TRG_CLUTCH, TRG_COUNT };

static std::array<Trigger, TRG_COUNT> g_triggers = {
    Trigger{"Throttle (A)", ABS_GAS, false, 0.0, 0.15},
    Trigger{"Brake (B)",        ABS_BRAKE, false, 0.0, 0.15},
    Trigger{"Hand Brake (2)", ABS_Z, false, 0.0, 0.15},
    Trigger{"Clutch (Down)",   ABS_RZ, false, 0.0, 0.15},
};

// Analog emulator instance: will produce smooth 0..1 values from digital inputs
static AnalogEmulator g_analog_emulator(TRG_COUNT);

static void apply_global_ramp(double sec) {
    for (auto &t : g_triggers) t.ramp_sec = sec;
}

// update_trigger removed — using AnalogEmulator for smoothing

extern "C" {
#include <xwiimote.h>
}


// Find wiimote syspath (unchanged logic)
static char *find_wiimote_syspath() {
    struct xwii_monitor *mon = xwii_monitor_new(false, false);
    if (!mon) { std::cerr << "Could not open xwiimote monitor.\n"; return nullptr; }
    char *path = xwii_monitor_poll(mon);
    xwii_monitor_unref(mon);
    return path; // free() by caller
}

// Accel and plane
struct accel_t { int32_t x, y, z; };
enum class plane_t { XZ, XY, YZ };
static plane_t g_plane = plane_t::XY;

static double angle_from_accel(const accel_t &a, plane_t plane) {
    switch (plane) {
        case plane_t::XY: return std::atan2((double)a.x, (double)a.y);
        case plane_t::YZ: return std::atan2((double)a.y, (double)a.z);
        case plane_t::XZ:
        default: return std::atan2((double)a.x, (double)a.z);
    }
}

static void print_usage(const char *prog) {
    std::cout << "Usage: " << prog << " [syspath] [options]\n"
              << "  syspath          wiimote device path\n"
              << "  --debug          prints raw accel value\n"
              << "  --plane xz|xy|yz default rotation plane (default: " << (g_plane == plane_t::XY ? "xy" : g_plane == plane_t::YZ ? "yz" : "xz") << ")\n"
              << "  --max-steer N    max steering angle in degrees (default: " << MAX_STEER_DEGREES << ")\n"
              << "  --ramp N         ramp time in seconds (default: " << COMMON_RAMP_TIME_SEC << ")\n"
              << "  --trim N         steering offset correction (default: " << CALIBRATION_OFFSET << ")\n"
              << "  --smooth N       0..1, smooth-out filter alpha value (default: " << STEERING_SMOOTHING_ALPHA << ", 1=none)\n"
              << "  --invert         invert sx/dx\n"
              << "  --center N       deadzone near center (default: " << STEERING_CENTER_DEADZONE_DEGREES << ")\n"
              << "  -h, --help       shows help message\n";
}

int main(int argc, char **argv) {
    bool debug = false;
    char *syspath_arg = nullptr;

    static struct option long_opts[] = {
        {"debug", no_argument, 0, 'd'},
        {"plane", required_argument, 0, 'p'},
        {"max-steer", required_argument, 0, 'm'},
        {"ramp", required_argument, 0, 'r'},
        {"trim", required_argument, 0, 't'},
        {"smooth", required_argument, 0, 's'},
        {"invert", no_argument, 0, 'i'},
        {"center", required_argument, 0, 'c'},
        {"help", no_argument, 0, 'h'},
        {0,0,0,0}
    };

    int opt, idx;
    while ((opt = getopt_long(argc, argv, "hdp:m:r:t:s:ic:", long_opts, &idx)) != -1) {
        switch (opt) {
            case 'd': debug = true; break;
            case 'p':
                if (std::strcmp(optarg, "xy") == 0) g_plane = plane_t::XY;
                else if (std::strcmp(optarg, "yz") == 0) g_plane = plane_t::YZ;
                else g_plane = plane_t::XZ;
                break;
            case 'm': MAX_STEER_DEGREES = std::atof(optarg); break;
            case 'r': COMMON_RAMP_TIME_SEC = std::atof(optarg); break;
            case 't': CALIBRATION_OFFSET = std::atof(optarg); break;
            case 's': STEERING_SMOOTHING_ALPHA = clampd(std::atof(optarg), 0.01, 1.0); break;
            case 'i': WIIMOTE_INVERT = !WIIMOTE_INVERT; break;
            case 'c': STEERING_CENTER_DEADZONE_DEGREES = clampd(std::atof(optarg), 0.0, 5.0); break;
            case 'h':
            default:
                print_usage(argv[0]);
                return (opt == 'h') ? 0 : 1;
        }
    }
    if (optind < argc) syspath_arg = argv[optind];

    apply_global_ramp(COMMON_RAMP_TIME_SEC);

    // configure analog emulator ramps to match trigger defaults
    g_analog_emulator.resize(TRG_COUNT);
    for (int i = 0; i < TRG_COUNT; ++i) g_analog_emulator.setRamp(i, g_triggers[i].ramp_sec);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    char *syspath = syspath_arg ? strdup(syspath_arg) : find_wiimote_syspath();
    if (!syspath) {
        std::cerr << "No Wiimote found. Connect one with Bluetooth or specify a device path.\n";
        return 1;
    }
    std::cout << "Wiimote found: " << syspath << "\n";

    struct xwii_iface *iface = nullptr;
    if (xwii_iface_new(&iface, syspath) < 0) {
        std::cerr << "Error creating xwii interface\n";
        free(syspath);
        return 1;
    }
    free(syspath);

    unsigned int requested_ifaces = XWII_IFACE_CORE | XWII_IFACE_ACCEL | XWII_IFACE_WRITABLE;
    int ret = xwii_iface_open(iface, requested_ifaces);
    if (ret < 0) {
        std::cerr << "Error opening wiimote interfaces: " << std::strerror(-ret) << "\n";
        xwii_iface_unref(iface);
        return 1;
    }
    unsigned int opened_ifaces = (unsigned int)ret;
    if (!(opened_ifaces & XWII_IFACE_ACCEL)) std::cerr << "Attention: accelerometer interface not available.\n";

    AnalogEmulatorDevice aedev;
    std::vector<int> abs_codes;
    abs_codes.push_back(ABS_X);
    for (int i = 0; i < TRG_COUNT; ++i) abs_codes.push_back(g_triggers[i].abs_code);
    std::vector<int> btn_codes = { BTN_TOP, BTN_TOP2 };
    if (!aedev.open_device("Wii4Race Device", abs_codes, btn_codes)) {
        xwii_iface_close(iface, opened_ifaces);
        xwii_iface_unref(iface);
        return 1;
    }
    // set ranges
    aedev.setAxisRange(ABS_X, AXIS_MIN, AXIS_MAX);
    for (int i = 0; i < TRG_COUNT; ++i) aedev.setAxisRange(g_triggers[i].abs_code, TRIGGER_MIN, TRIGGER_MAX);
    std::cout << "Virtual udev joystick device opened: /dev/input/js* (name \"Wii4Race Device\")\n";

    int wfd = xwii_iface_get_fd(iface);

    bool btn_plus = false, btn_minus = false;
    accel_t last_accel{0,0,1};
    bool have_accel = false;
    double ref_angle = 0.0;
    double filtered_angle = 0.0;
    bool filter_init = false;
    bool calibrating = true;
    int calib_count = 0;
    double calib_sum_x = 0.0, calib_sum_y = 0.0, calib_sum_z = 0.0;

    std::cout << "Initial steering calibration: keep the wiimote still in the preferred neutral position...\n";

    double last_t = now_seconds();
    const double loop_dt = 1.0 / (double)UPDATE_HERTZ;

    while (!g_stop) {
        struct pollfd pfd;
        pfd.fd = wfd; pfd.events = POLLIN; pfd.revents = 0;
        int pret = poll(&pfd, 1, (int)(loop_dt * 1000));
        if (pret < 0) { if (errno == EINTR) continue; std::cerr << "poll() failed: " << std::strerror(errno) << "\n"; break; }

        if (pret > 0 && (pfd.revents & POLLIN)) {
            struct xwii_event ev;
            while (xwii_iface_dispatch(iface, &ev, sizeof(ev)) == 0) {
                if (ev.type == XWII_EVENT_KEY) {
                    unsigned int code = ev.v.key.code;
                    bool pressed = (ev.v.key.state == 1);
                    switch (code) {
                        case XWII_KEY_A: g_analog_emulator.setDigital(TRG_GAS, pressed); break;
                        case XWII_KEY_B: g_analog_emulator.setDigital(TRG_BRAKE, pressed); break;
                        case XWII_KEY_TWO: g_analog_emulator.setDigital(TRG_HANDBRAKE, pressed); break;
                        case XWII_KEY_DOWN: g_analog_emulator.setDigital(TRG_CLUTCH, pressed); break;
                        case XWII_KEY_PLUS: btn_plus = pressed; break;
                        case XWII_KEY_MINUS: btn_minus = pressed; break;
                        case XWII_KEY_ONE:
                            if (pressed) {
                                calibrating = true; calib_count = 0; calib_sum_x = calib_sum_y = calib_sum_z = 0.0; filter_init = false;
                                std::cout << "Calibration: keep the wiimote still in the preferred neutral position...\n";
                            }
                            break;
                        default: break;
                    }
                } else if (ev.type == XWII_EVENT_ACCEL) {
                    last_accel.x = ev.v.abs[0].x;
                    last_accel.y = ev.v.abs[0].y;
                    last_accel.z = ev.v.abs[0].z;
                    have_accel = true;
                    if (debug) std::printf("accel x=%d y=%d z=%d\n", last_accel.x, last_accel.y, last_accel.z);
                    if (calibrating) {
                        calib_sum_x += last_accel.x; calib_sum_y += last_accel.y; calib_sum_z += last_accel.z; calib_count++;
                        if (calib_count >= CALIBRATION_SAMPLES) {
                            accel_t avg{ (int32_t)(calib_sum_x / calib_count), (int32_t)(calib_sum_y / calib_count), (int32_t)(calib_sum_z / calib_count) };
                            ref_angle = angle_from_accel(avg, g_plane);
                            calibrating = false; filtered_angle = ref_angle; filter_init = true;
                            std::printf("Steering calibrated (zero set to average of %d samples).\n", calib_count);
                        }
                    }
                } else if (ev.type == XWII_EVENT_GONE) {
                    std::cerr << "Wiimote disconnected.\n"; g_stop = 1; break;
                }
            }
        }

        double t = now_seconds();
        double dt = t - last_t; last_t = t; if (dt <= 0 || dt > 0.5) dt = loop_dt;

        int steer_out = 0;
        if (have_accel && !calibrating) {
            double raw_angle = angle_from_accel(last_accel, g_plane);
            if (!filter_init) { filtered_angle = raw_angle; filter_init = true; }
            else filtered_angle += STEERING_SMOOTHING_ALPHA * (raw_angle - filtered_angle);

            double angle_rad = filtered_angle - ref_angle;
            double angle_deg = angle_rad * 180.0 / M_PI;
            while (angle_deg > 180.0) angle_deg -= 360.0;
            while (angle_deg < -180.0) angle_deg += 360.0;
            angle_deg += CALIBRATION_OFFSET;

            double deadzoned = angle_deg;
            if (std::fabs(deadzoned) < STEERING_DEADZONE_DEGREES) deadzoned = 0.0;
            else deadzoned -= (deadzoned > 0 ? STEERING_DEADZONE_DEGREES : -STEERING_DEADZONE_DEGREES);
            if (std::fabs(deadzoned) <= STEERING_CENTER_DEADZONE_DEGREES) deadzoned = 0.0;
            double norm = clampd(deadzoned / MAX_STEER_DEGREES, -1.0, 1.0);
            if (WIIMOTE_INVERT) norm = -norm;
            steer_out = (int)(norm * AXIS_MAX);
        }

        // Update analog emulator and emit via unified device
        g_analog_emulator.update(dt);
        // emit raw axis values like the original implementation
        aedev.setAxisRaw(ABS_X, steer_out);
        for (int i = 0; i < TRG_COUNT; ++i) {
            int out = (int)(g_analog_emulator.getValue(i) * TRIGGER_MAX);
            aedev.setAxisRaw(g_triggers[i].abs_code, out);
        }
        aedev.setButton(BTN_TOP, btn_plus);
        aedev.setButton(BTN_TOP2, btn_minus);
        aedev.syn_report();
    }

    std::cout << "Closing in progress\n";
    xwii_iface_close(iface, opened_ifaces);
    xwii_iface_unref(iface);
    return 0;
}