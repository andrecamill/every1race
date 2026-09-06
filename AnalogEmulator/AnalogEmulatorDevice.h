#ifndef ANALOG_EMULATOR_DEVICE_H
#define ANALOG_EMULATOR_DEVICE_H

#include "AnalogEmulator.h"

#include <cstring>
#include <iostream>
#include <map>
#include <vector>

#include <fcntl.h>
#include <linux/uinput.h>
#include <unistd.h>
#include <sys/ioctl.h>

// Minimal helper that creates a uinput device exposing specified ABS axes and buttons,
// and provides a simple API to update axis values (0..1) and button states.

class AnalogEmulatorDevice {
public:
    AnalogEmulatorDevice() = default;
    ~AnalogEmulatorDevice() { close_device(); }

    bool open_device(const std::string &name,
                     const std::vector<int> &abs_codes,
                     const std::vector<int> &btn_codes) {
        fd_ = ::open("/dev/uinput", O_WRONLY | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "Unable to open /dev/uinput: " << std::strerror(errno) << "\n";
            return false;
        }
        ioctl(fd_, UI_SET_EVBIT, EV_ABS);
        for (int c : abs_codes) ioctl(fd_, UI_SET_ABSBIT, c);
        ioctl(fd_, UI_SET_EVBIT, EV_KEY);
        for (int b : btn_codes) ioctl(fd_, UI_SET_KEYBIT, b);

        struct uinput_user_dev uidev;
        std::memset(&uidev, 0, sizeof(uidev));
        std::snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "%s", name.c_str());
        uidev.id.bustype = BUS_USB;
        uidev.id.vendor = 0x0079;
        uidev.id.product = 0x1811;
        uidev.id.version = 1;

        // default ranges
        for (int c : abs_codes) {
            if (c >= 0 && c < ABS_CNT) {
                uidev.absmin[c] = 0;
                uidev.absmax[c] = 32767;
            }
        }

        if (::write(fd_, &uidev, sizeof(uidev)) != (ssize_t)sizeof(uidev)) {
            std::cerr << "Error writing uinput_user_dev: " << std::strerror(errno) << "\n";
            ::close(fd_); fd_ = -1; return false;
        }
        if (ioctl(fd_, UI_DEV_CREATE) < 0) {
            std::cerr << "Error UI_DEV_CREATE: " << std::strerror(errno) << "\n";
            ::close(fd_); fd_ = -1; return false;
        }

        // store mapping
        for (int c : abs_codes) axes_.push_back(c);
        for (int b : btn_codes) buttons_.push_back(b);

        analog_.resize(axes_.size());
        return true;
    }

    void close_device() {
        if (fd_ >= 0) {
            ioctl(fd_, UI_DEV_DESTROY);
            ::close(fd_);
            fd_ = -1;
        }
    }

    void setAxisRange(int abs_code, int minv, int maxv) {
        // cannot change after device create in this simple implementation
        // but we keep a range map for scaling
        ranges_[abs_code] = {minv, maxv};
    }

    // value in 0..1
    void setAxisValue(int abs_code, double v) {
        if (fd_ < 0) return;
        int idx = findAxisIndex(abs_code);
        if (idx < 0) return;
        v = std::max(0.0, std::min(1.0, v));
        int minv = 0, maxv = 32767;
        auto it = ranges_.find(abs_code);
        if (it != ranges_.end()) { minv = it->second.first; maxv = it->second.second; }
        int out = (int)(minv + v * (double)(maxv - minv));
        emit(EV_ABS, abs_code, out);
    }

    void setButton(int btn_code, bool pressed) {
        if (fd_ < 0) return;
        emit(EV_KEY, btn_code, pressed ? 1 : 0);
    }

    void syn_report() { if (fd_ >= 0) emit(EV_SYN, SYN_REPORT, 0); }

private:
    int fd_ = -1;
    std::vector<int> axes_;
    std::vector<int> buttons_;
    AnalogEmulator analog_{0};
    std::map<int, std::pair<int,int>> ranges_;

    int findAxisIndex(int code) {
        for (size_t i = 0; i < axes_.size(); ++i) if (axes_[i] == code) return (int)i;
        return -1;
    }

    void emit(int type, int code, int value) {
        if (fd_ < 0) return;
        struct input_event ie;
        std::memset(&ie, 0, sizeof(ie));
        ie.type = type; ie.code = code; ie.value = value;
        ::write(fd_, &ie, sizeof(ie));
    }
};

#endif // ANALOG_EMULATOR_DEVICE_H
