#ifndef ANALOG_EMULATOR_H
#define ANALOG_EMULATOR_H

#include <vector>
#include <cstddef>
#include <algorithm>

// Simple header-only AnalogEmulator
// Each channel can be driven by a digital input (pressed/unpressed)
// and will produce a smooth analog value in range [0,1] using a per-channel ramp time.

class AnalogEmulator {
public:
    struct Channel {
        double value = 0.0;   // current analog value 0..1
        bool digital = false;  // current digital input state
        double ramp_sec = 0.15; // time to ramp from 0->1 or 1->0
    };

    AnalogEmulator() = default;
    explicit AnalogEmulator(std::size_t n) { channels_.resize(n); }

    void resize(std::size_t n) { channels_.resize(n); }
    std::size_t size() const { return channels_.size(); }

    void setDigital(std::size_t idx, bool pressed) {
        if (idx >= channels_.size()) return;
        channels_[idx].digital = pressed;
    }

    void setRamp(std::size_t idx, double sec) {
        if (idx >= channels_.size()) return;
        channels_[idx].ramp_sec = (sec <= 0.0) ? 0.000001 : sec;
    }

    void setGlobalRamp(double sec) {
        for (auto &c : channels_) c.ramp_sec = (sec <= 0.0) ? 0.000001 : sec;
    }

    void update(double dt) {
        if (dt <= 0.0) return;
        for (auto &c : channels_) {
            double target = c.digital ? 1.0 : 0.0;
            double rate = (c.ramp_sec > 0.0) ? (dt / c.ramp_sec) : 1.0;
            if (c.value < target) c.value = std::min(1.0, c.value + rate);
            else if (c.value > target) c.value = std::max(0.0, c.value - rate);
        }
    }

    double getValue(std::size_t idx) const {
        if (idx >= channels_.size()) return 0.0;
        return channels_[idx].value;
    }

private:
    std::vector<Channel> channels_;
};

#endif // ANALOG_EMULATOR_H
