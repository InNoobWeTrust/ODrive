
#include "odrive_main.h"

Gearbox::Gearbox(Config_t& config) : config_(config){};

float Gearbox::torque_fwd_ratio() {
    return (float)config_.gears_out / (float)config_.gears_in;
}

float Gearbox::torque_bwd_ratio() {
    return (float)config_.gears_in / (float)config_.gears_out;
}

float Gearbox::pos_fwd_ratio() {
    return (float)config_.gears_in / (float)config_.gears_out;
}

float Gearbox::pos_bwd_ratio() {
    return (float)config_.gears_out / (float)config_.gears_in;
}
