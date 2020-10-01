
#include "odrive_main.h"

Gearbox::Gearbox(Config_t& config) : config_(config){};

float Gearbox::torque_fwd(float torque_in) {
    return torque_in * (float)config_.gears_out / (float)config_.gears_in * config_.efficiency - config_.friction_torque;
}

float Gearbox::torque_bwd(float torque_out) {
    return (torque_out + config_.friction_torque) / config_.efficiency * (float)config_.gears_in / (float)config_.gears_out;
}

float Gearbox::pos_fwd_ratio() {
    return (float)config_.gears_in / (float)config_.gears_out;
}

float Gearbox::pos_bwd_ratio() {
    return (float)config_.gears_out / (float)config_.gears_in;
}
