
#include "odrive_main.h"

Gearbox::Gearbox(Config_t& config) : config_(config){};

float Gearbox::torque_mul_ratio() {
    return (float)config_.gears_out / (float)config_.gears_in;
}

float Gearbox::pos_mul_ratio() {
    return (float)config_.gears_in / (float)config_.gears_out;
}
