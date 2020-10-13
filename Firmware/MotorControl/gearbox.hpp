#ifndef __GEARBOX_HPP
#define __GEARBOX_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class Gearbox : public ODriveIntf::GearboxIntf {
public:
    struct Config_t {
        bool enabled = false;
        int32_t gears_in = 1;
        int32_t gears_out = 1;
        MountPoint mount_point = MOUNT_POINT_BETWEEN;

        Gearbox* parent = nullptr;
    };

    explicit Gearbox(Config_t& config);
    Axis* axis_ = nullptr; // set by Axis constructor
    Config_t& config_;

    bool encoder_is_scaled() {
        return config_.enabled && MOUNT_POINT_BETWEEN == config_.mount_point;
    }

    float pos_fwd_ratio();
    float pos_bwd_ratio();
};
#endif /* __GEARBOX_HPP */
