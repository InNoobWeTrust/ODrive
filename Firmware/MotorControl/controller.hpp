#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class Controller : public ODriveIntf::ControllerIntf {
public:
    typedef struct {
        uint32_t index = 0;
        float cogging_map[3600];
        bool pre_calibrated = false;
        bool calib_anticogging = false;
        float calib_pos_threshold = 1.0f;
        float calib_vel_threshold = 1.0f;
        float cogging_ratio = 1.0f;
        bool anticogging_enabled = true;
    } Anticogging_t;

    struct Config_t {
        ControlMode control_mode = CONTROL_MODE_POSITION_CONTROL;  //see: ControlMode_t
        InputMode input_mode = INPUT_MODE_PASSTHROUGH;             //see: InputMode_t
        float pos_gain = 20.0f;                  // [(turn/s) / turn]
        float vel_gain = 1.0f / 6.0f;            // [Nm/(turn/s)]
        // float vel_gain = 0.2f / 200.0f,       // [Nm/(rad/s)] <sensorless example>
        float vel_integrator_gain = 2.0f / 6.0f; // [Nm/(turn/s * s)]
        float vel_limit = 2.0f;                  // [turn/s] Infinity to disable.
        float vel_limit_tolerance = 1.2f;        // ratio to vel_lim. Infinity to disable.
        float vel_ramp_rate = 1.0f;              // [(turn/s) / s]
        float torque_ramp_rate = 0.01f;          // Nm / sec
        bool circular_setpoints = false;
        float circular_setpoint_range = 1.0f; // Circular range when circular_setpoints is true. [turn]
        float inertia = 0.0f;                 // [Nm/(turn/s^2)]
        float input_filter_bandwidth = 2.0f;  // [1/s]
        float homing_speed = 0.25f;           // [turn/s]
        Anticogging_t anticogging;
        float gain_scheduling_width = 10.0f;
        bool enable_gain_scheduling = false;
        bool enable_vel_limit = true;
        bool enable_overspeed_error = true;
        bool enable_current_mode_vel_limit = true;  // enable velocity limit in current control mode (requires a valid velocity estimator)
        //---------------------------- Servo mode ----------------------------//
        /*
        Compensate for backlash and friction on motor velocity:
        - Load position by encoder
        - Encoder vel (load side) and sensorless vel (motor side) for disturbance cancellation
        - [Need verification]: pll bandwidth of encoder and sensorless should be the same

        Disturbance compensated as follow:
            disturbance += ((gears_out / gears_in) * vel_load - vel_des) * gain
            vel_des = vel_cmd + disturbance
        Where:
            disturbance: the low-pass-filtered disturbance between load vel and vel_des
            gain: calculated from low pass filter bandwidth specified by `disturbance_filter_bandwidth`
            vel_load: load velocity measured by encoder
            vel_des: previously computed motor velocity
            vel_cmd: demanded motor velocity

        In short: Error from commanded and output vel is compensated in next control loop.
        Note: disturbance is filtered by simple low-pass filter
        (specifically, leaky integrator, see: https://en.wikipedia.org/wiki/Leaky_integrator)
        */
        float disturbance_filter_bandwidth = 100.0f;
        //--------------------------------------------------------------------//
        uint8_t axis_to_mirror = -1;
        float mirror_ratio = 1.0f;
        uint8_t load_encoder_axis = -1;  // default depends on Axis number and is set in load_configuration()

        // custom setters
        Controller* parent;
        void set_input_filter_bandwidth(float value) { input_filter_bandwidth = value; parent->update_filter_gains(); }
        void set_disturbance_filter_bandwidth(float value) { disturbance_filter_bandwidth = value; parent->update_disturbance_gains(); }
    };

    explicit Controller(Config_t& config);
    void reset();
    void set_error(Error error);

    constexpr void input_pos_updated() {
        input_pos_updated_ = true;
    }

    // Trajectory-Planned control
    void move_to_pos(float goal_point);
    void move_incremental(float displacement, bool from_goal_point);
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration();
    bool anticogging_calibration(float pos_estimate, float vel_estimate);

    void update_filter_gains();
    void update_disturbance_gains();
    bool update(float* torque_setpoint, bool servo_mode);

    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error error_ = ERROR_NONE;

    bool* pos_estimate_valid_src_ = nullptr;
    bool* vel_estimate_valid_src_ = nullptr;


    float pos_setpoint_ = 0.0f; // [turns]
    float vel_setpoint_ = 0.0f; // [turn/s]
    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_torque_ = 0.0f;    // [Nm]
    float torque_setpoint_ = 0.0f;  // [Nm]

    float disturbance_filter_gain_ = 0.0f;
    float vel_disturbance_ = 0.0f;
    float vel_des_ = 0.0f;

    float input_pos_ = 0.0f;     // [turns]
    float input_vel_ = 0.0f;     // [turn/s]
    float input_torque_ = 0.0f;  // [Nm]
    float input_filter_kp_ = 0.0f;
    float input_filter_ki_ = 0.0f;

    bool input_pos_updated_ = false;
    
    bool trajectory_done_ = true;

    bool anticogging_valid_ = false;

    // custom setters
    void set_input_pos(float value) { input_pos_ = value; input_pos_updated(); }

};

#endif // __CONTROLLER_HPP
