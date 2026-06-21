#include "diffbot/diffbot_system.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace diffbot {
    namespace {
        constexpr std::array<std::pair<double, int>, 7> kLeftFwdVelPwmLut = {
            {
                {4.334478, 424},
                {5.312312, 455},
                {7.581051, 492},
                {7.897859, 535},
                {8.748700, 591},
                {11.137767, 659},
                {15.019788, 918},
            }
        };

        constexpr std::array<std::pair<double, int>, 8> kLeftRevVelPwmLut = {
            {
                {2.604523, 399},
                {3.052242, 424},
                {4.309446, 455},
                {4.667298, 492},
                {7.676395, 535},
                {8.549105, 591},
                {10.144822, 739},
                {16.951065, 918},
            }
        };

        constexpr std::array<std::pair<double, int>, 7> kRightFwdVelPwmLut = {
            {
                {3.669481, 424},
                {4.455018, 455},
                {6.218632, 492},
                {6.783195, 535},
                {8.179876, 591},
                {10.313643, 831},
                {13.938352, 918},
            }
        };

        constexpr std::array<std::pair<double, int>, 9> kRightRevVelPwmLut = {
            {
                {2.275923, 399},
                {2.827943, 424},
                {3.844874, 455},
                {4.627152, 492},
                {7.556345, 535},
                {8.212594, 591},
                {9.960523, 739},
                {10.160175, 831},
                {16.574547, 918},
            }
        };

        constexpr std::array<std::pair<double, int>, 7> kLeftFwdArcVelPwmLut = {
            {
                {3.970032, 350},
                {4.013959, 358},
                {5.791637, 395},
                {6.202043, 410},
                {7.152794, 441},
                {8.866740, 496},
                {10.531654, 629},
            }
        };

        constexpr std::array<std::pair<double, int>, 7> kLeftRevArcVelPwmLut = {
            {
                {3.970606, 350},
                {4.903722, 382},
                {5.170994, 396},
                {5.796767, 444},
                {8.710090, 486},
                {9.248427, 572},
                {11.979371, 658},
            }
        };

        constexpr std::array<std::pair<double, int>, 7> kRightFwdArcVelPwmLut = {
            {
                {3.757721, 350},
                {3.933852, 387},
                {5.399056, 407},
                {5.817514, 422},
                {6.852131, 467},
                {8.145835, 533},
                {10.246681, 644},
            }
        };

        constexpr std::array<std::pair<double, int>, 8> kRightRevArcVelPwmLut = {
            {
                {3.898512, 350},
                {3.982059, 361},
                {4.735447, 388},
                {5.066951, 401},
                {5.728713, 461},
                {8.534064, 497},
                {8.976415, 588},
                {12.239586, 668},
            }
        };

        constexpr std::array<std::pair<double, int>, 2> kLeftSpinVelPwmLut = {
            {
                {6.413889, 707},
                {6.923044, 744}
            }
        };

        constexpr std::array<std::pair<double, int>, 3> kRightSpinVelPwmLut = {
            {
                {6.112850, 706},
                {7.804377, 744},
                {8.000700, 836},
            }
        };

        template<std::size_t N>
        int interpolate_lut_pwm(const std::array<std::pair<double, int>, N> &lut, const double abs_vel) {
            static_assert(N >= 2, "LUT must have at least 2 points");

            if (abs_vel <= lut.front().first) {
                return lut.front().second;
            }

            if (abs_vel >= lut.back().first) {
                const auto [x0, y0] = lut[N - 2];
                const auto [x1, y1] = lut[N - 1];
                const double span = x1 - x0;

                if (span <= 1e-9) {
                    return y1;
                }

                const double t = (abs_vel - x0) / span;
                const double y = static_cast<double>(y0) + t * static_cast<double>(y1 - y0);
                return static_cast<int>(std::lround(y));
            }

            for (std::size_t i = 1; i < N; ++i) {
                const auto [x1, y1] = lut[i];
                if (abs_vel <= x1) {
                    const auto [x0, y0] = lut[i - 1];
                    const double span = x1 - x0;

                    if (span <= 1e-9) {
                        return y1;
                    }

                    const double t = (abs_vel - x0) / span;
                    const double y = static_cast<double>(y0) + t * static_cast<double>(y1 - y0);
                    return static_cast<int>(std::lround(y));
                }
            }
            return lut.back().second;
        }

        double regime_to_double(const MotionRegime regime) {
            return static_cast<int>(regime);
        }

        constexpr double kTicksPerRevolution = 522.8;
        constexpr double kTicksPerRad = kTicksPerRevolution / (2.0 * M_PI);
        constexpr double kStandardGravity = 9.80665;
        constexpr double kDegreesToRadians = M_PI / 180.0;
        constexpr double kMicroTeslaToTesla = 1e-6;
        constexpr double kAngularVelocityCovariance = 0.02;
        constexpr double kLinearAccelerationCovariance = 0.2;
        constexpr double kMagneticFieldCovariance = 2.5e-7;

        std::string trim_copy(const std::string &input) {
            const auto begin = std::find_if_not(
                input.begin(),
                input.end(),
                [](const unsigned char ch) {
                    return std::isspace(ch) != 0;
                }
            );
            const auto end = std::find_if_not(
                input.rbegin(),
                input.rend(),
                [](const unsigned char ch) {
                    return std::isspace(ch) != 0;
                }
            ).base();

            if (begin >= end) {
                return {};
            }

            return std::string(begin, end);
        }

        std::vector<std::string> split_csv_line(const std::string &line) {
            std::vector<std::string> fields;
            std::size_t field_start = 0;

            while (field_start <= line.size()) {
                const std::size_t field_end = line.find(',', field_start);
                if (field_end == std::string::npos) {
                    fields.emplace_back(trim_copy(line.substr(field_start)));
                    break;
                }

                fields.emplace_back(trim_copy(line.substr(field_start, field_end - field_start)));
                field_start = field_end + 1;
            }

            return fields;
        }

        bool parse_int_field(const std::string &field, int &value) {
            if (field.empty()) {
                return false;
            }

            char *end = nullptr;
            errno = 0;
            const long parsed = std::strtol(field.c_str(), &end, 10);
            if (errno == ERANGE || end == field.c_str() || *end != '\0') {
                return false;
            }
            if (parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
                return false;
            }

            value = static_cast<int>(parsed);
            return true;
        }

        bool parse_double_field(const std::string &field, double &value) {
            if (field.empty()) {
                return false;
            }

            char *end = nullptr;
            errno = 0;
            const double parsed = std::strtod(field.c_str(), &end);
            if (errno == ERANGE || end == field.c_str() || *end != '\0') {
                return false;
            }

            value = parsed;
            return true;
        }
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
        const hardware_interface::HardwareInfo &info
    ) {
        if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_positions_[0] = hw_positions_[1] = 0.0;
        hw_velocities_[0] = hw_velocities_[1] = 0.0;
        hw_commands_[0] = hw_commands_[1] = 0.0;
        logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("DiffBotSystemHardware"));
        clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

        const auto &params = info_.hardware_parameters;
        if (const auto it = params.find("pwm_max"); it != params.end()) {
            pwm_max_ = std::stoi(it->second);
        }
        if (const auto it = params.find("pwm_min"); it != params.end()) {
            pwm_min_ = std::stoi(it->second);
        }

        if (const auto it = params.find("publish_debug_telemetry"); it != params.end()) {
            const std::string value = it->second;
            publish_debug_telemetry_ = value == "1" || value == "true" || value == "True" || value == "TRUE";
        }
        if (const auto it = params.find("debug_pub_decimation"); it != params.end()) {
            debug_pub_decimation_ = std::max(1, std::stoi(it->second));
        }
        if (const auto it = params.find("use_regime_luts"); it != params.end()) {
            const std::string value = it->second;
            use_regime_luts_ = value == "1" || value == "true" || value == "True" || value == "TRUE";
        }
        if (const auto it = params.find("command_deadband_rad_s"); it != params.end()) {
            command_deadband_rad_s_ = std::max(0.0, std::stod(it->second));
        }
        if (const auto it = params.find("spin_min_wheel_rad_s"); it != params.end()) {
            spin_min_wheel_rad_s_ = std::max(0.0, std::stod(it->second));
        }
        if (const auto it = params.find("straight_delta_abs_rad_s"); it != params.end()) {
            straight_delta_abs_rad_s_ = std::max(0.0, std::stod(it->second));
        }
        if (const auto it = params.find("straight_delta_ratio"); it != params.end()) {
            straight_delta_ratio_ = std::max(0.0, std::stod(it->second));
        }
        if (const auto it = params.find("pid_kp"); it != params.end()) {
            pid_kp_ = std::stod(it->second);
        }
        if (const auto it = params.find("pid_ki"); it != params.end()) {
            pid_ki_ = std::stod(it->second);
        }
        if (const auto it = params.find("pid_kd"); it != params.end()) {
            pid_kd_ = std::stod(it->second);
        }
        if (const auto it = params.find("pid_output_limit_pwm"); it != params.end()) {
            pid_output_limit_pwm_ = std::max(0.0, std::stod(it->second));
        }
        if (const auto it = params.find("pid_integral_limit"); it != params.end()) {
            pid_integral_limit_ = std::max(0.0, std::stod(it->second));
        }
        if (const auto it = params.find("imu_frame_id"); it != params.end() && !it->second.empty()) {
            imu_frame_id_ = it->second;
        }

        if (pwm_min_ < 0 || pwm_max_ <= pwm_min_) {
            RCLCPP_ERROR(
                rclcpp::get_logger("DiffBotSystemHardware"),
                "Expected 0 <= pwm_min < pwm_max, got pwm_min=%d pwm_max=%d",
                pwm_min_,
                pwm_max_
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "PWM map params: pwm_min=%d pwm_max=%d",
            pwm_min_,
            pwm_max_
        );
        RCLCPP_INFO(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Debug telemetry: enabled=%s decimation=%d topic=/diffbot_hw_debug",
            publish_debug_telemetry_ ? "true" : "false",
            debug_pub_decimation_
        );
        RCLCPP_INFO(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "Regime LUT selector: enabled=%s deadband=%.3f spin_min=%.3f straight_delta_abs=%.3f straight_delta_ratio=%.3f",
            use_regime_luts_ ? "true" : "false",
            command_deadband_rad_s_,
            spin_min_wheel_rad_s_,
            straight_delta_abs_rad_s_,
            straight_delta_ratio_
        );
        RCLCPP_INFO(
            rclcpp::get_logger("DiffBotSystemHardware"),
            "PID params: kp=%.6f ki=%.6f kd=%.6f out_limit_pwm=%.3f integral_limit=%.3f",
            pid_kp_,
            pid_ki_,
            pid_kd_,
            pid_output_limit_pwm_,
            pid_integral_limit_
        );

        try {
            imu_node_ = std::make_shared<rclcpp::Node>("diffbot_hw_imu_pub");
            imu_pub_ = imu_node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", rclcpp::SensorDataQoS());
            mag_pub_ =
                imu_node_->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", rclcpp::SensorDataQoS());
        } catch (const std::exception &e) {
            RCLCPP_WARN(
                rclcpp::get_logger("DiffBotSystemHardware"),
                "Failed to create IMU publisher: %s",
                e.what()
            );
        }

        if (publish_debug_telemetry_) {
            try {
                debug_node_ = std::make_shared<rclcpp::Node>("diffbot_hw_debug_pub");
                debug_pub_ = debug_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/diffbot_hw_debug", 20);
            } catch (const std::exception &e) {
                RCLCPP_WARN(
                    rclcpp::get_logger("DiffBotSystemHardware"),
                    "Failed to create debug telemetry publisher: %s",
                    e.what()
                );
                publish_debug_telemetry_ = false;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Hardware interface initialized");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    DiffBotSystemHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> states;

        states.emplace_back("left_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[0]);
        states.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]);
        states.emplace_back("right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[1]);
        states.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]);

        return states;
    }

    std::vector<hardware_interface::CommandInterface>
    DiffBotSystemHardware::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> cmds;

        cmds.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
        cmds.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]);

        return cmds;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
        const rclcpp_lifecycle::State &
    ) {
        try {
            serial_read_.open("/dev/imu-encoder-node");
            serial_read_.set_option(asio::serial_port_base::baud_rate(115200));
            serial_read_.set_option(asio::serial_port_base::character_size(8));
            serial_read_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
            serial_read_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));

            serial_write_.open("/dev/motor-controller");
            serial_write_.set_option(asio::serial_port_base::baud_rate(115200));
            serial_write_.set_option(asio::serial_port_base::character_size(8));
            serial_write_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
            serial_write_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Serial open failed: %s", e.what());

            return hardware_interface::CallbackReturn::ERROR;
        }

        io_read_.restart();
        io_write_.restart();
        rx_buffer_.clear();
        serial_line_count_ = 0;
        serial_parse_reject_count_ = 0;
        serial_encoder_sample_count_ = 0;
        serial_imu_sample_count_ = 0;
        serial_mag_sample_count_ = 0;
        serial_last_field_count_ = 0;
        {
            std::lock_guard encoder_lock(encoder_mutex_);
            left_ticks_ = 0;
            right_ticks_ = 0;
        }
        {
            std::lock_guard imu_lock(imu_mutex_);
            latest_imu_sample_ = {};
            last_published_imu_sequence_ = 0;
        }
        {
            std::lock_guard write_lock(write_mutex_);
            pending_write_cmd_.clear();
            write_cmd_ready_ = false;
        }

        running_ = true;
        start_async_read();

        io_read_thread_ = std::thread([this] {
            io_read_.run();
        });
        io_write_thread_ = std::thread([this] {
            while (running_) {
                std::string cmd;
                {
                    std::unique_lock lock(write_mutex_);
                    write_cv_.wait(lock, [this] {
                        return !running_ || write_cmd_ready_;
                    });

                    if (!running_) {
                        return;
                    }

                    cmd = std::move(pending_write_cmd_);
                    write_cmd_ready_ = false;
                }

                std::error_code ec;
                asio::write(serial_write_, asio::buffer(cmd), ec);
                if (ec) {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("DiffBotSystemHardware"),
                        "Serial write failed: %s",
                        ec.message().c_str()
                    );
                }
            }
        });

        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Hardware activated with serial");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State &
    ) {
        running_ = false;
        write_cv_.notify_all();
        io_read_.stop();

        if (io_write_thread_.joinable())
            io_write_thread_.join();

        if (io_read_thread_.joinable())
            io_read_thread_.join();

        if (serial_write_.is_open())
            serial_write_.close();

        if (serial_read_.is_open())
            serial_read_.close();

        RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Hardware deactivated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type DiffBotSystemHardware::read(
        const rclcpp::Time &time,
        const rclcpp::Duration &period
    ) {
        int64_t left_ticks = 0;
        int64_t right_ticks = 0;
        {
            std::lock_guard lock(encoder_mutex_);
            left_ticks = left_ticks_;
            right_ticks = right_ticks_;
            left_ticks_ = 0;
            right_ticks_ = 0;
        }

        const double dl = static_cast<double>(left_ticks) / kTicksPerRad;
        const double dr = static_cast<double>(right_ticks) / kTicksPerRad;

        hw_positions_[0] += dl;
        hw_positions_[1] += dr;

        hw_velocities_[0] = dl / period.seconds();
        hw_velocities_[1] = dr / period.seconds();
        publish_imu(time);

        return hardware_interface::return_type::OK;
    }

    MotionRegime DiffBotSystemHardware::select_motion_regime(
        const double cmd_left,
        const double cmd_right,
        double *cmd_abs_delta_out
    ) const {
        const double abs_left = std::abs(cmd_left);
        const double abs_right = std::abs(cmd_right);
        const double max_abs = std::max(abs_left, abs_right);
        const double delta_abs = std::abs(cmd_left - cmd_right);

        if (cmd_abs_delta_out != nullptr) {
            *cmd_abs_delta_out = delta_abs;
        }

        if (max_abs <= command_deadband_rad_s_) {
            return MotionRegime::Straight;
        }

        const bool opposite_sign = (cmd_left > 0.0 && cmd_right < 0.0) || (cmd_left < 0.0 && cmd_right > 0.0);
        if (opposite_sign && abs_left >= spin_min_wheel_rad_s_ && abs_right >= spin_min_wheel_rad_s_) {
            return MotionRegime::Spin;
        }

        const double straight_delta_threshold = straight_delta_abs_rad_s_ + straight_delta_ratio_ * max_abs;
        if (delta_abs <= straight_delta_threshold) {
            return MotionRegime::Straight;
        }

        return MotionRegime::Arc;
    }

    hardware_interface::return_type DiffBotSystemHardware::write(
        const rclcpp::Time &,
        const rclcpp::Duration &period
    ) {
        const int pwm_max = pwm_max_;
        const int pwm_min = pwm_min_;

        const double cmd_left = hw_commands_[0];
        const double cmd_right = hw_commands_[1];
        double cmd_abs_delta = 0.0;
        const MotionRegime regime = use_regime_luts_
                                        ? select_motion_regime(cmd_left, cmd_right, &cmd_abs_delta)
                                        : MotionRegime::Straight;

        auto vel_to_pwm = [pwm_max, pwm_min, regime](const int wheel_idx, const double vel_rad_s) -> int {
            if (vel_rad_s == 0.0) {
                return 0;
            }

            const double abs_vel = std::abs(vel_rad_s);
            const bool forward = vel_rad_s > 0.0;

            int lut_pwm = 0;
            if (regime == MotionRegime::Spin) {
                lut_pwm = wheel_idx == 0
                              ? interpolate_lut_pwm(kLeftSpinVelPwmLut, abs_vel)
                              : interpolate_lut_pwm(kRightSpinVelPwmLut, abs_vel);
            } else if (regime == MotionRegime::Arc) {
                if (wheel_idx == 0) {
                    lut_pwm = forward
                                  ? interpolate_lut_pwm(kLeftFwdArcVelPwmLut, abs_vel)
                                  : interpolate_lut_pwm(kLeftRevArcVelPwmLut, abs_vel);
                } else {
                    lut_pwm = forward
                                  ? interpolate_lut_pwm(kRightFwdArcVelPwmLut, abs_vel)
                                  : interpolate_lut_pwm(kRightRevArcVelPwmLut, abs_vel);
                }
            } else {
                if (wheel_idx == 0) {
                    lut_pwm = forward
                                  ? interpolate_lut_pwm(kLeftFwdVelPwmLut, abs_vel)
                                  : interpolate_lut_pwm(kLeftRevVelPwmLut, abs_vel);
                } else {
                    lut_pwm = forward
                                  ? interpolate_lut_pwm(kRightFwdVelPwmLut, abs_vel)
                                  : interpolate_lut_pwm(kRightRevVelPwmLut, abs_vel);
                }
            }

            const int pwm = std::clamp(lut_pwm, pwm_min, pwm_max);

            return vel_rad_s > 0.0 ? pwm : -pwm;
        };

        const int left_ff_pwm = vel_to_pwm(0, cmd_left);
        const int right_ff_pwm = vel_to_pwm(1, cmd_right);

        const double dt = period.seconds();

        auto compute_pid_corr = [this, dt](const int idx, const double cmd, const double meas) -> double {
            if (std::abs(cmd) <= command_deadband_rad_s_) {
                pid_integral_[idx] = 0.0;
                pid_prev_error_valid_[idx] = false;
                last_pid_corr_pwm_[idx] = 0.0;

                return 0.0;
            }

            const double err = cmd - meas;
            const double p_term = pid_kp_ * err;

            const double i_candidate = std::clamp(
                pid_integral_[idx] + err * dt,
                -pid_integral_limit_,
                pid_integral_limit_
            );
            const double i_term = pid_ki_ * i_candidate;

            double d_term = 0.0;
            if (pid_prev_error_valid_[idx]) {
                const double derr = (err - pid_prev_error_[idx]) / dt;
                d_term = pid_kd_ * derr;
            }

            const double corr_raw = p_term + i_term + d_term;
            const double corr = std::clamp(corr_raw, -pid_output_limit_pwm_, pid_output_limit_pwm_);

            const bool clamped = std::abs(corr - corr_raw) > 1e-9;
            const bool clamp_pushes_same_dir_as_error = (corr - corr_raw) * err > 0.0;

            if (!clamped || !clamp_pushes_same_dir_as_error) {
                pid_integral_[idx] = i_candidate;
            }

            pid_prev_error_[idx] = err;
            pid_prev_error_valid_[idx] = true;
            last_pid_corr_pwm_[idx] = corr;

            return corr;
        };

        double left_pid_corr_pwm = 0.0;
        double right_pid_corr_pwm = 0.0;

        if (dt > 1e-6) {
            left_pid_corr_pwm = compute_pid_corr(0, cmd_left, hw_velocities_[0]);
            right_pid_corr_pwm = compute_pid_corr(1, cmd_right, hw_velocities_[1]);
        }

        int left_pwm = std::clamp(left_ff_pwm + static_cast<int>(std::round(left_pid_corr_pwm)), -pwm_max, pwm_max);
        int right_pwm = std::clamp(right_ff_pwm + static_cast<int>(std::round(right_pid_corr_pwm)), -pwm_max, pwm_max);

        last_out_pwm_[0] = left_pwm;
        last_out_pwm_[1] = right_pwm;

        publish_debug(
            cmd_left,
            cmd_right,
            left_ff_pwm,
            right_ff_pwm,
            left_pid_corr_pwm,
            right_pid_corr_pwm,
            left_pwm,
            right_pwm,
            regime,
            cmd_abs_delta
        );

        char cmd[64] = {};

        const int cmd_len = std::snprintf(
            cmd,
            sizeof(cmd),
            "%d,%d\n",
            left_pwm,
            right_pwm
        );

        if (cmd_len <= 0) {
            RCLCPP_ERROR(rclcpp::get_logger("DiffBotSystemHardware"), "Failed to format motor command");

            return hardware_interface::return_type::ERROR;
        }

        if (!running_) {
            return hardware_interface::return_type::OK;
        }

        {
            std::lock_guard lock(write_mutex_);
            pending_write_cmd_.assign(cmd, static_cast<std::size_t>(cmd_len));
            write_cmd_ready_ = true;
        }

        write_cv_.notify_one();

        return hardware_interface::return_type::OK;
    }

    void DiffBotSystemHardware::start_async_read() {
        serial_read_.async_read_some(
            asio::buffer(temp_rx_),
            [this](const std::error_code ec, const std::size_t len) {
                if (!ec && running_) {
                    rx_buffer_.append(temp_rx_.data(), len);

                    process_rx_buffer();

                    start_async_read();
                } else if (ec && running_) {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("DiffBotSystemHardware"),
                        "Serial read failed: %s",
                        ec.message().c_str()
                    );
                }
            });
    }

    void DiffBotSystemHardware::process_rx_buffer() {
        size_t pos;

        while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
            std::string line = rx_buffer_.substr(0, pos);

            rx_buffer_.erase(0, pos + 1);

            parse_encoder_line(line);
        }
    }

    void DiffBotSystemHardware::parse_encoder_line(const std::string &line) {
        const auto fields = split_csv_line(trim_copy(line));
        ++serial_line_count_;
        serial_last_field_count_ = fields.size();
        if (fields.size() != 2 && fields.size() != 8 && fields.size() != 11) {
            ++serial_parse_reject_count_;
            return;
        }

        int left_ticks = 0;
        int right_ticks = 0;
        if (!parse_int_field(fields[0], left_ticks) || !parse_int_field(fields[1], right_ticks)) {
            ++serial_parse_reject_count_;
            return;
        }

        ImuSampleSnapshot imu_sample;
        if (fields.size() == 8 || fields.size() == 11) {
            double ax_g = 0.0;
            double ay_g = 0.0;
            double az_g = 0.0;
            double gx_dps = 0.0;
            double gy_dps = 0.0;
            double gz_dps = 0.0;

            if (
                !parse_double_field(fields[2], ax_g) ||
                !parse_double_field(fields[3], ay_g) ||
                !parse_double_field(fields[4], az_g) ||
                !parse_double_field(fields[5], gx_dps) ||
                !parse_double_field(fields[6], gy_dps) ||
                !parse_double_field(fields[7], gz_dps)
            ) {
                ++serial_parse_reject_count_;
                return;
            }

            imu_sample.ax_mps2 = ax_g * kStandardGravity;
            imu_sample.ay_mps2 = ay_g * kStandardGravity;
            imu_sample.az_mps2 = az_g * kStandardGravity;
            imu_sample.gx_rad_s = gx_dps * kDegreesToRadians;
            imu_sample.gy_rad_s = gy_dps * kDegreesToRadians;
            imu_sample.gz_rad_s = gz_dps * kDegreesToRadians;

            if (fields.size() == 11) {
                double mx_ut = 0.0;
                double my_ut = 0.0;
                double mz_ut = 0.0;

                if (
                    !parse_double_field(fields[8], mx_ut) ||
                    !parse_double_field(fields[9], my_ut) ||
                    !parse_double_field(fields[10], mz_ut)
                ) {
                    ++serial_parse_reject_count_;
                    return;
                }

                imu_sample.mx_t = mx_ut * kMicroTeslaToTesla;
                imu_sample.my_t = my_ut * kMicroTeslaToTesla;
                imu_sample.mz_t = mz_ut * kMicroTeslaToTesla;
                imu_sample.mag_valid = true;
            }

            imu_sample.valid = true;
        }

        {
            std::lock_guard lock(encoder_mutex_);
            left_ticks_ += left_ticks;
            right_ticks_ += right_ticks;
        }
        ++serial_encoder_sample_count_;

        if (imu_sample.valid) {
            std::lock_guard lock(imu_mutex_);
            imu_sample.sequence = latest_imu_sample_.sequence + 1;
            latest_imu_sample_ = imu_sample;
            ++serial_imu_sample_count_;
            if (imu_sample.mag_valid) {
                ++serial_mag_sample_count_;
            }
        }
    }

    void DiffBotSystemHardware::publish_imu(const rclcpp::Time &time) {
        if (!imu_pub_) {
            return;
        }

        ImuSampleSnapshot imu_sample;
        {
            std::lock_guard lock(imu_mutex_);
            if (!latest_imu_sample_.valid || latest_imu_sample_.sequence == last_published_imu_sequence_) {
                return;
            }

            imu_sample = latest_imu_sample_;
            last_published_imu_sequence_ = imu_sample.sequence;
        }

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = time.nanoseconds() > 0 ? time : imu_node_->get_clock()->now();
        msg.header.frame_id = imu_frame_id_;
        msg.orientation_covariance[0] = -1.0;
        msg.angular_velocity.x = imu_sample.gx_rad_s;
        msg.angular_velocity.y = imu_sample.gy_rad_s;
        msg.angular_velocity.z = imu_sample.gz_rad_s;
        msg.angular_velocity_covariance[0] = kAngularVelocityCovariance;
        msg.angular_velocity_covariance[4] = kAngularVelocityCovariance;
        msg.angular_velocity_covariance[8] = kAngularVelocityCovariance;
        msg.linear_acceleration.x = imu_sample.ax_mps2;
        msg.linear_acceleration.y = imu_sample.ay_mps2;
        msg.linear_acceleration.z = imu_sample.az_mps2;
        msg.linear_acceleration_covariance[0] = kLinearAccelerationCovariance;
        msg.linear_acceleration_covariance[4] = kLinearAccelerationCovariance;
        msg.linear_acceleration_covariance[8] = kLinearAccelerationCovariance;
        imu_pub_->publish(msg);

        if (mag_pub_ && imu_sample.mag_valid) {
            sensor_msgs::msg::MagneticField mag_msg;
            mag_msg.header = msg.header;
            mag_msg.magnetic_field.x = imu_sample.mx_t;
            mag_msg.magnetic_field.y = imu_sample.my_t;
            mag_msg.magnetic_field.z = imu_sample.mz_t;
            mag_msg.magnetic_field_covariance[0] = kMagneticFieldCovariance;
            mag_msg.magnetic_field_covariance[4] = kMagneticFieldCovariance;
            mag_msg.magnetic_field_covariance[8] = kMagneticFieldCovariance;
            mag_pub_->publish(mag_msg);
        }
    }

    void DiffBotSystemHardware::publish_debug(
        const double cmd_left,
        const double cmd_right,
        const int left_ff_pwm,
        const int right_ff_pwm,
        double left_pid_corr_pwm,
        double right_pid_corr_pwm,
        const int left_pwm,
        const int right_pwm,
        const MotionRegime regime,
        const double cmd_abs_delta
    ) {
        if (publish_debug_telemetry_ && debug_pub_) {
            ++debug_pub_counter_;
            if (debug_pub_counter_ >= debug_pub_decimation_) {
                debug_pub_counter_ = 0;
                std_msgs::msg::Float64MultiArray msg;
                msg.data = {
                    cmd_left, cmd_right,
                    hw_velocities_[0], hw_velocities_[1],
                    static_cast<double>(left_ff_pwm), static_cast<double>(right_ff_pwm),
                    left_pid_corr_pwm, right_pid_corr_pwm,
                    static_cast<double>(left_pwm), static_cast<double>(right_pwm),
                    regime_to_double(regime),
                    cmd_abs_delta,
                    static_cast<double>(serial_line_count_.load()),
                    static_cast<double>(serial_parse_reject_count_.load()),
                    static_cast<double>(serial_encoder_sample_count_.load()),
                    static_cast<double>(serial_imu_sample_count_.load()),
                    static_cast<double>(serial_mag_sample_count_.load()),
                    static_cast<double>(serial_last_field_count_.load())
                };
                debug_pub_->publish(msg);
            }
        }
    }
}

PLUGINLIB_EXPORT_CLASS(
    diffbot::DiffBotSystemHardware,
    hardware_interface::SystemInterface
)
