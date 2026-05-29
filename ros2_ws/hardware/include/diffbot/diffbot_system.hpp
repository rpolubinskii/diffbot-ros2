#ifndef DIFFBOT__DIFFBOT_SYSTEM_HPP_
#define DIFFBOT__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <asio.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace diffbot {
    enum class MotionRegime : int {
        Straight = 0,
        Arc = 1,
        Spin = 2,
    };

    class DiffBotSystemHardware : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info
        ) override;

        std::vector<hardware_interface::StateInterface>
        export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface>
        export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state
        ) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state
        ) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time,
            const rclcpp::Duration &period
        ) override;

        void publish_debug(
            double cmd_left,
            double cmd_right,
            int left_ff_pwm,
            int right_ff_pwm,
            double left_pid_corr_pwm,
            double right_pid_corr_pwm,
            int left_pwm,
            int right_pwm,
            MotionRegime regime,
            double cmd_abs_delta
        );

        hardware_interface::return_type write(
            const rclcpp::Time &time,
            const rclcpp::Duration &period
        ) override;

        rclcpp::Logger get_logger() const { return *logger_; }

        rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

    private:
        struct ImuSampleSnapshot {
            double ax_mps2 = 0.0;
            double ay_mps2 = 0.0;
            double az_mps2 = 0.0;
            double gx_rad_s = 0.0;
            double gy_rad_s = 0.0;
            double gz_rad_s = 0.0;
            double mx_t = 0.0;
            double my_t = 0.0;
            double mz_t = 0.0;
            bool mag_valid = false;
            std::uint64_t sequence = 0;
            bool valid = false;
        };

        int pwm_max_ = 1023;
        int pwm_min_ = 350;
        bool use_regime_luts_ = true;
        double command_deadband_rad_s_ = 0.05;
        double spin_min_wheel_rad_s_ = 1.0;
        double straight_delta_abs_rad_s_ = 0.35;
        double straight_delta_ratio_ = 0.12;
        bool publish_debug_telemetry_ = true;
        int debug_pub_decimation_ = 1;
        int debug_pub_counter_ = 0;

        double pid_kp_ = 35.0;
        double pid_ki_ = 12.0;
        double pid_kd_ = 0.0;

        double pid_integral_[2] = {0.0, 0.0};
        double pid_prev_error_[2] = {0.0, 0.0};
        bool pid_prev_error_valid_[2] = {false, false};

        // limit PID correction contribution in PWM units
        double pid_output_limit_pwm_ = 300.0;
        // limit integral term (in error*sec units)
        double pid_integral_limit_ = 50.0;

        // store last PID correction for debug telemetry slots 6/7
        double last_pid_corr_pwm_[2] = {0.0, 0.0};

        double hw_positions_[2] = {};
        double hw_velocities_[2] = {};
        double hw_commands_[2] = {};
        int last_out_pwm_[2] = {};

        asio::io_context io_read_;
        asio::serial_port serial_read_{io_read_};
        std::thread io_read_thread_;

        asio::io_context io_write_;
        asio::serial_port serial_write_{io_write_};
        std::thread io_write_thread_;
        std::mutex write_mutex_;
        std::condition_variable write_cv_;
        std::string pending_write_cmd_;
        bool write_cmd_ready_ = false;

        std::mutex encoder_mutex_;
        int64_t left_ticks_ = 0;
        int64_t right_ticks_ = 0;

        std::mutex imu_mutex_;
        ImuSampleSnapshot latest_imu_sample_{};
        std::uint64_t last_published_imu_sequence_ = 0;
        std::string imu_frame_id_ = "imu_link";

        std::string rx_buffer_;
        std::array<char, 256> temp_rx_ = {};
        std::atomic<bool> running_{false};
        std::atomic<std::uint64_t> serial_line_count_{0};
        std::atomic<std::uint64_t> serial_parse_reject_count_{0};
        std::atomic<std::uint64_t> serial_encoder_sample_count_{0};
        std::atomic<std::uint64_t> serial_imu_sample_count_{0};
        std::atomic<std::uint64_t> serial_mag_sample_count_{0};
        std::atomic<std::uint64_t> serial_last_field_count_{0};

        void start_async_read();

        void process_rx_buffer();

        void parse_encoder_line(const std::string &line);

        void publish_imu(const rclcpp::Time &time);

        std::shared_ptr<rclcpp::Logger> logger_;
        rclcpp::Clock::SharedPtr clock_;
        rclcpp::Node::SharedPtr imu_node_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
        rclcpp::Node::SharedPtr debug_node_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;

        MotionRegime select_motion_regime(double cmd_left, double cmd_right, double *cmd_abs_delta_out) const;
    };
} // namespace diffbot

#endif  // DIFFBOT__DIFFBOT_SYSTEM_HPP_
