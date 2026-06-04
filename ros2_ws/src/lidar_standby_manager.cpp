#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <rcl_interfaces/msg/logger_level.hpp>
#include <rcl_interfaces/srv/set_logger_levels.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <cstdint>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace diffbot
{

using namespace std::chrono_literals;

class LidarStandbyManager : public rclcpp::Node
{
public:
  LidarStandbyManager()
  : Node("diffbot_lidar_standby_manager")
  {
    idle_timeout_sec_ = declare_parameter<double>("idle_timeout_sec", 10.0);
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 3.0);
    start_motor_service_ = declare_parameter<std::string>("start_motor_service", "/start_motor");
    stop_motor_service_ = declare_parameter<std::string>("stop_motor_service", "/stop_motor");
    pause_rtabmap_service_ = declare_parameter<std::string>("pause_rtabmap_service", "/rtabmap/pause");
    resume_rtabmap_service_ = declare_parameter<std::string>("resume_rtabmap_service", "/rtabmap/resume");
    pause_odom_service_ = declare_parameter<std::string>("pause_odom_service", "/pause_odom");
    resume_odom_service_ = declare_parameter<std::string>("resume_odom_service", "/resume_odom");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    manage_consumer_log_levels_ = declare_parameter<bool>("manage_consumer_log_levels", true);
    idle_consumer_log_level_ = declare_parameter<std::string>("idle_consumer_log_level", "error");
    active_consumer_log_level_ = declare_parameter<std::string>("active_consumer_log_level", "info");
    rtabmap_logger_service_ =
      declare_parameter<std::string>("rtabmap_logger_service", "/rtabmap/set_logger_levels");
    icp_odom_logger_service_ =
      declare_parameter<std::string>("icp_odom_logger_service", "/icp_odometry/set_logger_levels");
    rtabmap_logger_name_ = declare_parameter<std::string>("rtabmap_logger_name", "rtabmap");
    icp_odom_logger_name_ = declare_parameter<std::string>("icp_odom_logger_name", "icp_odometry");

    start_motor_client_ = create_client<std_srvs::srv::Empty>(start_motor_service_);
    stop_motor_client_ = create_client<std_srvs::srv::Empty>(stop_motor_service_);
    pause_rtabmap_client_ = create_client<std_srvs::srv::Empty>(pause_rtabmap_service_);
    resume_rtabmap_client_ = create_client<std_srvs::srv::Empty>(resume_rtabmap_service_);
    pause_odom_client_ = create_client<std_srvs::srv::Empty>(pause_odom_service_);
    resume_odom_client_ = create_client<std_srvs::srv::Empty>(resume_odom_service_);
    rtabmap_logger_client_ =
      create_client<rcl_interfaces::srv::SetLoggerLevels>(rtabmap_logger_service_);
    icp_odom_logger_client_ =
      create_client<rcl_interfaces::srv::SetLoggerLevels>(icp_odom_logger_service_);

    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    navigate_to_pose_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", status_qos,
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        handle_status(msg, ActionStatusSource::NavigateToPose);
      });
    navigate_through_poses_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_through_poses/_action/status", status_qos,
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        handle_status(msg, ActionStatusSource::NavigateThroughPoses);
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr) {
        {
          std::lock_guard<std::mutex> lock(scan_mutex_);
          ++scan_count_;
        }
        scan_cv_.notify_all();
      });

    schedule_standby_timer();

    RCLCPP_INFO(
      get_logger(),
      "Lidar standby manager started: idle_timeout_sec=%.3f, scan_timeout_sec=%.3f, scan_topic=%s",
      idle_timeout_sec_, scan_timeout_sec_, scan_topic_.c_str());
  }

  ~LidarStandbyManager() override
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      shutting_down_ = true;
      ++state_generation_;
      if (standby_timer_) {
        standby_timer_->cancel();
      }
    }
    scan_cv_.notify_all();

    std::lock_guard<std::mutex> workers_lock(workers_mutex_);
    for (auto & worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

private:
  enum class ActionStatusSource
  {
    NavigateToPose,
    NavigateThroughPoses
  };

  enum class DesiredState
  {
    Active,
    Idle
  };

  using EmptyClient = rclcpp::Client<std_srvs::srv::Empty>;
  using LoggerLevelsClient = rclcpp::Client<rcl_interfaces::srv::SetLoggerLevels>;

  void handle_status(
    const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg,
    ActionStatusSource source)
  {
    const bool source_active = has_active_goal(*msg);

    bool became_active = false;
    bool became_idle = false;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (source == ActionStatusSource::NavigateToPose) {
        navigate_to_pose_active_ = source_active;
      } else {
        navigate_through_poses_active_ = source_active;
      }

      const bool new_navigation_active = navigate_to_pose_active_ || navigate_through_poses_active_;
      if (new_navigation_active == navigation_active_) {
        return;
      }

      navigation_active_ = new_navigation_active;
      ++state_generation_;
      became_active = navigation_active_;
      became_idle = !navigation_active_;

      if (standby_timer_) {
        standby_timer_->cancel();
      }
    }

    if (became_active) {
      const auto generation = current_generation();
      RCLCPP_INFO(get_logger(), "Nav2 goal activity detected; resuming lidar-dependent consumers");
      launch_worker([this, generation]() { run_active_sequence(generation); });
    } else if (became_idle) {
      RCLCPP_INFO(
        get_logger(), "No active Nav2 goals; scheduling lidar standby in %.3f seconds",
        idle_timeout_sec_);
      schedule_standby_timer();
    }
  }

  static bool has_active_goal(const action_msgs::msg::GoalStatusArray & msg)
  {
    for (const auto & status : msg.status_list) {
      if (
        status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        status.status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
      {
        return true;
      }
    }
    return false;
  }

  void schedule_standby_timer()
  {
    const auto generation = current_generation();
    const auto delay = std::chrono::duration<double>(std::max(0.0, idle_timeout_sec_));
    standby_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(delay),
      [this, generation]() {
        bool run_standby = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          run_standby = !shutting_down_ && generation == state_generation_ && !navigation_active_;
          if (run_standby && standby_timer_) {
            standby_timer_->cancel();
          }
        }
        if (run_standby) {
          launch_worker([this, generation]() { run_idle_sequence(generation); });
        }
      });
  }

  std::uint64_t current_generation()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_generation_;
  }

  bool should_continue(std::uint64_t generation, DesiredState desired_state)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (shutting_down_ || generation != state_generation_) {
      return false;
    }

    if (desired_state == DesiredState::Active) {
      return navigation_active_;
    }
    return !navigation_active_;
  }

  void launch_worker(std::function<void()> work)
  {
    std::lock_guard<std::mutex> lock(workers_mutex_);
    workers_.emplace_back(std::move(work));
  }

  void run_active_sequence(std::uint64_t generation)
  {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (!should_continue(generation, DesiredState::Active)) {
      return;
    }

    const auto baseline_scan_count = observed_scan_count();
    if (!call_empty_service(start_motor_client_, start_motor_service_, generation, DesiredState::Active)) {
      return;
    }

    if (!wait_for_fresh_scan(baseline_scan_count, generation)) {
      return;
    }

    set_consumer_log_levels(active_consumer_log_level_, generation, DesiredState::Active);

    if (!call_empty_service(resume_odom_client_, resume_odom_service_, generation, DesiredState::Active)) {
      return;
    }
    call_empty_service(resume_rtabmap_client_, resume_rtabmap_service_, generation, DesiredState::Active);
  }

  void run_idle_sequence(std::uint64_t generation)
  {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (!should_continue(generation, DesiredState::Idle)) {
      return;
    }

    if (!call_empty_service(pause_rtabmap_client_, pause_rtabmap_service_, generation, DesiredState::Idle)) {
      return;
    }
    if (!call_empty_service(pause_odom_client_, pause_odom_service_, generation, DesiredState::Idle)) {
      return;
    }
    set_consumer_log_levels(idle_consumer_log_level_, generation, DesiredState::Idle);
    call_empty_service(stop_motor_client_, stop_motor_service_, generation, DesiredState::Idle);
  }

  bool call_empty_service(
    const EmptyClient::SharedPtr & client,
    const std::string & service_name,
    std::uint64_t generation,
    DesiredState desired_state)
  {
    while (rclcpp::ok() && should_continue(generation, desired_state)) {
      if (!client->wait_for_service(1s)) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Service %s is not available; retrying while this standby state is still current",
          service_name.c_str());
        continue;
      }

      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      auto future = client->async_send_request(request);
      while (rclcpp::ok() && should_continue(generation, desired_state)) {
        if (future.wait_for(100ms) != std::future_status::ready) {
          continue;
        }

        try {
          future.get();
        } catch (const std::exception & ex) {
          RCLCPP_ERROR(
            get_logger(), "Service %s call failed: %s; retrying", service_name.c_str(), ex.what());
          break;
        }

        RCLCPP_INFO(get_logger(), "Called %s", service_name.c_str());
        return true;
      }
    }

    return false;
  }

  void set_consumer_log_levels(
    const std::string & level_name,
    std::uint64_t generation,
    DesiredState desired_state)
  {
    if (!manage_consumer_log_levels_) {
      return;
    }

    std::uint32_t level = rcl_interfaces::msg::LoggerLevel::LOG_LEVEL_INFO;
    if (!parse_log_level(level_name, level)) {
      RCLCPP_ERROR(get_logger(), "Invalid consumer logger level '%s'", level_name.c_str());
      return;
    }

    call_logger_level_service(
      rtabmap_logger_client_, rtabmap_logger_service_, rtabmap_logger_name_, level, generation,
      desired_state);
    call_logger_level_service(
      icp_odom_logger_client_, icp_odom_logger_service_, icp_odom_logger_name_, level, generation,
      desired_state);
  }

  static bool parse_log_level(const std::string & level_name, std::uint32_t & level)
  {
    std::string normalized = level_name;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });

    if (normalized == "debug") {
      level = rcl_interfaces::msg::LoggerLevel::LOG_LEVEL_DEBUG;
    } else if (normalized == "info") {
      level = rcl_interfaces::msg::LoggerLevel::LOG_LEVEL_INFO;
    } else if (normalized == "warn" || normalized == "warning") {
      level = rcl_interfaces::msg::LoggerLevel::LOG_LEVEL_WARN;
    } else if (normalized == "error") {
      level = rcl_interfaces::msg::LoggerLevel::LOG_LEVEL_ERROR;
    } else if (normalized == "fatal") {
      level = rcl_interfaces::msg::LoggerLevel::LOG_LEVEL_FATAL;
    } else {
      return false;
    }

    return true;
  }

  void call_logger_level_service(
    const LoggerLevelsClient::SharedPtr & client,
    const std::string & service_name,
    const std::string & logger_name,
    std::uint32_t level,
    std::uint64_t generation,
    DesiredState desired_state)
  {
    if (!should_continue(generation, desired_state)) {
      return;
    }

    if (!client->wait_for_service(500ms)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Logger service %s is not available; standby will continue without log-level adjustment",
        service_name.c_str());
      return;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetLoggerLevels::Request>();
    rcl_interfaces::msg::LoggerLevel logger_level;
    logger_level.name = logger_name;
    logger_level.level = level;
    request->levels.push_back(logger_level);

    auto future = client->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (
      rclcpp::ok() && should_continue(generation, desired_state) &&
      std::chrono::steady_clock::now() < deadline)
    {
      if (future.wait_for(100ms) != std::future_status::ready) {
        continue;
      }

      const auto response = future.get();
      if (!response->results.empty() && !response->results.front().successful) {
        RCLCPP_WARN(
          get_logger(), "Failed to set logger %s through %s: %s",
          logger_name.c_str(), service_name.c_str(), response->results.front().reason.c_str());
      }
      RCLCPP_INFO(get_logger(), "Set logger %s through %s", logger_name.c_str(), service_name.c_str());
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Timed out setting logger %s through %s; standby will continue",
      logger_name.c_str(), service_name.c_str());
  }

  std::uint64_t observed_scan_count()
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    return scan_count_;
  }

  bool wait_for_fresh_scan(std::uint64_t baseline_scan_count, std::uint64_t generation)
  {
    const auto timeout = std::chrono::duration<double>(std::max(0.0, scan_timeout_sec_));
    std::unique_lock<std::mutex> lock(scan_mutex_);
    const bool received_scan = scan_cv_.wait_for(
      lock, timeout,
      [this, baseline_scan_count, generation]() {
        return scan_count_ > baseline_scan_count ||
               !should_continue(generation, DesiredState::Active);
      });

    if (!should_continue(generation, DesiredState::Active)) {
      return false;
    }

    if (!received_scan || scan_count_ <= baseline_scan_count) {
      RCLCPP_ERROR(
        get_logger(),
        "No fresh scan received on %s within %.3f seconds after starting lidar motor; resuming consumers anyway",
        scan_topic_.c_str(), scan_timeout_sec_);
    }
    return true;
  }

  double idle_timeout_sec_{10.0};
  double scan_timeout_sec_{3.0};
  std::string start_motor_service_;
  std::string stop_motor_service_;
  std::string pause_rtabmap_service_;
  std::string resume_rtabmap_service_;
  std::string pause_odom_service_;
  std::string resume_odom_service_;
  std::string scan_topic_;
  bool manage_consumer_log_levels_{true};
  std::string idle_consumer_log_level_;
  std::string active_consumer_log_level_;
  std::string rtabmap_logger_service_;
  std::string icp_odom_logger_service_;
  std::string rtabmap_logger_name_;
  std::string icp_odom_logger_name_;

  EmptyClient::SharedPtr start_motor_client_;
  EmptyClient::SharedPtr stop_motor_client_;
  EmptyClient::SharedPtr pause_rtabmap_client_;
  EmptyClient::SharedPtr resume_rtabmap_client_;
  EmptyClient::SharedPtr pause_odom_client_;
  EmptyClient::SharedPtr resume_odom_client_;
  LoggerLevelsClient::SharedPtr rtabmap_logger_client_;
  LoggerLevelsClient::SharedPtr icp_odom_logger_client_;

  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigate_to_pose_status_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigate_through_poses_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr standby_timer_;

  std::mutex state_mutex_;
  bool navigate_to_pose_active_{false};
  bool navigate_through_poses_active_{false};
  bool navigation_active_{false};
  bool shutting_down_{false};
  std::uint64_t state_generation_{0};

  std::mutex operation_mutex_;
  std::mutex workers_mutex_;
  std::vector<std::thread> workers_;

  std::mutex scan_mutex_;
  std::condition_variable scan_cv_;
  std::uint64_t scan_count_{0};
};

}  // namespace diffbot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<diffbot::LidarStandbyManager>());
  rclcpp::shutdown();
  return 0;
}
