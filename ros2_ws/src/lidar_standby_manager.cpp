#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
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
    initial_idle_timeout_sec_ = declare_parameter<double>("initial_idle_timeout_sec", 25.0);
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 3.0);
    start_motor_service_ = declare_parameter<std::string>("start_motor_service", "/start_motor");
    stop_motor_service_ = declare_parameter<std::string>("stop_motor_service", "/stop_motor");
    pause_rtabmap_service_ = declare_parameter<std::string>("pause_rtabmap_service", "/rtabmap/pause");
    resume_rtabmap_service_ = declare_parameter<std::string>("resume_rtabmap_service", "/rtabmap/resume");
    pause_odom_service_ = declare_parameter<std::string>("pause_odom_service", "/pause_odom");
    resume_odom_service_ = declare_parameter<std::string>("resume_odom_service", "/resume_odom");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    managed_scan_topic_ = declare_parameter<std::string>("managed_scan_topic", "/diffbot/standby_scan");
    publish_standby_scan_heartbeat_ = declare_parameter<bool>("publish_standby_scan_heartbeat", true);
    standby_scan_heartbeat_hz_ = declare_parameter<double>("standby_scan_heartbeat_hz", 1.0);
    nav_action_status_topics_ = declare_parameter<std::vector<std::string>>(
      "nav_action_status_topics",
      {
        "/navigate_to_pose/_action/status",
        "/navigate_through_poses/_action/status",
        "/spin/_action/status"
      });
    nav_action_status_topics_.erase(
      std::remove_if(
        nav_action_status_topics_.begin(), nav_action_status_topics_.end(),
        [](const auto & topic) {
          return topic.empty();
        }),
      nav_action_status_topics_.end());
    if (nav_action_status_topics_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "nav_action_status_topics is empty; falling back to /navigate_to_pose/_action/status");
      nav_action_status_topics_.push_back("/navigate_to_pose/_action/status");
    }

    start_motor_client_ = create_client<std_srvs::srv::Empty>(start_motor_service_);
    stop_motor_client_ = create_client<std_srvs::srv::Empty>(stop_motor_service_);
    pause_rtabmap_client_ = create_client<std_srvs::srv::Empty>(pause_rtabmap_service_);
    resume_rtabmap_client_ = create_client<std_srvs::srv::Empty>(resume_rtabmap_service_);
    pause_odom_client_ = create_client<std_srvs::srv::Empty>(pause_odom_service_);
    resume_odom_client_ = create_client<std_srvs::srv::Empty>(resume_odom_service_);

    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    for (const auto & status_topic : nav_action_status_topics_) {
      nav_action_status_active_[status_topic] = false;
      nav_action_status_subs_.push_back(create_subscription<action_msgs::msg::GoalStatusArray>(
        status_topic, status_qos,
        [this, status_topic](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
          handle_status(msg, status_topic);
        }));
    }

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        if (standby_scan_heartbeat_active()) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(scan_mutex_);
          last_scan_ = *msg;
          has_last_scan_ = true;
          ++scan_count_;
        }
        scan_heartbeat_pub_->publish(*msg);
        scan_cv_.notify_all();
      });
    scan_heartbeat_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      managed_scan_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    schedule_standby_timer(true);

    RCLCPP_INFO(
      get_logger(),
      "Lidar standby manager started: initial_idle_timeout_sec=%.3f, idle_timeout_sec=%.3f, scan_timeout_sec=%.3f, scan_topic=%s, managed_scan_topic=%s",
      initial_idle_timeout_sec_, idle_timeout_sec_, scan_timeout_sec_, scan_topic_.c_str(),
      managed_scan_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "Watching Nav2 action status topics: %s",
      join_strings(nav_action_status_topics_).c_str());
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
      if (scan_heartbeat_timer_) {
        scan_heartbeat_timer_->cancel();
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
  enum class DesiredState
  {
    Active,
    Idle
  };

  using EmptyClient = rclcpp::Client<std_srvs::srv::Empty>;

  void handle_status(
    const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg,
    const std::string & status_topic)
  {
    const bool source_active = has_active_goal(*msg);

    bool became_active = false;
    bool became_idle = false;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      nav_action_status_active_[status_topic] = source_active;

      const bool new_navigation_active = std::any_of(
        nav_action_status_active_.begin(), nav_action_status_active_.end(),
        [](const auto & entry) {
          return entry.second;
        });
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
      if (became_active) {
        scan_heartbeat_active_ = false;
        if (scan_heartbeat_timer_) {
          scan_heartbeat_timer_->cancel();
        }
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
      schedule_standby_timer(false);
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

  static std::string join_strings(const std::vector<std::string> & values)
  {
    std::string joined;
    for (const auto & value : values) {
      if (!joined.empty()) {
        joined += ", ";
      }
      joined += value;
    }
    return joined;
  }

  void schedule_standby_timer(bool initial_delay)
  {
    const auto generation = current_generation();
    const auto timeout_sec = initial_delay ? initial_idle_timeout_sec_ : idle_timeout_sec_;
    const auto delay = std::chrono::duration<double>(std::max(0.0, timeout_sec));
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

    stop_standby_scan_heartbeat();

    const auto baseline_scan_count = observed_scan_count();
    if (!call_empty_service(start_motor_client_, start_motor_service_, generation, DesiredState::Active)) {
      return;
    }

    if (!wait_for_fresh_scan(baseline_scan_count, generation)) {
      return;
    }

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
    if (call_empty_service(stop_motor_client_, stop_motor_service_, generation, DesiredState::Idle)) {
      start_standby_scan_heartbeat(generation);
    }
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

  bool standby_scan_heartbeat_active()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return scan_heartbeat_active_;
  }

  void start_standby_scan_heartbeat(std::uint64_t generation)
  {
    if (!publish_standby_scan_heartbeat_ || standby_scan_heartbeat_hz_ <= 0.0) {
      return;
    }
    if (!should_continue(generation, DesiredState::Idle)) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      scan_heartbeat_active_ = true;
    }

    const auto period = std::chrono::duration<double>(1.0 / standby_scan_heartbeat_hz_);
    scan_heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this, generation]() {
        publish_standby_scan_heartbeat(generation);
      });

    RCLCPP_INFO(
      get_logger(), "Publishing standby scan heartbeat on %s at %.3f Hz",
      managed_scan_topic_.c_str(), standby_scan_heartbeat_hz_);
  }

  void stop_standby_scan_heartbeat()
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      scan_heartbeat_active_ = false;
      if (scan_heartbeat_timer_) {
        scan_heartbeat_timer_->cancel();
      }
    }
  }

  void publish_standby_scan_heartbeat(std::uint64_t generation)
  {
    if (!should_continue(generation, DesiredState::Idle)) {
      stop_standby_scan_heartbeat();
      return;
    }

    sensor_msgs::msg::LaserScan heartbeat;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (!has_last_scan_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Cannot publish standby scan heartbeat yet: no real scan has been observed on %s",
          scan_topic_.c_str());
        return;
      }
      heartbeat = last_scan_;
    }

    heartbeat.header.stamp = now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (
        shutting_down_ || generation != state_generation_ || navigation_active_ ||
        !scan_heartbeat_active_)
      {
        if (scan_heartbeat_timer_) {
          scan_heartbeat_timer_->cancel();
        }
        scan_heartbeat_active_ = false;
        return;
      }
      scan_heartbeat_pub_->publish(heartbeat);
    }
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
  double initial_idle_timeout_sec_{25.0};
  double scan_timeout_sec_{3.0};
  std::string start_motor_service_;
  std::string stop_motor_service_;
  std::string pause_rtabmap_service_;
  std::string resume_rtabmap_service_;
  std::string pause_odom_service_;
  std::string resume_odom_service_;
  std::string scan_topic_;
  std::string managed_scan_topic_;
  std::vector<std::string> nav_action_status_topics_;
  bool publish_standby_scan_heartbeat_{true};
  double standby_scan_heartbeat_hz_{1.0};

  EmptyClient::SharedPtr start_motor_client_;
  EmptyClient::SharedPtr stop_motor_client_;
  EmptyClient::SharedPtr pause_rtabmap_client_;
  EmptyClient::SharedPtr resume_rtabmap_client_;
  EmptyClient::SharedPtr pause_odom_client_;
  EmptyClient::SharedPtr resume_odom_client_;

  std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr>
    nav_action_status_subs_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr standby_timer_;
  rclcpp::TimerBase::SharedPtr scan_heartbeat_timer_;

  std::mutex state_mutex_;
  std::unordered_map<std::string, bool> nav_action_status_active_;
  bool navigation_active_{false};
  bool shutting_down_{false};
  bool scan_heartbeat_active_{false};
  std::uint64_t state_generation_{0};

  std::mutex operation_mutex_;
  std::mutex workers_mutex_;
  std::vector<std::thread> workers_;

  std::mutex scan_mutex_;
  std::condition_variable scan_cv_;
  std::uint64_t scan_count_{0};
  bool has_last_scan_{false};
  sensor_msgs::msg::LaserScan last_scan_;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<diffbot::LidarStandbyManager>());
  rclcpp::shutdown();
  return 0;
}
