#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace diffbot
{

constexpr std::size_t kYawCovIndex = 35;

class IcpOdomReweighter : public rclcpp::Node
{
public:
  IcpOdomReweighter()
  : Node("diffbot_icp_odom_reweighter")
  {
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/rtabmap/icp_odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data_body");
    output_odom_topic_ =
      declare_parameter<std::string>("output_odom_topic", "/rtabmap/icp_odom_reweighted");

    yaw_disagreement_gain_ = declare_parameter<double>("yaw_disagreement_gain", 2.0);
    disagreement_deadband_ = declare_parameter<double>("disagreement_deadband", 0.1);
    min_yaw_variance_ = declare_parameter<double>("min_yaw_variance", 0.0);
    max_yaw_variance_ = declare_parameter<double>("max_yaw_variance", 1.0);
    reweight_twist_ = declare_parameter<bool>("reweight_twist", true);
    gyro_timeout_sec_ = declare_parameter<double>("gyro_timeout_sec", 0.5);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      output_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(msg); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odom(msg); });

    RCLCPP_INFO(
      get_logger(),
      "icp_odom_reweighter: %s + %s -> %s | gain=%.3f deadband=%.3f rad/s "
      "min_var=%.3g max_var=%.3g reweight_twist=%s",
      input_odom_topic_.c_str(), imu_topic_.c_str(), output_odom_topic_.c_str(),
      yaw_disagreement_gain_, disagreement_deadband_, min_yaw_variance_, max_yaw_variance_,
      reweight_twist_ ? "true" : "false");
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
  {
    gyro_wz_sum_ += msg->angular_velocity.z;
    ++gyro_sample_count_;
    last_gyro_time_ = now();
    have_gyro_ = true;
  }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    nav_msgs::msg::Odometry out = *msg;

    const bool gyro_fresh =
      have_gyro_ && (now() - last_gyro_time_).seconds() <= gyro_timeout_sec_;

    if (!gyro_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No fresh gyro on %s within %.3f s; passing icp_odom through unmodified",
        imu_topic_.c_str(), gyro_timeout_sec_);
      odom_pub_->publish(out);
      reset_gyro_window();
      return;
    }

    const double gyro_wz =
      gyro_sample_count_ > 0 ? (gyro_wz_sum_ / static_cast<double>(gyro_sample_count_))
                             : last_gyro_wz_;
    last_gyro_wz_ = gyro_wz;
    reset_gyro_window();

    const double icp_wz = msg->twist.twist.angular.z;
    const double disagreement = std::abs(gyro_wz - icp_wz);
    const double excess = std::max(0.0, disagreement - disagreement_deadband_);

    const double base_var = std::max(out.pose.covariance[kYawCovIndex], min_yaw_variance_);
    const double extra_std = yaw_disagreement_gain_ * excess;
    double yaw_var = base_var + extra_std * extra_std;
    yaw_var = std::min(yaw_var, max_yaw_variance_);

    out.pose.covariance[kYawCovIndex] = yaw_var;
    if (reweight_twist_) {
      const double base_twist_var =
        std::max(out.twist.covariance[kYawCovIndex], min_yaw_variance_);
      out.twist.covariance[kYawCovIndex] =
        std::min(base_twist_var + extra_std * extra_std, max_yaw_variance_);
    }

    if (excess > 0.0) {
      RCLCPP_DEBUG(
        get_logger(),
        "spin disagreement: gyro=%.3f icp=%.3f rad/s -> yaw_var %.3g -> %.3g",
        gyro_wz, icp_wz, base_var, yaw_var);
    }

    odom_pub_->publish(out);
  }

  void reset_gyro_window()
  {
    gyro_wz_sum_ = 0.0;
    gyro_sample_count_ = 0;
  }

  std::string input_odom_topic_;
  std::string imu_topic_;
  std::string output_odom_topic_;
  double yaw_disagreement_gain_{2.0};
  double disagreement_deadband_{0.1};
  double min_yaw_variance_{0.0};
  double max_yaw_variance_{1.0};
  bool reweight_twist_{true};
  double gyro_timeout_sec_{0.5};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  double gyro_wz_sum_{0.0};
  std::uint64_t gyro_sample_count_{0};
  double last_gyro_wz_{0.0};
  bool have_gyro_{false};
  rclcpp::Time last_gyro_time_{0, 0, RCL_ROS_TIME};
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<diffbot::IcpOdomReweighter>());
  rclcpp::shutdown();
  return 0;
}
