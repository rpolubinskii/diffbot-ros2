// icp_odom_reweighter
//
// PURPOSE. icp_odometry (rtabmap) reports a DISHONEST yaw covariance: measured
// on this robot (bags diffbot_latest_2 / _3) the yaw variance is hardwired to
// EXACTLY 0.1x the x variance (one scalar quality number on a fixed diagonal),
// ~1.5e-5 == claiming +/-0.22 deg heading confidence even mid-spin. icp is also
// BLIND to its own rotational error: its quality signals (structural_complexity,
// inlier ratio) do NOT degrade during spins -- on self-similar walls it finds
// good correspondences at a slightly-WRONG rotation and reports high confidence.
// Because the EKF pins absolute yaw to icp (ekf.yaml odom1 idx5) and trusts that
// tiny covariance, it follows icp's wrong heading through spins -> the live map
// feature "gets misplaced when rotating".
//
// We proved the cheap fixes can't help: Icp/PointToPlane didn't change the fixed
// 0.1 ratio, and EKF-side reweighting/rejection can't loosen the pin while icp
// reports ~1e-5. The ONLY honest signal for "is icp's yaw wrong right now" is an
// INDEPENDENT rotation reference, and the RealSense gyro (~198 Hz, true body yaw
// rate, scan-geometry-independent) is exactly that.
//
// WHAT THIS NODE DOES. Sits between icp_odometry and the EKF. Republishes
// icp_odom unchanged EXCEPT the yaw covariance, which it INFLATES in proportion
// to how much icp's rotation rate disagrees with the gyro's over the same window:
//   - agree (disagreement < deadband): pass icp's covariance through untouched
//     -> EKF still pins heading to icp -> drift corrected (stable behavior is
//     IDENTICAL to baseline; this node does nothing when not spinning).
//   - disagree (a spin icp is mis-measuring): yaw variance balloons -> EKF
//     down-weights icp's absolute yaw and lets the gyro rate (imu0) carry heading
//     through the spin; when agreement returns, icp re-pins and corrects any gyro
//     drift accumulated during the spin.
// icp's x/y/translation covariance is left untouched (icp translation is good).
//
// This is a complementary filter implemented as honest covariance: gyro leads
// rotation transients, icp anchors absolute heading. The EKF does the blending;
// this node only supplies the missing uncertainty estimate icp cannot produce.

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace diffbot
{

// Index of the yaw (and yaw-rate) diagonal term in a row-major 6x6 covariance:
// order is [x, y, z, roll, pitch, yaw] -> yaw is row/col 5 -> 5*6 + 5 = 35.
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

    // KEY TUNING KNOB. Added yaw STANDARD DEVIATION (rad) per (rad/s) of rate
    // disagreement above the deadband; output variance = input + (gain*excess)^2.
    // Example: a 0.4 rad/s spin mismatch with gain 2.0 -> +0.8 rad std -> variance
    // ~0.64, swamping icp's ~1.5e-5 so the EKF effectively ignores icp yaw for
    // that frame. Too high: tiny mismatches kill icp's drift anchor. Too low: icp
    // keeps dragging heading during spins. Raise if heading still follows icp
    // through spins; lower if the map drifts (icp no longer correcting).
    yaw_disagreement_gain_ = declare_parameter<double>("yaw_disagreement_gain", 2.0);
    // Disagreement below this (rad/s) is treated as sensor noise -> no inflation,
    // so straight/stable driving is untouched. ~0.1 rad/s ~= 5.7 deg/s.
    disagreement_deadband_ = declare_parameter<double>("disagreement_deadband", 0.1);
    // Floor applied to the yaw variance even when icp & gyro agree. 0.0 = pass
    // icp's reported value straight through (default; preserves baseline pin).
    min_yaw_variance_ = declare_parameter<double>("min_yaw_variance", 0.0);
    // Clamp so a transient never produces a degenerate covariance. 1.0 rad^2 std
    // ~= 57 deg = "ignore icp yaw entirely" -- the intended saturation.
    max_yaw_variance_ = declare_parameter<double>("max_yaw_variance", 1.0);
    // Also inflate the twist (vyaw RATE) covariance on disagreement, so icp's rate
    // is down-weighted too if the EKF fuses icp vyaw (ekf.yaml odom1 idx11).
    reweight_twist_ = declare_parameter<bool>("reweight_twist", true);
    // If no gyro sample arrives within this many seconds, we cannot judge icp ->
    // pass through UNMODIFIED (fall back to trusting icp) and warn.
    gyro_timeout_sec_ = declare_parameter<double>("gyro_timeout_sec", 0.5);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      output_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    // Best-effort sub accepts a reliable publisher; robust for sensor streams.
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
  // Accumulate the gyro yaw rate between icp messages so we compare icp's
  // per-frame rotation against the gyro's MEAN over the same window (not a single
  // sample). Single-threaded executor -> callbacks are serialized, no mutex.
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
      // No trustworthy reference -> don't fabricate uncertainty; pass through.
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

}  // namespace diffbot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<diffbot::IcpOdomReweighter>());
  rclcpp::shutdown();
  return 0;
}
