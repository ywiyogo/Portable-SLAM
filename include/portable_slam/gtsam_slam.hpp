/**
 * @brief GTSAM factor graph SLAM node for handheld portable SLAM.
 *        Phase 2a: IMU preintegration + rf2o odometry + barometric altitude.
 * @author Yongkie Wiyogo
 */

#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationCombinedParams.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <gtsam/navigation/ImuBias.h>

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

class GtsamSlamNode : public rclcpp::Node {
 public:
  GtsamSlamNode();
  ~GtsamSlamNode();

 private:
  /// Integrate raw IMU measurement into preintegrated buffer (called per /imu/data_raw msg)
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /// Check keyframe threshold, trigger new keyframe if needed, publish optimized odometry
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// Convert barometric pressure to altitude and store for next keyframe's Z constraint
  void pressureCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);

  /// Add a new keyframe: CombinedImuFactor + BetweenFactor<Pose3> + altitude PriorFactor
  void addNewKeyframe(const nav_msgs::msg::Odometry::SharedPtr& odom_msg);

  /// Publish nav_msgs/Odometry and TF odom→base_link from a GTSAM pose/velocity
  void publishOdometry(const gtsam::Pose3& pose, const gtsam::Velocity3& vel,
                       const rclcpp::Time& stamp);

  /// Return true if translation or rotation since last keyframe exceeds thresholds
  bool isKeyframeNeeded(const gtsam::Pose3& current_pose) const;

  /// Declare all ROS2 parameters with defaults (called before loadParameters)
  void declareParameters();

  /// Load all declared parameters into member variables
  void loadParameters();

  /// Create odometry publisher and TF broadcaster
  void initPublishers();

  /// Create subscribers for /imu/data_raw, /odom_rf2o, /pressure (single MutuallyExclusive group)
  void initSubscribers();

  /// Initialize GTSAM: build IMU preintegration params, create first keyframe priors, seed ISAM2
  void initGtsam();

  /// Build ISAM2Params from loaded YAML parameters
  gtsam::ISAM2Params makeIsam2Params() const;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<gtsam::PreintegrationCombinedParams> imu_params_;
  std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> pim_;
  std::unique_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values new_values_;

  std::optional<gtsam::Pose3> last_keyframe_pose_;
  std::optional<rclcpp::Time> last_imu_stamp_;
  gtsam::imuBias::ConstantBias current_bias_;
  size_t keyframe_index_{0};

  double latest_altitude_{0.0};
  bool altitude_received_{false};

  // Parameters loaded from YAML
  double keyframe_trans_thresh_{0.3};
  double keyframe_rot_thresh_{0.26};
  double accel_noise_sigma_{0.4905};
  double gyro_noise_sigma_{0.01745};
  double accel_bias_rw_sigma_{0.001};
  double gyro_bias_rw_sigma_{0.0001};
  double integration_cov_sigma_{1e-8};
  double bias_acc_cov_sigma_{0.001};
  double bias_gyro_cov_sigma_{0.0001};
  double bias_init_sigma_{1e-3};
  gtsam::Vector3 n_gravity_{0.0, 0.0, -9.81};
  gtsam::Vector6 odom_noise_sigmas_;
  double altitude_sigma_{1.0};
  double isam2_relinearize_thresh_{0.1};
  double isam2_relinearize_skip_{1.0};
  double prior_pose_sigma_{0.01};
  double prior_vel_sigma_{0.1};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string imu_frame_{"imu_link"};
  std::string imu_topic_{"imu/data_raw"};
  std::string odom_topic_{"odom_rf2o"};
  std::string pressure_topic_{"pressure"};
  std::string output_odom_topic_{"odometry/filtered"};
};