/**
 * @file gtsam_slam_node.cpp
 * @brief ROS2 node implementing Phase 2a of the GTSAM factor graph SLAM pipeline.
 *
 * This node replaces the Madgwick filter + robot_localization EKF with a single
 * GTSAM ISAM2 factor graph. It subscribes to raw IMU, rf2o laser odometry, and
 * barometric pressure, then publishes optimized odometry and TF transforms.
 *
 * Factors implemented (Phase 2a):
 *   - CombinedImuFactor: IMU preintegration between keyframes (replaces Madgwick + EKF)
 *   - BetweenFactor<Pose3>: rf2o scan-matching odometry (high Z/roll/pitch uncertainty)
 *   - PriorFactor<Pose3>: barometric altitude Z constraint from LPS22HB
 *
 * GTSAM state per keyframe: Pose3 (X), Velocity3 (V), imuBias::ConstantBias (B)
 *
 * Keyframe selection: triggered when accumulated motion from rf2o exceeds
 * translation or rotation thresholds. Between keyframes, IMU data is
 * preintegrated. Inter-keyframe odometry is produced via IMU prediction
 * from the latest optimized keyframe state.
 *
 * Future phases will add: gravity/magnetometer orientation factors (2b),
 * occupancy grid map generation (2c), and loop closure with ScanContext (2d).
 *
 * @author Yongkie Wiyogo
 */

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

#include <gtsam/imuBias/ConstantBias.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "portable_slam/gtsam_slam.hpp"

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

GtsamSlamNode::GtsamSlamNode()
    : Node("gtsam_slam_node"),
      isam2_(makeIsam2Params()),
      odom_noise_sigmas_(
          (gtsam::Vector(6) << 0.1, 0.1, 0.01, 0.05, 0.05, 10.0)
              .finished()) {
  declareParameters();
  loadParameters();
  initGtsam();
  initPublishers();
  initSubscribers();

  RCLCPP_INFO(this->get_logger(),
              "GTSAM SLAM node initialized (Phase 2a: IMU + odometry + "
              "altitude, no loop closure)");
}

GtsamSlamNode::~GtsamSlamNode() = default;

void GtsamSlamNode::declareParameters() {
  this->declare_parameter("keyframe_trans_thresh", 0.3);
  this->declare_parameter("keyframe_rot_thresh", 0.26);
  this->declare_parameter("accel_noise_sigma", 0.4905);
  this->declare_parameter("gyro_noise_sigma", 0.01745);
  this->declare_parameter("accel_bias_rw_sigma", 0.001);
  this->declare_parameter("gyro_bias_rw_sigma", 0.0001);
  this->declare_parameter("integration_cov_sigma", 1e-8);
  this->declare_parameter("bias_acc_cov_sigma", 0.001);
  this->declare_parameter("bias_gyro_cov_sigma", 0.0001);
  this->declare_parameter("bias_init_sigma", 1e-3);

  std::vector<double> default_gravity = {0.0, 0.0, -9.81};
  this->declare_parameter("gravity", default_gravity);

  std::vector<double> default_odom_sigmas = {0.1, 0.1, 0.01, 0.05, 0.05, 10.0};
  this->declare_parameter("odom_noise_sigmas", default_odom_sigmas);

  this->declare_parameter("altitude_sigma", 1.0);
  this->declare_parameter("isam2_relinearize_thresh", 0.1);
  this->declare_parameter("isam2_relinearize_skip", 1.0);
  this->declare_parameter("prior_pose_sigma", 0.01);
  this->declare_parameter("prior_vel_sigma", 0.1);

  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("imu_frame", "imu_link");
  this->declare_parameter("imu_topic", "/imu/data_raw");
  this->declare_parameter("odom_topic", "/odom_rf2o");
  this->declare_parameter("pressure_topic", "/pressure");
  this->declare_parameter("output_odom_topic", "/odometry/filtered");
}

void GtsamSlamNode::loadParameters() {
  keyframe_trans_thresh_ = this->get_parameter("keyframe_trans_thresh").as_double();
  keyframe_rot_thresh_ = this->get_parameter("keyframe_rot_thresh").as_double();
  accel_noise_sigma = this->get_parameter("accel_noise_sigma").as_double();
  gyro_noise_sigma = this->get_parameter("gyro_noise_sigma").as_double();
  accel_bias_rw_sigma = this->get_parameter("accel_bias_rw_sigma").as_double();
  gyro_bias_rw_sigma = this->get_parameter("gyro_bias_rw_sigma").as_double();
  integration_cov_sigma = this->get_parameter("integration_cov_sigma").as_double();
  bias_acc_cov_sigma = this->get_parameter("bias_acc_cov_sigma").as_double();
  bias_gyro_cov_sigma = this->get_parameter("bias_gyro_cov_sigma").as_double();
  bias_init_sigma = this->get_parameter("bias_init_sigma").as_double();

  auto grav = this->get_parameter("gravity").as_double_array();
  n_gravity_ = gtsam::Vector3(grav[0], grav[1], grav[2]);

  auto sigmas = this->get_parameter("odom_noise_sigmas").as_double_array();
  odom_noise_sigmas_ =
      (gtsam::Vector(6) << sigmas[0], sigmas[1], sigmas[2], sigmas[3],
       sigmas[4], sigmas[5])
          .finished();

  altitude_sigma = this->get_parameter("altitude_sigma").as_double();
  isam2_relinearize_thresh_ =
      this->get_parameter("isam2_relinearize_thresh").as_double();
  isam2_relinearize_skip_ =
      this->get_parameter("isam2_relinearize_skip").as_double();
  prior_pose_sigma = this->get_parameter("prior_pose_sigma").as_double();
  prior_vel_sigma = this->get_parameter("prior_vel_sigma").as_double();

  odom_frame_ = this->get_parameter("odom_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  imu_frame_ = this->get_parameter("imu_frame").as_string();
  imu_topic = this->get_parameter("imu_topic").as_string();
  odom_topic = this->get_parameter("odom_topic").as_string();
  pressure_topic = this->get_parameter("pressure_topic").as_string();
  output_odom_topic = this->get_parameter("output_odom_topic").as_string();
}

void GtsamSlamNode::initPublishers() {
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      output_odom_topic, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void GtsamSlamNode::initSubscribers() {
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&GtsamSlamNode::imuCallback, this, std::placeholders::_1),
      sub_opts);

  auto odom_sub_opts = sub_opts;
  odom_sub_opts.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&GtsamSlamNode::odomCallback, this, std::placeholders::_1),
      odom_sub_opts);

  auto pressure_sub_opts = sub_opts;
  pressure_sub_opts.callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      pressure_topic, rclcpp::SensorDataQoS(),
      std::bind(&GtsamSlamNode::pressureCallback, this,
                std::placeholders::_1),
      pressure_sub_opts);
}

void GtsamSlamNode::initGtsam() {
  auto imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(
      n_gravity_.norm());

  imu_params->accelerometerCovariance =
      std::pow(accel_noise_sigma, 2) * gtsam::I_3x3;
  imu_params->gyroscopeCovariance =
      std::pow(gyro_noise_sigma, 2) * gtsam::I_3x3;
  imu_params->integrationCovariance =
      integration_cov_sigma * gtsam::I_3x3;
  imu_params->biasAccCovariance =
      std::pow(bias_acc_cov_sigma, 2) * gtsam::I_3x3;
  imu_params->biasOmegaCovariance =
      std::pow(bias_gyro_cov_sigma, 2) * gtsam::I_3x3;
  imu_params->biasAccOmegaInit =
      std::pow(bias_init_sigma, 2) * gtsam::Matrix6::Identity();

  imu_params_ = imu_params;

  current_bias_ = gtsam::imuBias::ConstantBias();
  pim_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(
      imu_params_, current_bias_);

  gtsam::Pose3 prior_pose = gtsam::Pose3();
  gtsam::Vector3 prior_vel = gtsam::Vector3::Zero();

  auto prior_pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << prior_pose_sigma, prior_pose_sigma,
       prior_pose_sigma, prior_pose_sigma, prior_pose_sigma, prior_pose_sigma)
          .finished());
  auto prior_vel_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << prior_vel_sigma, prior_vel_sigma, prior_vel_sigma)
          .finished());
  auto prior_bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << bias_init_sigma, bias_init_sigma, bias_init_sigma,
       bias_init_sigma, bias_init_sigma, bias_init_sigma)
          .finished());

  new_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(keyframe_index_), prior_pose, prior_pose_noise));
  new_factors_.add(gtsam::PriorFactor<gtsam::Vector3>(
      V(keyframe_index_), prior_vel, prior_vel_noise));
  new_factors_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      B(keyframe_index_), current_bias_, prior_bias_noise));

  new_values_.insert(X(keyframe_index_), prior_pose);
  new_values_.insert(V(keyframe_index_), prior_vel);
  new_values_.insert(B(keyframe_index_), current_bias_);

  isam2_.update(new_factors_, new_values_);
  new_factors_ = gtsam::NonlinearFactorGraph();
  new_values_.clear();

  last_keyframe_pose_ = prior_pose;

  RCLCPP_INFO(this->get_logger(),
              "GTSAM initialized with first keyframe %zu, gravity=%.2f m/s^2",
              keyframe_index_, n_gravity_.norm());
}

gtsam::ISAM2Params GtsamSlamNode::makeIsam2Params() const {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = isam2_relinearize_thresh_;
  params.relinearizeSkip = static_cast<int>(isam2_relinearize_skip_);
  params.factorization = gtsam::ISAM2Params::CHOLESKY;
  params.enableDetailedResults = false;
  return params;
}

void GtsamSlamNode::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!last_imu_stamp_.has_value()) {
    last_imu_stamp_ = rclcpp::Time(msg->header.stamp);
    return;
  }

  double dt = (rclcpp::Time(msg->header.stamp) - last_imu_stamp_.value())
                   .seconds();
  last_imu_stamp_ = rclcpp::Time(msg->header.stamp);

  if (dt <= 0.0 || dt > 0.5) {
    RCLCPP_WARN(this->get_logger(),
                "IMU dt out of range: %.4f s, skipping integration", dt);
    return;
  }

  gtsam::Vector3 accel(msg->linear_acceleration.x,
                       msg->linear_acceleration.y,
                       msg->linear_acceleration.z);
  gtsam::Vector3 gyro(msg->angular_velocity.x, msg->angular_velocity.y,
                      msg->angular_velocity.z);

  pim_->integrateMeasurement(accel, gyro, dt);
}

void GtsamSlamNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto& p = msg->pose.pose.position;
  const auto& q = msg->pose.pose.orientation;

  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

  gtsam::Rot3 rot = gtsam::Rot3::RzRyRx(roll, pitch, yaw);
  gtsam::Point3 pos(p.x, p.y, p.z);
  gtsam::Pose3 current_pose(rot, pos);

  if (isKeyframeNeeded(current_pose)) {
    addNewKeyframe(msg);
  }

  rclcpp::Time stamp(msg->header.stamp);
  gtsam::Values optimized = isam2_.calculateEstimate();
  if (optimized.exists(X(keyframe_index_)) && optimized.exists(V(keyframe_index_))) {
    auto kf_pose = optimized.at<gtsam::Pose3>(X(keyframe_index_));
    auto kf_vel = optimized.at<gtsam::Vector3>(V(keyframe_index_));

    if (pim_->deltaTij() > 0.0) {
      try {
        auto predicted = pim_->predict(gtsam::NavState(kf_pose, kf_vel),
                                       current_bias_);
        publishOdometry(predicted.pose(), predicted.velocity(), stamp);
        return;
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
                     "IMU prediction failed: %s, using keyframe pose", e.what());
      }
    }
    publishOdometry(kf_pose, kf_vel, stamp);
  }
}

void GtsamSlamNode::pressureCallback(
    const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  double pressure_pa = msg->fluid_pressure;
  double pressure_hpa = pressure_pa / 100.0;
  latest_altitude_ = 44330.0 * (1.0 - std::pow(pressure_hpa / 1013.25, 0.1903));
  altitude_received_ = true;
}

void GtsamSlamNode::addNewKeyframe(
    const nav_msgs::msg::Odometry::SharedPtr& odom_msg) {
  size_t prev_idx = keyframe_index_;
  keyframe_index_++;

  const auto& p = odom_msg->pose.pose.position;
  const auto& q = odom_msg->pose.pose.orientation;
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

  gtsam::Pose3 current_pose(gtsam::Rot3::RzRyRx(roll, pitch, yaw),
                            gtsam::Point3(p.x, p.y, p.z));

  gtsam::Pose3 delta = gtsam::Pose3();
  if (last_keyframe_pose_.has_value()) {
    delta = last_keyframe_pose_.value().between(current_pose);
  }

  auto odom_noise =
      gtsam::noiseModel::Diagonal::Sigmas(odom_noise_sigmas_);
  new_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(prev_idx), X(keyframe_index_), delta, odom_noise));

  new_factors_.add(gtsam::CombinedImuFactor(
      X(prev_idx), V(prev_idx), X(keyframe_index_), V(keyframe_index_),
      B(prev_idx), B(keyframe_index_), *pim_));

  if (altitude_received_) {
    gtsam::Pose3 alt_pose(gtsam::Rot3(),
                          gtsam::Point3(0.0, 0.0, latest_altitude_));
    auto alt_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 10.0, 10.0, 10.0, 10.0, 10.0, altitude_sigma)
            .finished());
    new_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(
        X(keyframe_index_), alt_pose, alt_noise));
  }

  gtsam::NavState prev_state;
  gtsam::Values optimized = isam2_.calculateEstimate();
  if (optimized.exists(X(prev_idx))) {
    auto prev_pose = optimized.at<gtsam::Pose3>(X(prev_idx));
    auto prev_vel = optimized.at<gtsam::Vector3>(V(prev_idx));
    prev_state = gtsam::NavState(prev_pose, prev_vel);
  }

  auto predicted_state = pim_->predict(prev_state, current_bias_);
  new_values_.insert(X(keyframe_index_), predicted_state.pose());
  new_values_.insert(V(keyframe_index_), predicted_state.velocity());
  new_values_.insert(B(keyframe_index_), current_bias_);

  isam2_.update(new_factors_, new_values_);
  isam2_.update();

  new_factors_ = gtsam::NonlinearFactorGraph();
  new_values_.clear();

  double pim_dt = pim_->deltaTij();
  pim_->resetIntegrationAndSetBias(current_bias_);

  auto updated_bias = isam2_.calculateEstimate()
                          .at<gtsam::imuBias::ConstantBias>(
                              B(keyframe_index_));
  current_bias_ = updated_bias;

  last_keyframe_pose_ = current_pose;

  RCLCPP_INFO(this->get_logger(),
              "Keyframe %zu added (prev=%zu, IMU dt=%.3f s, alt=%.2f m)",
              keyframe_index_, prev_idx, pim_dt,
              altitude_received_ ? latest_altitude_ : -1.0);
}

void GtsamSlamNode::publishOdometry(const gtsam::Pose3& pose,
                                    const gtsam::Velocity3& vel,
                                    const rclcpp::Time& stamp) {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  const auto& t = pose.translation();
  const auto& r = pose.rotation().quaternion();
  odom_msg.pose.pose.position.x = t.x();
  odom_msg.pose.pose.position.y = t.y();
  odom_msg.pose.pose.position.z = t.z();
  odom_msg.pose.pose.orientation.w = r(0);
  odom_msg.pose.pose.orientation.x = r(1);
  odom_msg.pose.pose.orientation.y = r(2);
  odom_msg.pose.pose.orientation.z = r(3);

  odom_msg.twist.twist.linear.x = vel(0);
  odom_msg.twist.twist.linear.y = vel(1);
  odom_msg.twist.twist.linear.z = vel(2);

  for (int i = 0; i < 36; ++i) {
    odom_msg.pose.covariance[i] = 0.0;
    odom_msg.twist.covariance[i] = 0.0;
  }

  odom_pub_->publish(odom_msg);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;
  tf.transform.translation.x = t.x();
  tf.transform.translation.y = t.y();
  tf.transform.translation.z = t.z();
  tf.transform.rotation.w = r(0);
  tf.transform.rotation.x = r(1);
  tf.transform.rotation.y = r(2);
  tf.transform.rotation.z = r(3);
  tf_broadcaster_->sendTransform(tf);
}

bool GtsamSlamNode::isKeyframeNeeded(
    const gtsam::Pose3& current_pose) const {
  if (!last_keyframe_pose_.has_value()) {
    return true;
  }

  auto delta = last_keyframe_pose_.value().between(current_pose);
  double trans = delta.translation().norm();

  gtsam::Vector3 rpy = gtsam::Rot3::Logmap(delta.rotation());
  double rot = rpy.norm();

  return (trans > keyframe_trans_thresh_ || rot > keyframe_rot_thresh_);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<GtsamSlamNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}