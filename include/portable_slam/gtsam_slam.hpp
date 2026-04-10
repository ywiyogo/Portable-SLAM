/**
 * @brief GTSAM factor graph SLAM node for handheld portable SLAM.
 *        Phase 2c: IMU + rf2o + altitude + gravity + magnetometer + occupancy grid.
 * @author Yongkie Wiyongo
 */

#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose2.h>
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
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
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
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pressureCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

  /// Store laser scan for map update at next keyframe
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void addNewKeyframe(const nav_msgs::msg::Odometry::SharedPtr& odom_msg);
  void publishOdometry(const gtsam::Pose3& pose, const gtsam::Velocity3& vel,
                       const rclcpp::Time& stamp);
  bool isKeyframeNeeded(const gtsam::Pose3& current_pose) const;
  gtsam::Rot3 gravityToRotation(double ax, double ay, double az) const;
  double magnetometerYaw(double mx, double my, double mz,
                         const gtsam::Rot3& tilt) const;
  bool isMagConsistent(double mx, double my, double mz) const;

  /// Update occupancy grid with latest laser scan at the given pose
  void updateMap(const gtsam::Pose2& pose_2d, const sensor_msgs::msg::LaserScan& scan);

  /// Publish the current occupancy grid map
  void publishMap(const rclcpp::Time& stamp);

  /// Convert world (x,y) to grid indices; returns false if out of bounds
  bool worldToGrid(double wx, double wy, int& gx, int& gy) const;

  /// Convert grid indices to world coordinates
  void gridToWorld(int gx, int gy, double& wx, double& wy) const;

  /// Bresenham ray-trace from (x0,y0) to (x1,y1), clearing cells along the ray
  void rayTrace(int x0, int y0, int x1, int y1);

  void declareParameters();
  void loadParameters();
  void initPublishers();
  void initSubscribers();
  void initGtsam();
  void initMap();
  gtsam::ISAM2Params makeIsam2Params() const;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
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

  gtsam::Vector3 latest_accel_{0.0, 0.0, 0.0};
  gtsam::Vector3 latest_mag_{0.0, 0.0, 0.0};
  bool mag_received_{false};

  std::optional<sensor_msgs::msg::LaserScan> latest_scan_;

  // Occupancy grid map state
  std::vector<int8_t> map_data_;
  std::vector<double> log_odds_;
  size_t map_width_{0};
  size_t map_height_{0};
  double map_resolution_{0.05};
  double map_origin_x_{-50.0};
  double map_origin_y_{-50.0};
  double log_odds_hit_{0.7};
  double log_odds_miss_{-0.2};
  double log_odds_clamp_max_{5.0};
  double log_odds_clamp_min_{-5.0};
  double min_laser_range_{0.5};
  double max_laser_range_{12.0};
  std::string map_frame_{"map"};
  double map_update_interval_{2.0};
  rclcpp::Time last_map_publish_time_{0, 0, RCL_ROS_TIME};
  bool map_updated_{false};

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
  double gravity_tilt_roll_sigma_{0.01};
  double gravity_tilt_pitch_sigma_{0.01};
  double gravity_tilt_yaw_sigma_{10.0};
  double mag_yaw_sigma_indoor_{5.0};
  double mag_yaw_sigma_outdoor_{0.1};
  double mag_consistency_thresh_{0.3};
  double mag_reference_norm_{50.0};
  std::string scan_topic_{"scan"};
  std::string mag_topic_{"imu/mag"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string imu_frame_{"imu_link"};
  std::string imu_topic_{"imu/data_raw"};
  std::string odom_topic_{"odom_rf2o"};
  std::string pressure_topic_{"pressure"};
  std::string output_odom_topic_{"odometry/filtered"};
};