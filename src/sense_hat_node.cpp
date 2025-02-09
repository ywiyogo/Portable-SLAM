// C++ Standard Library
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <unistd.h>

// ROS headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_srvs/srv/trigger.hpp"

// Project headers
#include "portable_slam/icm20948.hpp"

using namespace std::chrono_literals;

class SenseHatNode : public rclcpp::Node {
public:
  SenseHatNode() : Node("sense_hat_node") {
    // Add calibration service
    calibration_service_ = this->create_service<std_srvs::srv::Trigger>(
        "calibrate_imu",
        std::bind(&SenseHatNode::calibrationCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Configureable parameter declarations
    this->declare_parameter("publish_rate", 20.0);     // 20 Hz default
    this->declare_parameter("latency_threshold", 0.2); // 200ms default
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("qos_depth", 10);
    this->declare_parameter("calibration_mode",
                            false); // Add calibration mode parameter

    // Create publisher for IMU data with the size of message queue from QoS
    // depth
    const int qos_depth = this->get_parameter("qos_depth").as_int();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth))
                   .reliability(rclcpp::ReliabilityPolicy::Reliable)
                   .durability(rclcpp::DurabilityPolicy::Volatile)
                   .history(rclcpp::HistoryPolicy::KeepLast);
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", qos);
    publisher_mag_ =
        this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);

    double rate = this->get_parameter("publish_rate").as_double();
    if (rate <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid publish rate: %f", rate);
      rclcpp::shutdown();
      return;
    }
    auto period = std::chrono::duration<double>(1.0 / rate);

    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
        period, std::bind(&SenseHatNode::timer_callback, this));

    this->declare_parameter("i2c_bus", 5); // Default to 5
    int bus = this->get_parameter("i2c_bus").as_int();
    try {
      // Initialize ICM20948 sensor
      sensor_ = std::make_unique<ICM20948>(bus);
      RCLCPP_INFO(this->get_logger(), "IMU sensor initialized successfully");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to initialize IMU sensor: %s bus %d", e.what(), bus);
      rclcpp::shutdown();
    }
  }
  ~SenseHatNode() {
    if (timer_) {
      timer_->cancel();
    }
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_mag_;
  std::unique_ptr<ICM20948> sensor_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
  // Initialize as an optional to handle the first update case
  std::optional<rclcpp::Time> last_publish_time_;

  void calibrationCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    try {
      RCLCPP_INFO(this->get_logger(), "Starting IMU calibration...");
      sensor_->performCalibration();
      response->success = true;
      response->message = "IMU calibration completed successfully";
      RCLCPP_INFO(this->get_logger(), "IMU calibration completed");
    } catch (const std::exception &e) {
      response->success = false;
      response->message = std::string("Calibration failed: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Calibration failed: %s", e.what());
    }
  }

  void timer_callback() {
    auto now = this->now();
    if (last_publish_time_.has_value()) {
      double dt = (now - last_publish_time_.value()).seconds();
      double threshold =
          1000. / this->get_parameter("publish_rate").as_double();

      if (dt > threshold) {
        RCLCPP_WARN(this->get_logger(),
                    "High publish latency detected: %.3f ms", dt * 1000);
      }
    }
    last_publish_time_ = now;
    try {
      auto message = sensor_msgs::msg::Imu();
      auto mag_message = sensor_msgs::msg::MagneticField();
      // Set header and using the same timestamp for both checks and message
      message.header.stamp = now;
      mag_message.header.stamp = now;
      message.header.frame_id = this->get_parameter("frame_id").as_string();
      mag_message.header.frame_id = this->get_parameter("frame_id").as_string();

      // Read acceleration data
      auto imu_data = sensor_->readSensorData();

      // Convert and set linear acceleration (m/s^2)
      message.linear_acceleration.x =
          sensor_->convertAcceleration(imu_data.accel.x);
      message.linear_acceleration.y =
          sensor_->convertAcceleration(imu_data.accel.y);
      message.linear_acceleration.z =
          sensor_->convertAcceleration(imu_data.accel.z);

      // Convert and set angular velocity deg/s to rad/s
      message.angular_velocity.x =
          sensor_->convertGyro(imu_data.gyro.x) * M_PI / 180.0;
      message.angular_velocity.y =
          sensor_->convertGyro(imu_data.gyro.y) * M_PI / 180.0;
      message.angular_velocity.z =
          sensor_->convertGyro(imu_data.gyro.z) * M_PI / 180.0;

      mag_message.magnetic_field.x =
          sensor_->convertMagneticField(imu_data.mag.x);
      mag_message.magnetic_field.y =
          sensor_->convertMagneticField(imu_data.mag.y);
      mag_message.magnetic_field.z =
          sensor_->convertMagneticField(imu_data.mag.z);
      // The covariance matrix is a 3x3 matrix stored as a 9-element array,
      // where:
      // Diagonal elements (0,4,8) represent variances for x, y, z.
      // Off-diagonal elements represent correlations between axes.
      // Variance = standard_deviation2.
      // Lower values indicate higher confidence in measurements.
      // Higher values indicate more uncertainty.

      // Set main diagonal elements for linear acceleration.
      message.linear_acceleration_covariance[0] = sensor_->ACCEL_VARIANCE; // xx
      message.linear_acceleration_covariance[4] = sensor_->ACCEL_VARIANCE; // yy
      message.linear_acceleration_covariance[8] = sensor_->ACCEL_VARIANCE; // zz

      // Set main diagonal elements for angular velocity
      message.angular_velocity_covariance[0] = sensor_->GYRO_VARIANCE; // xx
      message.angular_velocity_covariance[4] = sensor_->GYRO_VARIANCE; // yy
      message.angular_velocity_covariance[8] = sensor_->GYRO_VARIANCE; // zz

      mag_message.magnetic_field_covariance[0] = sensor_->MAG_VARIANCE;
      mag_message.magnetic_field_covariance[4] = sensor_->MAG_VARIANCE;
      mag_message.magnetic_field_covariance[8] = sensor_->MAG_VARIANCE;
      // Off-diagonal elements (set to 0 if measurements are independent)
      for (size_t i = 0; i < 9; ++i) {
        if (i != 0 && i != 4 && i != 8) {
          message.linear_acceleration_covariance[i] = 0.0;
          message.angular_velocity_covariance[i] = 0.0;
          mag_message.magnetic_field_covariance[i] = 0.0;
        }
      }

      // Orientation covariance (set to -1 if orientation not provided)
      for (size_t i = 0; i < 9; ++i) {
        message.orientation_covariance[i] = -1;
      }

      // Only publish magnetometer data if it's valid (non-zero)
      if (imu_data.mag.x != 0 || imu_data.mag.y != 0 || imu_data.mag.z != 0) {
        publisher_mag_->publish(mag_message);
      }

      // Always publish IMU data since accelerometer and gyroscope update faster
      publisher_->publish(message);
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(),
                  "[SenseHat] Error reading sensor data: %s", e.what());
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SenseHatNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("sense_hat_node"), "Node crashed: %s",
                 e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
