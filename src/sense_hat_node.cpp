// C++ Standard Library
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <unistd.h>

// ROS headers
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Project headers
#include "portable_slam/icm20948.hpp"

using namespace std::chrono_literals;

class SenseHatNode : public rclcpp::Node {
public:
  SenseHatNode() : Node("sense_hat_node") {

    // Configureable parameter declarations
    this->declare_parameter("publish_rate", 20.0);     // 20 Hz default
    this->declare_parameter("latency_threshold", 0.1); // 100ms default
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("qos_depth", 10);

    // Create publisher for IMU data with the size of message queue from QoS
    // depth
    const int qos_depth = this->get_parameter("qos_depth").as_int();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", qos);

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
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize IMU sensor: %s",
                   e.what());
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
  std::unique_ptr<ICM20948> sensor_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Initialize as an optional to handle the first update case
  std::optional<rclcpp::Time> last_publish_time_;

  void timer_callback() {
    auto now = this->now();
    if (last_publish_time_.has_value()) {
      double dt = (now - last_publish_time_.value()).seconds();
      double threshold = this->get_parameter("latency_threshold").as_double();

      if (dt > threshold) { // More than 20ms between updates
        RCLCPP_WARN(this->get_logger(),
                    "High publish latency detected: %.3f ms", dt * 1000);
      }
    }
    last_publish_time_ = now;
    try {
      auto message = sensor_msgs::msg::Imu();

      // Set header and using the same timestamp for both checks and message
      message.header.stamp = now;
      message.header.frame_id = this->get_parameter("frame_id").as_string();

      // Read acceleration data
      auto [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z] =
          sensor_->readSensorData();

      // Convert and set linear acceleration (m/s^2)
      message.linear_acceleration.x = sensor_->convertAcceleration(accel_x);
      message.linear_acceleration.y = sensor_->convertAcceleration(accel_y);
      message.linear_acceleration.z = sensor_->convertAcceleration(accel_z);

      // Convert and set angular velocity deg/s to rad/s
      message.angular_velocity.x = sensor_->convertGyro(gyro_x) * M_PI / 180.0;
      message.angular_velocity.y = sensor_->convertGyro(gyro_y) * M_PI / 180.0;
      message.angular_velocity.z = sensor_->convertGyro(gyro_z) * M_PI / 180.0;

      // The covariance matrix is a 3x3 matrix stored as a 9-element array,
      // where:
      // Diagonal elements (0,4,8) represent variances for x, y, z.
      // Off-diagonal elements represent correlations between axes.
      // Variance = standard_deviation².
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

      // Off-diagonal elements (set to 0 if measurements are independent)
      for (size_t i = 0; i < 9; ++i) {
        if (i != 0 && i != 4 && i != 8) {
          message.linear_acceleration_covariance[i] = 0.0;
          message.angular_velocity_covariance[i] = 0.0;
        }
      }

      // Orientation covariance (set to -1 if orientation not provided)
      for (size_t i = 0; i < 9; ++i) {
        message.orientation_covariance[i] = -1;
      }

      // Publish the message
      publisher_->publish(message);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "[SenseHat]Error reading sensor data: %s", e.what());
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
