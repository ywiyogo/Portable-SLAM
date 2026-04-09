#include <cmath>
#include <functional>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float64.hpp"

#include "portable_slam/lps22hb.hpp"

using namespace std::chrono_literals;

class PressureNode : public rclcpp::Node {
public:
  PressureNode() : Node("lps22hb_node") {
    this->declare_parameter("publish_rate", 25.0);
    this->declare_parameter("i2c_bus", 5);
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("qos_depth", 5);
    this->declare_parameter("sea_level_pressure", SEA_LEVEL_PRESSURE_HPA);

    const int qos_depth = this->get_parameter("qos_depth").as_int();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth))
                   .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                   .durability(rclcpp::DurabilityPolicy::Volatile)
                   .history(rclcpp::HistoryPolicy::KeepLast);

    pressure_pub_ =
        this->create_publisher<sensor_msgs::msg::FluidPressure>("/pressure",
                                                                 qos);
    altitude_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("/altitude", qos);

    double rate = this->get_parameter("publish_rate").as_double();
    if (rate <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid publish rate: %f", rate);
      rclcpp::shutdown();
      return;
    }
    auto period = std::chrono::duration<double>(1.0 / rate);

    int bus = this->get_parameter("i2c_bus").as_int();
    try {
      sensor_ = std::make_unique<LPS22HB>(bus);
      RCLCPP_INFO(this->get_logger(),
                  "LPS22HB pressure sensor initialized on bus %d", bus);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to initialize LPS22HB: %s bus %d", e.what(), bus);
      rclcpp::shutdown();
      return;
    }

    timer_ = this->create_wall_timer(
        period, std::bind(&PressureNode::timer_callback, this));
  }

  ~PressureNode() {
    if (timer_) {
      timer_->cancel();
    }
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altitude_pub_;
  std::unique_ptr<LPS22HB> sensor_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<rclcpp::Time> last_publish_time_;
  double reference_pressure_ = SEA_LEVEL_PRESSURE_HPA;

  void timer_callback() {
    auto now = this->now();

    if (last_publish_time_.has_value()) {
      double dt = (now - last_publish_time_.value()).seconds();
      double threshold =
          1000.0 / this->get_parameter("publish_rate").as_double();
      if (dt > threshold) {
        RCLCPP_WARN(this->get_logger(),
                     "High publish latency detected: %.3f ms", dt * 1000);
      }
    }
    last_publish_time_ = now;

    try {
      PressureData data = sensor_->readData();

      if (data.pressure_hpa <= 0.0) {
        return;
      }

      reference_pressure_ =
          this->get_parameter("sea_level_pressure").as_double();

      sensor_msgs::msg::FluidPressure pressure_msg;
      pressure_msg.header.stamp = now;
      pressure_msg.header.frame_id =
          this->get_parameter("frame_id").as_string();
      pressure_msg.fluid_pressure = data.pressure_hpa * 100.0;
      pressure_msg.variance = PRESSURE_VARIANCE;
      pressure_pub_->publish(pressure_msg);

      std_msgs::msg::Float64 altitude_msg;
      altitude_msg.data =
          sensor_->getAltitude(data.pressure_hpa, reference_pressure_);
      altitude_pub_->publish(altitude_msg);

    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(),
                   "Error reading LPS22HB data: %s", e.what());
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PressureNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("lps22hb_node"), "Node crashed: %s",
                 e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}