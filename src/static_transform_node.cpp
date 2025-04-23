// ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class StaticTransformNode : public rclcpp::Node {
public:
  StaticTransformNode() : Node("static_transform_node") {
    // Declare parameters with default values
    this->declare_parameter("parent_frame", "base_link");
    this->declare_parameter("child_frame", "imu_link");
    this->declare_parameter("translation_x", 0.0);
    this->declare_parameter("translation_y", 0.0);
    this->declare_parameter("translation_z", 0.0);
    this->declare_parameter("rotation_x", 0.0);
    this->declare_parameter("rotation_y", 0.0);
    this->declare_parameter("rotation_z", 0.0);
    this->declare_parameter("rotation_w", 1.0);
    
    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    // Publish the static transform
    publishTransform();
    
    RCLCPP_INFO(this->get_logger(), "Static transform node initialized");
  }
  
private:
  void publishTransform() {
    geometry_msgs::msg::TransformStamped transform;
    
    // Set header
    transform.header.stamp = this->now();
    transform.header.frame_id = this->get_parameter("parent_frame").as_string();
    transform.child_frame_id = this->get_parameter("child_frame").as_string();
    
    // Set translation
    transform.transform.translation.x = this->get_parameter("translation_x").as_double();
    transform.transform.translation.y = this->get_parameter("translation_y").as_double();
    transform.transform.translation.z = this->get_parameter("translation_z").as_double();
    
    // Set rotation
    transform.transform.rotation.x = this->get_parameter("rotation_x").as_double();
    transform.transform.rotation.y = this->get_parameter("rotation_y").as_double();
    transform.transform.rotation.z = this->get_parameter("rotation_z").as_double();
    transform.transform.rotation.w = this->get_parameter("rotation_w").as_double();
    
    // Publish the transform
    tf_broadcaster_->sendTransform(transform);
    
    RCLCPP_INFO(this->get_logger(), "Published static transform from %s to %s",
                transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
  }
  
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTransformNode>());
  rclcpp::shutdown();
  return 0;
}