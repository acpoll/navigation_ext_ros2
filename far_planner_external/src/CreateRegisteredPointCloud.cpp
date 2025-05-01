#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"  // Added for PointCloud2
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

using std::placeholders::_1;

class CreateRegisteredPointCloud : public rclcpp::Node
{
public:
  CreateRegisteredPointCloud()
  : Node("CreateRegisteredPointCloud")
  {

    target_frame_ = this->declare_parameter<std::string>("target_frame", "odom");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/point_cloud", 10, std::bind(&CreateRegisteredPointCloud::pointcloud_callback, this, _1));

    registered_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/registered_point_cloud", 10);

  }

private:
void pointcloud_callback(const sensor_msgs::msg::PointCloud2 & msg) const
{
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                             "Received PointCloud2 with %u points", msg.width * msg.height);

    std::string fromFrame = msg.header.frame_id;
    std::string toFrame = target_frame_.c_str();
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
            transformStamped = tf_buffer_->lookupTransform(
                    toFrame, fromFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                                     "Could not transform %s to %s: %s",
                                                     fromFrame.c_str(), toFrame.c_str(), ex.what());
            return;
    }

    sensor_msgs::msg::PointCloud2 transformed_pointcloud;
    try {
            tf2::doTransform(msg, transformed_pointcloud, transformStamped);
    } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                                     "Transform failed: %s", ex.what());
            return;
    }

    registered_pointcloud_publisher_->publish(transformed_pointcloud);
}

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_pointcloud_publisher_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CreateRegisteredPointCloud>());
  rclcpp::shutdown();
  return 0;
}