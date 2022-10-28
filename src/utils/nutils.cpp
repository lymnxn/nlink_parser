#include "nutils.h"

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

void TopicAdvertisedTip(const char *topic, rclcpp::Node::ConstSharedPtr node)
{
  RCLCPP_INFO(
      node->get_logger(),
      "node name %s: %s has been advertised,use 'ros2 topic "
      "echo /%s' to view the data",
      node->get_name(), topic, topic);
}
