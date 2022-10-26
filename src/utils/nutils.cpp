#include "nutils.h"

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

void TopicAdvertisedTip(const char *topic, rclcpp::Node::ConstSharedPtr node)
{
  // ROS_INFO("%s has been advertised,use 'rostopic "
  //          "echo /%s' to view the data",
  //          topic, topic);
  RCLCPP_INFO(
      node->get_logger(),
      "%s has been advertised,use 'rostopic "
      "echo /%s' to view the data",
      topic, topic);
}
