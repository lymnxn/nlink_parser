#include "nutils.h"

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

void TopicAdvertisedTip(const char *topic)
{
  // ROS_INFO("%s has been advertised,use 'rostopic "
  //          "echo /%s' to view the data",
  //          topic, topic);
  RCLCPP_INFO(
      get_logger(),
      "%s has been advertised,use 'rostopic "
      "echo /%s' to view the data",
      topic, topic);
}
