#ifndef IOT_INIT_H
#define IOT_INIT_H

#include "protocol_extracter/nprotocol_extracter.h"
// #include <nlink_parser/IotFrame0.h>
// #include <ros/ros.h>
#include "nlink_parser/msg/iot_frame0.hpp"
#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <unordered_map>

namespace iot
{
  class Init
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction, rclcpp::Node::SharedPtr node);

  private:
    void InitFrame0(NProtocolExtracter *protocol_extraction);
    // std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
    // ros::NodeHandle nh_;
    rclcpp::Node::SharedPtr nh_;
  };

} // namespace iot
#endif // IOT_INIT_H
