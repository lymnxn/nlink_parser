#ifndef LINKTRACKAOAINIT_H
#define LINKTRACKAOAINIT_H

#include "nlink_parser/msg/linktrack_aoa_nodeframe0.hpp"
#include "nlink_parser/msg/linktrack_nodeframe0.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"

#include "protocol_extracter/nprotocol_extracter.h"

namespace linktrack_aoa
{
  class Init
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction,
                  serial::Serial *serial, rclcpp::Node::SharedPtr node);

  private:
    void initDataTransmission();
    void initNodeFrame0(NProtocolExtracter *protocol_extraction);
    void InitAoaNodeFrame0(NProtocolExtracter *protocol_extraction);
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;
  };
} // namespace linktrack_aoa

#endif // LINKTRACKAOAINIT_H
