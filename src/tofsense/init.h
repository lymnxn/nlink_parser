#ifndef TOFSENSEINIT_H
#define TOFSENSEINIT_H

// #include <nlink_parser/TofsenseCascade.h>
// #include <nlink_parser/TofsenseFrame0.h>
// #include <ros/ros.h>
#include "nlink_parser/msg/tofsense_cascade.hpp"
#include "nlink_parser/msg/tofsense_frame0.hpp"
#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>

#include <map>
#include "protocol_extracter/nprotocol_extracter.h"

namespace tofsense
{
  class Init
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction,
                  serial::Serial *serial, rclcpp::Node::SharedPtr nh);

  private:
    void InitFrame0(NProtocolExtracter *protocol_extraction);

    std::map<int, nlink_parser::msg::TofsenseFrame0> frame0_map_;

    serial::Serial *serial_;

    const int frequency_ = 10;
    bool is_inquire_mode_ = true;

    rclcpp::Node::SharedPtr nh_;
    rclcpp::TimerBase::SharedPtr timer_scan_;
    rclcpp::TimerBase::SharedPtr timer_read_;
    uint8_t node_index_ = 0;
  };

} // namespace tofsense
#endif // TOFSENSEINIT_H
