// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"

int main(int argc, char **argv)
{
  // ros::init(argc, argv, "tofsense_parser");
  // ros::NodeHandle nh;
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("tofsense_parser");
  serial::Serial serial;
  initSerial(&serial, nh);

  NProtocolExtracter extracter;
  tofsense::Init init(&extracter, &serial, nh);

  // while (ros::ok())
  while (rclcpp::ok())
  {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes)
    {
      serial.read(str_received, available_bytes);
      extracter.AddNewData(str_received);
    }
    // ros::spinOnce();
    rclcpp::spin_some(nh);
  }
  return EXIT_SUCCESS;
}
