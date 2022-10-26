// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include "init.h"
#include "init_serial.h"
#include "protocol_extracter/nprotocol_extracter.h"
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
  // ros::init(argc, argv, "iot_parser");
  // ros::NodeHandle nh;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("iot_parser");
  serial::Serial serial;
  initSerial(&serial, node);

  NProtocolExtracter extracter;
  iot::Init init(&extracter, node);

  // while (ros::ok())
  while(rclcpp::ok())
  {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes)
    {
      serial.read(str_received, available_bytes);
      extracter.AddNewData(str_received);
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // ros::spinOnce();
    rclcpp::spin_some(node);
  }
  return EXIT_SUCCESS;
}
