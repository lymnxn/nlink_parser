#include "rclcpp/rclcpp.hpp"

#include "init.h"
#include "init_serial.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("linktrack_aoa");
  serial::Serial serial;
  initSerial(&serial, node);
  NProtocolExtracter protocol_extraction;
  linktrack_aoa::Init aoaInit(&protocol_extraction, &serial, node);
  rclcpp::Rate loop_rate(1000);
  while (rclcpp::ok())
  {
    auto available_bytes = serial.available();
    std::string str_received;
    if (available_bytes)
    {
      serial.read(str_received, available_bytes);
      protocol_extraction.AddNewData(str_received);
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}
