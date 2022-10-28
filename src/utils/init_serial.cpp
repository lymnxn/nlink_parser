#include "init_serial.h"

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include <string>
/*
void enumerate_ports() {
  auto devices_found = serial::list_ports();
  auto iter = devices_found.begin();
  while (iter != devices_found.end()) {
    serial::PortInfo device = *iter++;

    printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
           device.hardware_id.c_str());
  }
  std::string test;
  test.clear();
}
*/

void initSerial(serial::Serial *serial, rclcpp::Node::ConstSharedPtr node)
{
  try
  {
    std::string port_name("/dev/ttyUSB0");
    if(node->get_parameter<std::string>("~port_name", port_name))
    {
      RCLCPP_ERROR(node->get_logger(), "Fail to read port_name, %s", port_name.c_str());
      port_name = std::string("dev/ttyUSB0");
    }
    int baud_rate(921600);
    if(!node->get_parameter<int>("~baud_rate", baud_rate))
    {
      RCLCPP_ERROR(node->get_logger(), "Fail to read baud_rate, %d", baud_rate);
      baud_rate = 921600;
    }

    serial->setPort(port_name);
    serial->setBaudrate(static_cast<uint32_t>(baud_rate));
    // ROS_INFO("try to open serial port with %s,%d", port_name.data(), baud_rate);
    RCLCPP_INFO(node->get_logger(), "try to open serial port with %s,%d", port_name.data(), baud_rate);
    auto timeout = serial::Timeout::simpleTimeout(10);
    // without setTimeout,serial can not write any data
    // https://stackoverflow.com/questions/52048670/can-read-but-cannot-write-serial-ports-on-ubuntu-16-04/52051660?noredirect=1#comment91056825_52051660
    serial->setTimeout(timeout);
    serial->open();
  

    if (serial->isOpen())
    {
      // ROS_INFO("Serial port opened successfully, waiting for data.");
      RCLCPP_INFO(node->get_logger(), "Serial port opened successfully, waiting for data.");
    }
    else
    {
      // ROS_ERROR("Failed to open serial port, please check and retry.");
      RCLCPP_ERROR(node->get_logger(),"Failed to open serial port, please check and retry.");
      exit(EXIT_FAILURE);
    }
  }
  catch (const std::exception &e)
  {
    // ROS_ERROR("Unhandled Exception: %s", e.what());
    RCLCPP_ERROR(node->get_logger(), "Unhandled Exception: %s", e.what());
    exit(EXIT_FAILURE);
  }
}
