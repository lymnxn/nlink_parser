#include "rclcpp/rclcpp.hpp"

#include "init.h"
#include "init_serial.h"
#include "nlink_unpack/nlink_tofsensem_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "protocol_extracter/nprotocol_extracter.h"
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("tofsensem_parser");
  serial::Serial serial;
  initSerial(&serial, nh);

  NProtocolExtracter extracter;
  tofsensem::Init init(&extracter, nh);

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
    rclcpp::spin_some(nh);
  }
  return EXIT_SUCCESS;
}
