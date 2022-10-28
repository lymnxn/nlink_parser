#ifndef TOFSENSEMINIT_H
#define TOFSENSEMINIT_H

#include "protocol_extracter/nprotocol_extracter.h"
#include "nlink_parser/msg/tofsense_m_frame0.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tofsensem
{
  class Init
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction, rclcpp::Node::SharedPtr nh);

  private:
    void InitFrame0(NProtocolExtracter *protocol_extraction);
    rclcpp::Node::SharedPtr nh_;
  };

} // namespace tofsensem
#endif // TOFSENSEMINIT_H
