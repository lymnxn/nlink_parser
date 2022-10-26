
#include "init.h"

#include "nlink_protocol.h"
#include "nlink_unpack/nlink_iot_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "nutils.h"
#include "rclcpp/rclcpp.hpp"

namespace
{
  class ProtocolFrame0 : public NLinkProtocol
  {
  public:
    ProtocolFrame0()
        : NLinkProtocol(true, g_iot_frame0.fixed_part_size,
                        {g_iot_frame0.frame_header, g_iot_frame0.function_mark})
    {
    }

  protected:
    void UnpackFrameData(const uint8_t *data) override
    {
      g_iot_frame0.UnpackData(data, length());
    }
  };

} // namespace

namespace iot
{
  // nlink_parser::IotFrame0 g_msg_iotframe0;
  nlink_parser::msg::IotFrame0 g_msg_iotframe0;

  Init::Init(NProtocolExtracter *protocol_extraction, rclcpp::Node::SharedPtr node)
  {
    InitFrame0(protocol_extraction);
    nh_ = node;
  }

  void Init::InitFrame0(NProtocolExtracter *protocol_extraction)
  {
    static auto protocol = new ProtocolFrame0;
    auto topic = "nlink_iot_frame0";
    static auto publishers_ = nh_->create_publisher<nlink_parser::msg::IotFrame0>(topic, 50);
    TopicAdvertisedTip(topic, nh_);
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback(
        [=]
        {
          // if (!publishers_[protocol])
          // {
          //   ros::NodeHandle nh_;
          //   auto topic = "nlink_iot_frame0";
          //   publishers_[protocol] =
          //       nh_.advertise<nlink_parser::IotFrame0>(topic, 50);
          //   TopicAdvertisedTip(topic);
          // }

          const auto &data = g_iot_frame0;
          g_msg_iotframe0.uid = data.uid;
          g_msg_iotframe0.nodes.resize(IOT_FRAME0_NODE_COUNT);
          for (int i = 0; i < IOT_FRAME0_NODE_COUNT; ++i)
          {
            g_msg_iotframe0.nodes[i].uid = data.nodes[i].uid;
            g_msg_iotframe0.nodes[i].cnt = data.nodes[i].cnt;
            g_msg_iotframe0.nodes[i].dis = data.nodes[i].dis;
            g_msg_iotframe0.nodes[i].aoa_angle_horizontal =
                data.nodes[i].aoa_angle_horizontal;
            g_msg_iotframe0.nodes[i].aoa_angle_vertical =
                data.nodes[i].aoa_angle_vertical;
          }

          // publishers_.at(protocol).publish(g_msg_iotframe0);
          publishers_->publish(g_msg_iotframe0);
        });
  }

} // namespace iot
