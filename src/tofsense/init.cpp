
#include "init.h"

#include "nlink_protocol.h"
#include "nlink_unpack/nlink_tofsense_frame0.h"
#include "nlink_unpack/nlink_utils.h"
#include "nutils.h"

class NTS_ProtocolFrame0 : public NLinkProtocol
{
public:
  NTS_ProtocolFrame0();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

NTS_ProtocolFrame0::NTS_ProtocolFrame0()
    : NLinkProtocol(true, g_nts_frame0.fixed_part_size,
                    {g_nts_frame0.frame_header, g_nts_frame0.function_mark})
{
}

void NTS_ProtocolFrame0::UnpackFrameData(const uint8_t *data)
{
  g_nts_frame0.UnpackData(data, length());
}

namespace tofsense
{
  nlink_parser::msg::TofsenseFrame0 g_msg_frame0;

#pragma pack(push, 1)
  struct
  {
    char header[2]{0x57, 0x10};
    uint8_t reserved0[2]{0xff, 0xff};
    uint8_t id{};
    uint8_t reserved1[2]{0xff, 0xff};
    uint8_t checkSum{};
  } g_command_read;
#pragma pack(pop)

  Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial, rclcpp::Node::SharedPtr nh)
      : serial_(serial)
  {
    nh_ = nh;
    if(!nh_->get_parameter<bool>("~inquire_mode", is_inquire_mode_))
    {
      RCLCPP_INFO(nh_->get_logger(), "No inquire_mode");
      is_inquire_mode_ = false;
    }
    InitFrame0(protocol_extraction);
  }

  void Init::InitFrame0(NProtocolExtracter *protocol_extraction)
  {
    static auto protocol_frame0_ = new NTS_ProtocolFrame0;
    static auto is_inquire_publisher_ = nh_->create_publisher<nlink_parser::msg::TofsenseCascade>("nlink_tofsense_cascade", 50);
    static auto no_inquire_publisher_ = nh_->create_publisher<nlink_parser::msg::TofsenseFrame0>("nlink_tofsense_frame0", 50);
    protocol_extraction->AddProtocol(protocol_frame0_);
    if (is_inquire_mode_)
    {
      TopicAdvertisedTip("nlink_tofsense_cascade", nh_);
    }
    else
    {
      TopicAdvertisedTip("nlink_tofsense_frame0", nh_);
    }
    protocol_frame0_->SetHandleDataCallback(
        [=]
        {
          const auto &data = g_nts_frame0.result;

          g_msg_frame0.id = data.id;
          g_msg_frame0.system_time = data.system_time;
          g_msg_frame0.dis = data.dis;
          g_msg_frame0.dis_status = data.dis_status;
          g_msg_frame0.signal_strength = data.signal_strength;
          g_msg_frame0.range_precision = data.range_precision;

          if (is_inquire_mode_)
          {
            frame0_map_[data.id] = g_msg_frame0;
          }
          else
          {
            no_inquire_publisher_->publish(g_msg_frame0);
          }
        });

    if (is_inquire_mode_)
    {
      timer_scan_ = rclcpp::create_timer(nh_, nh_->get_clock(), rclcpp::Duration::from_seconds(1.0/frequency_), [=]() -> void{
        frame0_map_.clear();
        node_index_ = 0;
        timer_read_->reset(); });
      timer_read_ = nh_->create_wall_timer(std::chrono::duration<int64_t, std::milli>(6), [=]() ->void{
        if (node_index_ >= 8)
        {
          if (!frame0_map_.empty())
          {
            nlink_parser::msg::TofsenseCascade msg_cascade;
            for (const auto &msg : frame0_map_)
            {
              msg_cascade.nodes.push_back(msg.second);
            }
            is_inquire_publisher_->publish(msg_cascade);
          }
          timer_read_->cancel();
        }
        else
        {
          g_command_read.id = node_index_;
          auto data = reinterpret_cast<uint8_t *>(&g_command_read);
          NLink_UpdateCheckSum(data, sizeof(g_command_read));
          serial_->write(data, sizeof(g_command_read));
          ++node_index_;
        }
      });
    }
  }

} // namespace tofsense
