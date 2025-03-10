// #include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Path.h>
// #include <nlink_parser/LinktrackAnchorframe0.h>
// #include <nlink_parser/LinktrackNodeframe1.h>
// #include <nlink_parser/LinktrackNodeframe2.h>
// #include <nlink_parser/LinktrackTagframe0.h>
// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nlink_parser/msg/linktrack_anchorframe0.hpp"
#include "nlink_parser/msg/linktrack_nodeframe1.hpp"
#include "nlink_parser/msg/linktrack_nodeframe2.hpp"
#include "nlink_parser/msg/linktrack_tagframe0.hpp"


#include <map>
#include <sstream>

#include "nutils.h"

using std::placeholders::_1;
class uwbsubscriber : public rclcpp::Node
{
  public:
    uwbsubscriber():Node("linktrack_example")
    {
      if(!this->get_parameter<std::string>("~map_frame", frameId))
      {
        RCLCPP_INFO(this->get_logger(), "Do not get map frame id");
        frameId = std::string("linktrack_map");
      };
      sub_anchorframe0 = this->create_subscription<nlink_parser::msg::LinktrackAnchorframe0>(
        "topic", 10, std::bind(&uwbsubscriber::Anchorframe0Callback, this, _1));
      sub_tagframe1 = this->create_subscription<nlink_parser::msg::LinktrackTagframe0>(
        "topic", 10, std::bind(&uwbsubscriber::Tagframe0Callback, this, _1));
      sub_nodeframe1 = this->create_subscription<nlink_parser::msg::LinktrackNodeframe1>(
        "topic", 10, std::bind(&uwbsubscriber::Nodeframe1Callback, this, _1));
      sub_nodeframe2 = this->create_subscription<nlink_parser::msg::LinktrackNodeframe2>(
        "topic", 10, std::bind(&uwbsubscriber::Nodeframe2Callback, this, _1));
    }
  private:
    rclcpp::Subscription<nlink_parser::msg::LinktrackAnchorframe0>::SharedPtr sub_anchorframe0;
    rclcpp::Subscription<nlink_parser::msg::LinktrackTagframe0>::SharedPtr sub_tagframe1;
    rclcpp::Subscription<nlink_parser::msg::LinktrackNodeframe1>::SharedPtr sub_nodeframe1;
    rclcpp::Subscription<nlink_parser::msg::LinktrackNodeframe2>::SharedPtr sub_nodeframe2;
    void Anchorframe0Callback(const nlink_parser::msg::LinktrackAnchorframe0 &msg);
    void Nodeframe1Callback(const nlink_parser::msg::LinktrackNodeframe1 &msg);
    void Tagframe0Callback(const nlink_parser::msg::LinktrackTagframe0 &msg);
    void Nodeframe2Callback(const nlink_parser::msg::LinktrackNodeframe2 &msg);
    std::string frameId;
    struct PosePair
    {
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
      geometry_msgs::msg::PoseStamped msg;
      inline void publish() { publisher->publish(msg); }
    };
};

void uwbsubscriber::Anchorframe0Callback(const nlink_parser::msg::LinktrackAnchorframe0 &msg)
{
  static std::map<uint8_t, PosePair> poses;
  for (const auto &node : msg.nodes)
  {
    auto id = node.id;

    if (!poses.count(id))
    {
      std::ostringstream string_stream;
      string_stream << "nlt_anchorframe0_pose_node" << static_cast<int>(id);
      auto topic = string_stream.str();
      poses[id].publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
      TopicAdvertisedTip(topic.c_str(), this->shared_from_this());
      auto &msg_pose = poses[id].msg;
      msg_pose.header.frame_id = frameId;
      msg_pose.pose.orientation.w = 0;
      msg_pose.pose.orientation.x = 0;
      msg_pose.pose.orientation.y = 0;
      msg_pose.pose.orientation.z = 1;
    }
    auto &msg_pose = poses[id].msg;
    msg_pose.header.stamp = rclcpp::Time();
    msg_pose.pose.position.x = static_cast<double>(node.pos_3d[0]);
    msg_pose.pose.position.y = static_cast<double>(node.pos_3d[1]);
    msg_pose.pose.position.z = static_cast<double>(node.pos_3d[2]);
    poses.at(id).publish();
  }
}

void uwbsubscriber::Nodeframe1Callback(const nlink_parser::msg::LinktrackNodeframe1 &msg)
{
  static std::map<uint8_t, PosePair> poses;
  for (const auto &node : msg.nodes)
  {
    auto id = node.id;

    if (!poses.count(id))
    {
      std::ostringstream string_stream;
      string_stream << "nlt_nodeframe1_pose_node" << static_cast<int>(id);
      auto topic = string_stream.str();
      poses[id].publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
      TopicAdvertisedTip(topic.c_str(), this->shared_from_this());
      auto &msg_pose = poses[id].msg;
      msg_pose.header.frame_id = frameId;
      msg_pose.pose.orientation.w = 0;
      msg_pose.pose.orientation.x = 0;
      msg_pose.pose.orientation.y = 0;
      msg_pose.pose.orientation.z = 1;
    }
    auto &msg_pose = poses[id].msg;
    msg_pose.header.stamp = rclcpp::Time();
    msg_pose.pose.position.x = static_cast<double>(node.pos_3d[0]);
    msg_pose.pose.position.y = static_cast<double>(node.pos_3d[1]);
    msg_pose.pose.position.z = static_cast<double>(node.pos_3d[2]);
    poses.at(id).publish();
  }
}

void uwbsubscriber::Tagframe0Callback(const nlink_parser::msg::LinktrackTagframe0 &msg)
{
  static PosePair *pose = nullptr;
  if (!pose)
  {
    pose = new PosePair;
    auto topic = "nlt_tagframe0_pose";
    pose->publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
    TopicAdvertisedTip(topic, this->shared_from_this());
    pose->msg.header.frame_id = frameId;
  }
  auto &msg_pose = pose->msg;
  msg_pose.header.stamp = rclcpp::Time();
  msg_pose.pose.orientation.w = static_cast<double>(msg.quaternion[0]);
  msg_pose.pose.orientation.x = static_cast<double>(msg.quaternion[1]);
  msg_pose.pose.orientation.y = static_cast<double>(msg.quaternion[2]);
  msg_pose.pose.orientation.z = static_cast<double>(msg.quaternion[3]);
  msg_pose.pose.position.x = static_cast<double>(msg.pos_3d[0]);
  msg_pose.pose.position.y = static_cast<double>(msg.pos_3d[1]);
  msg_pose.pose.position.z = static_cast<double>(msg.pos_3d[2]);
  pose->publish();
}

void uwbsubscriber::Nodeframe2Callback(const nlink_parser::msg::LinktrackNodeframe2 &msg)
{
  static PosePair *pose = nullptr;
  if (!pose)
  {
    pose = new PosePair;
    auto topic = "nlt_nodeframe2_pose";
    pose->publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
    TopicAdvertisedTip(topic, this->shared_from_this());
    pose->msg.header.frame_id = frameId;
  }
  auto &msg_pose = pose->msg;
  msg_pose.header.stamp = rclcpp::Time();
  msg_pose.pose.orientation.w = static_cast<double>(msg.quaternion[0]);
  msg_pose.pose.orientation.x = static_cast<double>(msg.quaternion[1]);
  msg_pose.pose.orientation.y = static_cast<double>(msg.quaternion[2]);
  msg_pose.pose.orientation.z = static_cast<double>(msg.quaternion[3]);
  msg_pose.pose.position.x = static_cast<double>(msg.pos_3d[0]);
  msg_pose.pose.position.y = static_cast<double>(msg.pos_3d[1]);
  msg_pose.pose.position.z = static_cast<double>(msg.pos_3d[2]);
  pose->publish();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uwbsubscriber>());
  return 0;
}
