#ifndef MOVEIT_OBJECT_CREATOR__TOOLS__OBJECT_TOOL_HPP_
#define MOVEIT_OBJECT_CREATOR__TOOLS__OBJECT_TOOL_HPP_

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{
class DisplayContext;

namespace properties
{
class StringProperty;
class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace moveit_object_creator
{
namespace tools
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC ObjectTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  ObjectTool();
  ~ObjectTool() override;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::QosProfileProperty * qos_profile_property_;

  rclcpp::QoS qos_profile_;
};

}  // namespace tools
}  // namespace moveit_object_creator

#endif  // MOVEIT_OBJECT_CREATOR__TOOLS__OBJECT_TOOL_HPP_
