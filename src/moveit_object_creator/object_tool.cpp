#include "moveit_object_creator/object_tool.hpp"

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"

namespace moveit_object_creator
{
namespace tools
{

ObjectTool::ObjectTool()
: rviz_default_plugins::tools::PoseTool(),
  qos_profile_(5)
{
  shortcut_key_ = 'o';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "object_pose",
    "The topic on which to publish object poses.",
    getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

ObjectTool::~ObjectTool() = default;

void ObjectTool::onInitialize()
{
  PoseTool::onInitialize();

  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {
      this->qos_profile_ = profile;
    });

  setName("Object Pose Tool");
  updateTopic();
}

void ObjectTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();

  publisher_ = raw_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), qos_profile_);

  clock_ = raw_node->get_clock();
}

void ObjectTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = clock_->now();
  pose_msg.header.frame_id = fixed_frame;

  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = 0.0;
  pose_msg.pose.orientation = orientationAroundZAxis(theta);

  logPose("object", pose_msg.pose.position, pose_msg.pose.orientation, theta, fixed_frame);

  publisher_->publish(pose_msg);
}

}  // namespace tools
}  // namespace moveit_object_creator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_object_creator::tools::ObjectTool, rviz_common::Tool)
