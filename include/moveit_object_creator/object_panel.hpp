/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#ifndef MOVEIT_OBJECT_CREATOR__OBJECT_PANEL_HPP_
#define MOVEIT_OBJECT_CREATOR__OBJECT_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace moveit_object_creator
{
class ObjectPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit ObjectPanel(QWidget * parent = 0);
  ~ObjectPanel() override;

  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;

  void topicCallback(const geometry_msgs::msg::PoseStamped& pose);
  geometry_msgs::msg::Pose applyPoseOffset(const geometry_msgs::msg::Pose& base_pose, double dx, double dy, double dz);
  geometry_msgs::msg::Pose rotatePoseAroundZ(const geometry_msgs::msg::Pose& pose, double angle_rad);

  QLabel * label_;
  QPushButton * button_;
  QComboBox * comboBox_;
  QLineEdit * nameInput_;
  QLineEdit * textInput1_;
  QLineEdit * textInput2_;
  QLineEdit * textInput3_;
  QLineEdit * textInput4_;
  QSpinBox * numberSelector_;
  QLineEdit * shelf_height_;
  geometry_msgs::msg::PoseStamped object_pose_;
  std::string reference_frame_ = "map";

private Q_SLOTS:
  void buttonActivated();
  void onComboBoxChanged(int index);
};

}  // namespace moveit_object_creator

#endif  // RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_