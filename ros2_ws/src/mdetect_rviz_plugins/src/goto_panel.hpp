// RViz panel with a command-line style input for Nav2 goals.
//
// Type a target as "x_mm,y_mm,heading_deg" (heading optional, CCW-positive
// degrees, 0 = map +X) and press Enter or Go; the panel sends a
// NavigateToPose action goal in the map frame and shows live progress.
#pragma once

#include <chrono>
#include <mutex>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

namespace mdetect_rviz_plugins
{

class GotoPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GotoPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void sendTarget();
  void cancelGoal();

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // Action callbacks arrive on RViz's ROS spin thread; route all widget
  // updates through the Qt event loop.
  void setStatus(const QString & text);

  QLineEdit * target_input_ = nullptr;
  QPushButton * go_button_ = nullptr;
  QPushButton * cancel_button_ = nullptr;
  QLabel * status_label_ = nullptr;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  std::mutex goal_mutex_;
  GoalHandle::SharedPtr goal_handle_;
  std::chrono::steady_clock::time_point last_feedback_print_;
};

}  // namespace mdetect_rviz_plugins
