#include "goto_panel.hpp"

#include <cmath>
#include <vector>

#include <QHBoxLayout>
#include <QRegularExpression>
#include <QVBoxLayout>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace mdetect_rviz_plugins
{

GotoPanel::GotoPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  target_input_ = new QLineEdit(this);
  target_input_->setPlaceholderText(QStringLiteral("x_mm, y_mm, heading_deg   e.g. 1000,1000,90"));
  target_input_->setToolTip(QStringLiteral(
    "Map-frame target in Arduino units: x/y in mm, heading in degrees\n"
    "(CCW-positive, 0 = map +X). Heading may be omitted."));

  go_button_ = new QPushButton(QStringLiteral("Go"), this);
  cancel_button_ = new QPushButton(QStringLiteral("Cancel"), this);
  cancel_button_->setEnabled(false);
  status_label_ = new QLabel(QStringLiteral("Waiting for Nav2..."), this);

  auto * input_row = new QHBoxLayout();
  input_row->addWidget(target_input_, 1);
  input_row->addWidget(go_button_);
  input_row->addWidget(cancel_button_);

  auto * layout = new QVBoxLayout();
  layout->addLayout(input_row);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(target_input_, &QLineEdit::returnPressed, this, &GotoPanel::sendTarget);
  connect(go_button_, &QPushButton::clicked, this, &GotoPanel::sendTarget);
  connect(cancel_button_, &QPushButton::clicked, this, &GotoPanel::cancelGoal);
}

void GotoPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  setStatus(QStringLiteral("Ready. Enter x_mm,y_mm,heading_deg and press Enter."));
}

void GotoPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("LastTarget", target_input_->text());
}

void GotoPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString last;
  if (config.mapGetString("LastTarget", &last)) {
    target_input_->setText(last);
  }
}

void GotoPanel::setStatus(const QString & text)
{
  // QueuedConnection makes this safe from the ROS spin thread.
  QMetaObject::invokeMethod(
    status_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, text));
}

void GotoPanel::sendTarget()
{
  // Accept "1000,1000,90", "(1000, 1000, 90)" or "1000 1000 90"; heading
  // defaults to 0 degrees when omitted.
  QString text = target_input_->text();
  text.remove(QLatin1Char('(')).remove(QLatin1Char(')'));
  const QStringList parts = text.split(
    QRegularExpression(QStringLiteral("[,\\s]+")), Qt::SkipEmptyParts);
  if (parts.size() < 2 || parts.size() > 3) {
    setStatus(QStringLiteral("Invalid target: expected x_mm,y_mm[,heading_deg]"));
    return;
  }
  std::vector<double> values;
  for (const QString & part : parts) {
    bool ok = false;
    values.push_back(part.toDouble(&ok));
    if (!ok) {
      setStatus(QStringLiteral("Invalid number: %1").arg(part));
      return;
    }
  }
  const double x_mm = values[0];
  const double y_mm = values[1];
  const double heading_deg = values.size() == 3 ? values[2] : 0.0;

  if (!client_ || !client_->action_server_is_ready()) {
    setStatus(QStringLiteral("Nav2 navigate_to_pose server is not available"));
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->get_clock()->now();
  goal.pose.pose.position.x = x_mm / 1000.0;
  goal.pose.pose.position.y = y_mm / 1000.0;
  const double yaw = heading_deg * M_PI / 180.0;
  goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this](GoalHandle::SharedPtr handle) {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal_handle_ = handle;
      if (!handle) {
        setStatus(QStringLiteral("Goal rejected by Nav2"));
        QMetaObject::invokeMethod(
          cancel_button_, "setEnabled", Qt::QueuedConnection, Q_ARG(bool, false));
      }
    };
  options.feedback_callback =
    [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
      // Throttle the ~10 Hz feedback stream to one label update per second.
      const auto now = std::chrono::steady_clock::now();
      if (now - last_feedback_print_ < std::chrono::seconds(1)) {
        return;
      }
      last_feedback_print_ = now;
      setStatus(QStringLiteral("Driving: %1 m remaining")
        .arg(fb->distance_remaining, 0, 'f', 2));
    };
  options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        goal_handle_.reset();
      }
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          setStatus(QStringLiteral("Target reached"));
          break;
        case rclcpp_action::ResultCode::CANCELED:
          setStatus(QStringLiteral("Goal cancelled"));
          break;
        case rclcpp_action::ResultCode::ABORTED:
          setStatus(QStringLiteral("Navigation aborted (no valid path or robot stuck)"));
          break;
        default:
          setStatus(QStringLiteral("Navigation ended with unknown result"));
          break;
      }
      QMetaObject::invokeMethod(
        cancel_button_, "setEnabled", Qt::QueuedConnection, Q_ARG(bool, false));
    };

  last_feedback_print_ = std::chrono::steady_clock::now();
  client_->async_send_goal(goal, options);
  cancel_button_->setEnabled(true);
  setStatus(QStringLiteral("Sent target (%1 mm, %2 mm, %3 deg)")
    .arg(x_mm, 0, 'f', 0).arg(y_mm, 0, 'f', 0).arg(heading_deg, 0, 'f', 0));
}

void GotoPanel::cancelGoal()
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
    setStatus(QStringLiteral("Cancelling..."));
  }
}

}  // namespace mdetect_rviz_plugins

PLUGINLIB_EXPORT_CLASS(mdetect_rviz_plugins::GotoPanel, rviz_common::Panel)
