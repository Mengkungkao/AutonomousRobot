#include "teleop_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace mdetect_rviz_plugins
{

// Matches the serial bridge's clamps (max_linear_m_s / max_angular_rad_s).
constexpr double kMaxLinear = 0.25;
constexpr double kMaxAngular = 2.1;

TeleopPanel::TeleopPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  enable_button_ = new QPushButton(QStringLiteral("Teleop OFF"), this);
  enable_button_->setCheckable(true);
  enable_button_->setToolTip(QStringLiteral(
    "ON: hold W/A/S/D (or the buttons) to drive; idle hands give control\n"
    "back to Nav2 automatically. OFF: panel publishes nothing."));

  forward_button_ = new QPushButton(QStringLiteral("▲"), this);
  backward_button_ = new QPushButton(QStringLiteral("▼"), this);
  left_button_ = new QPushButton(QStringLiteral("◀"), this);
  right_button_ = new QPushButton(QStringLiteral("▶"), this);
  stop_button_ = new QPushButton(QStringLiteral("STOP"), this);

  speed_box_ = new QDoubleSpinBox(this);
  speed_box_->setRange(0.05, kMaxLinear);
  speed_box_->setSingleStep(0.05);
  speed_box_->setValue(0.15);
  speed_box_->setSuffix(QStringLiteral(" m/s"));
  turn_box_ = new QDoubleSpinBox(this);
  turn_box_->setRange(0.2, kMaxAngular);
  turn_box_->setSingleStep(0.1);
  turn_box_->setValue(1.0);
  turn_box_->setSuffix(QStringLiteral(" rad/s"));

  status_label_ = new QLabel(QStringLiteral("Teleop off — Nav2 has control"), this);

  auto * pad = new QGridLayout();
  pad->addWidget(forward_button_, 0, 1);
  pad->addWidget(left_button_, 1, 0);
  pad->addWidget(stop_button_, 1, 1);
  pad->addWidget(right_button_, 1, 2);
  pad->addWidget(backward_button_, 2, 1);

  auto * speeds = new QHBoxLayout();
  speeds->addWidget(new QLabel(QStringLiteral("Speed"), this));
  speeds->addWidget(speed_box_);
  speeds->addWidget(new QLabel(QStringLiteral("Turn"), this));
  speeds->addWidget(turn_box_);
  speeds->addStretch();

  auto * layout = new QVBoxLayout();
  layout->addWidget(enable_button_);
  layout->addLayout(pad);
  layout->addLayout(speeds);
  layout->addWidget(status_label_);
  setLayout(layout);

  // Clicking anywhere on the panel focuses it so W/A/S/D reach
  // keyPressEvent; the drive buttons must not swallow that focus.
  setFocusPolicy(Qt::ClickFocus);
  for (auto * b : {forward_button_, backward_button_, left_button_, right_button_, stop_button_}) {
    b->setFocusPolicy(Qt::NoFocus);
    b->setEnabled(false);
  }
  enable_button_->setFocusPolicy(Qt::NoFocus);
  speed_box_->setFocusPolicy(Qt::ClickFocus);
  turn_box_->setFocusPolicy(Qt::ClickFocus);

  connect(enable_button_, &QPushButton::toggled, this, &TeleopPanel::toggleEnabled);

  // pressed/released (not clicked) so holding a button keeps driving.
  connect(forward_button_, &QPushButton::pressed, this, [this]() {setDirection(1.0, button_th_);});
  connect(forward_button_, &QPushButton::released, this, [this]() {setDirection(0.0, button_th_);});
  connect(backward_button_, &QPushButton::pressed, this, [this]() {setDirection(-1.0, button_th_);});
  connect(backward_button_, &QPushButton::released, this, [this]() {setDirection(0.0, button_th_);});
  connect(left_button_, &QPushButton::pressed, this, [this]() {setDirection(button_x_, 1.0);});
  connect(left_button_, &QPushButton::released, this, [this]() {setDirection(button_x_, 0.0);});
  connect(right_button_, &QPushButton::pressed, this, [this]() {setDirection(button_x_, -1.0);});
  connect(right_button_, &QPushButton::released, this, [this]() {setDirection(button_x_, 0.0);});
  connect(stop_button_, &QPushButton::pressed, this, &TeleopPanel::stopNow);

  // 20 Hz command stream, matching the terminal teleop and well inside the
  // cmd_mux 0.4 s staleness window.
  publish_timer_ = new QTimer(this);
  publish_timer_->setInterval(50);
  connect(publish_timer_, &QTimer::timeout, this, &TeleopPanel::publishTick);
}

void TeleopPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_teleop", 10);
}

void TeleopPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Speed", speed_box_->value());
  config.mapSetValue("Turn", turn_box_->value());
}

void TeleopPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  float value = 0.0f;
  if (config.mapGetFloat("Speed", &value)) {
    speed_box_->setValue(value);
  }
  if (config.mapGetFloat("Turn", &value)) {
    turn_box_->setValue(value);
  }
}

void TeleopPanel::toggleEnabled(bool checked)
{
  enable_button_->setText(checked ? QStringLiteral("Teleop ON") : QStringLiteral("Teleop OFF"));
  for (auto * b : {forward_button_, backward_button_, left_button_, right_button_, stop_button_}) {
    b->setEnabled(checked);
  }
  key_x_ = key_th_ = button_x_ = button_th_ = 0.0;
  if (checked) {
    publish_timer_->start();
    setFocus();
    status_label_->setText(
      QStringLiteral("Teleop on — hold W/A/S/D or the buttons (click panel first)"));
  } else {
    // One last zero so the robot stops even mid-drive, then go silent;
    // after the mux timeout Nav2 is back in control.
    if (publisher_) {
      publisher_->publish(geometry_msgs::msg::Twist());
    }
    publish_timer_->stop();
    bursting_ = false;
    status_label_->setText(QStringLiteral("Teleop off — Nav2 has control"));
  }
}

void TeleopPanel::setDirection(double x_dir, double th_dir)
{
  button_x_ = x_dir;
  button_th_ = th_dir;
}

void TeleopPanel::stopNow()
{
  key_x_ = key_th_ = button_x_ = button_th_ = 0.0;
  // Stream zeros for the burst window so the mux latches the stop even if
  // Nav2 is mid-goal.
  bursting_ = true;
  zero_burst_until_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
}

bool TeleopPanel::handleDriveKey(int key, bool pressed)
{
  const double value = pressed ? 1.0 : 0.0;
  switch (key) {
    case Qt::Key_W:
    case Qt::Key_Up:
      key_x_ = value;
      return true;
    case Qt::Key_S:
    case Qt::Key_Down:
      key_x_ = -value;
      return true;
    case Qt::Key_A:
    case Qt::Key_Left:
      key_th_ = value;
      return true;
    case Qt::Key_D:
    case Qt::Key_Right:
      key_th_ = -value;
      return true;
    case Qt::Key_Space:
      if (pressed) {
        stopNow();
      }
      return true;
    default:
      return false;
  }
}

void TeleopPanel::keyPressEvent(QKeyEvent * event)
{
  // Auto-repeat events would fight the hold/release model.
  if (enable_button_->isChecked() && !event->isAutoRepeat() &&
    handleDriveKey(event->key(), true))
  {
    return;
  }
  rviz_common::Panel::keyPressEvent(event);
}

void TeleopPanel::keyReleaseEvent(QKeyEvent * event)
{
  if (enable_button_->isChecked() && !event->isAutoRepeat() &&
    handleDriveKey(event->key(), false))
  {
    return;
  }
  rviz_common::Panel::keyReleaseEvent(event);
}

void TeleopPanel::focusOutEvent(QFocusEvent * event)
{
  // Treat focus loss as "all keys released" so the robot cannot keep
  // driving on a stuck direction.
  key_x_ = key_th_ = 0.0;
  rviz_common::Panel::focusOutEvent(event);
}

void TeleopPanel::publishTick()
{
  if (!publisher_) {
    return;
  }
  const double x_dir = (key_x_ != 0.0) ? key_x_ : button_x_;
  const double th_dir = (key_th_ != 0.0) ? key_th_ : button_th_;

  if (x_dir != 0.0 || th_dir != 0.0) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = x_dir * speed_box_->value();
    twist.angular.z = th_dir * turn_box_->value();
    publisher_->publish(twist);
    // Arm the stop burst for when everything is released.
    bursting_ = true;
    zero_burst_until_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    status_label_->setText(QStringLiteral("Driving: %1 m/s, %2 rad/s")
      .arg(twist.linear.x, 0, 'f', 2).arg(twist.angular.z, 0, 'f', 2));
    return;
  }

  if (bursting_) {
    // Inputs released: stream zeros briefly, then go silent so the mux
    // times this source out and Nav2 regains control.
    if (std::chrono::steady_clock::now() < zero_burst_until_) {
      publisher_->publish(geometry_msgs::msg::Twist());
    } else {
      bursting_ = false;
      status_label_->setText(
        QStringLiteral("Teleop on (idle) — Nav2 has control until you drive"));
    }
  }
}

}  // namespace mdetect_rviz_plugins

PLUGINLIB_EXPORT_CLASS(mdetect_rviz_plugins::TeleopPanel, rviz_common::Panel)
