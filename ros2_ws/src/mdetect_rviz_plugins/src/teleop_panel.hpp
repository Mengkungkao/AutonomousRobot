// RViz teleop panel with an on/off toggle.
//
// While enabled AND a drive key (W/A/S/D or arrow keys) or an on-screen
// button is held, the panel streams Twist commands to /cmd_vel_teleop at
// 20 Hz — the cmd_mux's highest-priority source. On release it streams a
// short burst of zero commands (so the mux latches the stop) and then goes
// silent, letting Nav2 take back control. Toggled off, it publishes nothing.
#pragma once

#include <chrono>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace mdetect_rviz_plugins
{

class TeleopPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TeleopPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

protected:
  // Keyboard drive (panel must have focus; clicking the panel focuses it).
  void keyPressEvent(QKeyEvent * event) override;
  void keyReleaseEvent(QKeyEvent * event) override;
  // Losing focus or getting hidden counts as releasing every key.
  void focusOutEvent(QFocusEvent * event) override;

private Q_SLOTS:
  void toggleEnabled(bool checked);
  void publishTick();

private:
  void setDirection(double x_dir, double th_dir);
  void stopNow();
  bool handleDriveKey(int key, bool pressed);

  QPushButton * enable_button_ = nullptr;
  QPushButton * forward_button_ = nullptr;
  QPushButton * backward_button_ = nullptr;
  QPushButton * left_button_ = nullptr;
  QPushButton * right_button_ = nullptr;
  QPushButton * stop_button_ = nullptr;
  QDoubleSpinBox * speed_box_ = nullptr;
  QDoubleSpinBox * turn_box_ = nullptr;
  QLabel * status_label_ = nullptr;
  QTimer * publish_timer_ = nullptr;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  // Direction multipliers currently held (-1, 0, +1), scaled by the speed
  // boxes at publish time. key/button states are OR-ed per axis.
  double key_x_ = 0.0;
  double key_th_ = 0.0;
  double button_x_ = 0.0;
  double button_th_ = 0.0;
  // After all inputs release, zeros are streamed until this deadline so the
  // mux latches the stop before the panel goes silent.
  std::chrono::steady_clock::time_point zero_burst_until_;
  bool bursting_ = false;
};

}  // namespace mdetect_rviz_plugins
