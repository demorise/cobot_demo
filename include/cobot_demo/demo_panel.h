#ifndef DEMO_WIDGET_H
#define DEMO_WIDGET_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <cobot_demo/ui_panel.h>
#include <cmath>
#include <cobot_demo/robot_commander.h>

namespace Ui
{
class Form;
}

namespace cobot_demo
{
class DemoPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  DemoPanel(QWidget* parent = 0);
  ~DemoPanel();
  // void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config &conf) override;

public Q_SLOTS:

protected Q_SLOTS:
  void pick(bool clicked);
  void resetScene(bool clicked);
  void moveToTarget(bool clicked);
  void openGripper(bool clicked);
  void closeGripper(bool clicked);
  void controlGripperJaws(int i);
  void gripObject(bool clicked);
  void releaseObject(bool clicked);
  void goToPredefinedPose(bool clicked);
  void jointSliderCallback(int i);
  void jointPushButtonCallback(bool clicked);

private:
  void onInitialize() override;
  rclcpp::Node::SharedPtr nh_;
  robot_commander::RobotCommander rc_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  void jointStateCallback(const sensor_msgs::msg::JointState & msg) const;

protected:
  Ui::Form* ui_form_;
 
};

}  // end 

#endif  //