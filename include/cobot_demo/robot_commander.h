#ifndef ROBOT_COMMANDER_H
#define ROBOT_COMMANDER_H

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <memory>
#include <thread>

namespace robot_commander
{

class RobotCommander
{
public:
  RobotCommander();

  ~RobotCommander();

  bool planToJointPose(const std::vector<double>& joint_positions);

  bool planToCartesianPose(geometry_msgs::msg::Pose target_pose);

  bool executeTrajectory();

  bool setGripperPosition(const std::vector<double>& joint_positions);

  bool getTargetPose(geometry_msgs::msg::Pose& target_pose);

private:
  geometry_msgs::msg::Pose applyRotation(const geometry_msgs::msg::Pose& pose, double roll, double pitch, double yaw);

  geometry_msgs::msg::Pose applyTransform(const geometry_msgs::msg::Pose& pose,
                                        const geometry_msgs::msg::TransformStamped& t);

  bool getTransform(std::string fromFrame, std::string toFrame, geometry_msgs::msg::TransformStamped& t);

  bool getPose(std::string fromFrame, std::string toFrame, geometry_msgs::msg::Pose& pose);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Node::SharedPtr node_;

  moveit::planning_interface::MoveGroupInterface move_group_interface_;

  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper_;

  moveit::planning_interface::MoveGroupInterface::Plan motion_plan_;

  std::unique_ptr<std::thread> thread_;

  rclcpp::Executor::SharedPtr executor_;
};



}
#endif