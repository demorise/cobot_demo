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

std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

geometry_msgs::msg::Pose applyRotation(const geometry_msgs::msg::Pose& pose, double roll, double pitch, double yaw)
{
  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::convert(pose.orientation, q_orig); 
  q_rot.setRPY(roll*(M_PI/180.0), pitch*(M_PI/180.0), yaw*(M_PI/180.0));
  q_new =  q_orig*q_rot;
  q_new.normalize();
  geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q_new);
  geometry_msgs::msg::Pose pose_new;
  pose_new.position = pose.position;
  pose_new.orientation = msg_quat;
  return pose_new;
}

geometry_msgs::msg::Pose applyTransform(const geometry_msgs::msg::Pose& pose,
                                        const geometry_msgs::msg::TransformStamped& t)
{
  geometry_msgs::msg::PoseStamped pose_in, pose_out;
  pose_in.pose = pose;
  pose_in.header.frame_id = t.child_frame_id;
  tf2::doTransform(pose_in, pose_out, t);
  // std::cout<<"P: "<<pose_out.pose.position.x<<" "<<pose_out.pose.position.y<<" "<<pose_out.pose.position.z<<std::endl;
  return pose_out.pose;
}

bool getTransform(std::string fromFrame, std::string toFrame, geometry_msgs::msg::TransformStamped& t)
{
  try 
  {
    t = tf_buffer_->lookupTransform(fromFrame, toFrame, tf2::TimePointZero, tf2::durationFromSec(2.0));
  } 
  catch (const tf2::TransformException & ex) 
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Could not transform %s to %s: %s",
                toFrame.c_str(), fromFrame.c_str(), ex.what());
    return false;
  }
  // std::cout<<"T: "<<t.transform.translation.x<<" "<<t.transform.translation.y<<" "<<t.transform.translation.z<<std::endl;
  return true;
}

bool getPose(std::string fromFrame, std::string toFrame, geometry_msgs::msg::Pose& pose)
{
  geometry_msgs::msg::TransformStamped t;
  if (getTransform(fromFrame, toFrame, t))
  {
    geometry_msgs::msg::Pose pose_zero;
    pose_zero.orientation.w = 1.0;
    pose = applyTransform(pose_zero, t);
    return true;
  }
  return false;
}

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_to_pose", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));


  // Create a ROS logger
  auto const logger = rclcpp::get_logger("move_to_pose");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });


  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node->get_clock());
  
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



  geometry_msgs::msg::Pose target_pose;

  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("target_pose", 1);

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  // Set the frame ID and timestamp
  marker.header.frame_id = "base_link";
  marker.header.stamp = rclcpp::Clock().now();

  // Set the namespace and id for this marker. This serves to create a unique ID
  marker.id = 0;
  marker.ns = "visualize_pose";

  // Set the marker type to a sphere
  marker.type = visualization_msgs::msg::Marker::SPHERE;;

  // Set the marker action
  marker.action = visualization_msgs::msg::Marker::ADD;

  // target_pose.position.x = -0.035;
  // target_pose.position.y = -0.214;
  // target_pose.position.z = 0.353;
  // target_pose.orientation.w = 1.0;
  // target_pose = applyRotation(target_pose,-106.984, -23.960, -172.926);
  // getPose("base_link", "aruco", target_pose);


  geometry_msgs::msg::Pose pose;
  getPose("gripper", "link6", pose);
  geometry_msgs::msg::TransformStamped t;
  getTransform("base_link", "target", t);
  target_pose = applyTransform(pose, t);
  // std::cout<<"Pose: "<<target_pose.position.x<<" "<<target_pose.position.y<<" "<<target_pose.position.z<<std::endl;


  geometry_msgs::msg::Pose pose_marker;
  getPose("base_link", "target", pose_marker);
  marker.pose = pose_marker;

  // Set the scale of the marker
  marker.scale.x = 0.025;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;

  // Set the color
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0; //alpha

  // Set the lifetime of the marker -- 0 indicates forever
  marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

  // Publish the marker
  marker_pub->publish(marker);
  marker_array.markers.push_back(marker);  


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm_group");

  // Set a target Pose for MOveIt
  move_group_interface.setPoseTarget(target_pose);

  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
    move_group_interface.execute(plan);
  else
    RCLCPP_ERROR(logger, "Planning failed!");

  // Shutdown ROS
  // rclcpp::shutdown();
  spinner.join();
  return 0;
}
