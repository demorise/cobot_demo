#include "cobot_demo/robot_commander.h"


namespace robot_commander
{

  RobotCommander::RobotCommander()
    :node_(std::make_shared<rclcpp::Node>("robot_commander", 
           rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))),
     move_group_interface_(node_, "arm_group"),
     move_group_interface_gripper_(node_, "gripper_group"),
     planning_scene_interface_(std::make_unique<moveit::planning_interface::PlanningSceneInterface>())
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    thread_ = std::make_unique<std::thread>([&]() 
    {
        executor_->add_node(node_);
        executor_->spin();
        executor_->remove_node(node_);
    });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  RobotCommander::~RobotCommander()
  {
    executor_->cancel();
    thread_->join();
  }

  bool RobotCommander::planToJointPose(const std::vector<double>& joint_positions)
  {
    //TO DO : Assert vector size is 6
    std::vector<double> joint_positions_radians;
    for (double jt : joint_positions)
      joint_positions_radians.push_back(jt*(M_PI/180.0));

    bool within_bounds = move_group_interface_.setJointValueTarget(joint_positions_radians);
    if (!within_bounds)
    {
      RCLCPP_ERROR(node_->get_logger(), "Target joint configuration outside of limits");
      return false;
    }

    move_group_interface_.setMaxVelocityScalingFactor(0.05);
    move_group_interface_.setMaxAccelerationScalingFactor(0.05);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      motion_plan_ = plan;
      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
      return false;
    }
  }

  bool RobotCommander::planToCartesianPose(geometry_msgs::msg::Pose target_pose)
  {
    // Set a target Pose for MoveIt
    move_group_interface_.setPoseTarget(target_pose);

    auto const [success, plan] = [this] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface_.plan(msg));
      return std::make_pair(ok, msg);
    }();

    if (success)
    {
      motion_plan_ = plan;
      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
      return false;
    }
  }

  bool RobotCommander::planCartesianPath(std::vector<geometry_msgs::msg::Pose> waypoints)
  {
    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    motion_plan_.trajectory_ = trajectory;
    return true;
  }

  void RobotCommander::addCollisionMesh()
  {
    // planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor_);
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "aruco_36";
    collision_object.pose.orientation.w = 1.0;
    collision_object.id = "large_drive";

    std::string package_path = ament_index_cpp::get_package_share_directory("cobot_demo");
    std::string mesh_path = package_path + "/urdf/meshes/hdd_caddy.dae";
    shapes::Mesh* c_mesh = shapes::createMeshFromResource("file://" + mesh_path);
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(c_mesh, mesh_msg);
    shape_msgs::msg::Mesh custom_mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    for (size_t i = 0; i < custom_mesh.vertices.size(); ++i)
    {
      custom_mesh.vertices[i].x = custom_mesh.vertices[i].x;
      custom_mesh.vertices[i].y = custom_mesh.vertices[i].y;
      custom_mesh.vertices[i].z = custom_mesh.vertices[i].z;
    }

    collision_object.meshes.push_back(custom_mesh);
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.0254;
    pose.position.y = 0.0254;
    pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0*(M_PI/180.0), 0*(M_PI/180.0), 90.0*(M_PI/180.0));
    q.normalize();
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q);
    pose.orientation = msg_quat; 
    collision_object.mesh_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;
    collision_objects.push_back(collision_object);
    planning_scene_interface_->applyCollisionObjects(collision_objects);
  }

  void RobotCommander::removeCollisionMesh()
  {
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = "large_drive";                
    collision_object.operation = collision_object.REMOVE;
    collision_objects.push_back(collision_object);
    planning_scene_interface_->applyCollisionObjects(collision_objects);
  }

  bool RobotCommander::executeTrajectory()
  {
    move_group_interface_.execute(motion_plan_);
    return true;
  }

  bool RobotCommander::setGripperPosition(const std::vector<double>& joint_positions)
  {
    //TO DO : Assert vector size is 1
    std::vector<double> joint_positions_radians;
    for (double jt : joint_positions)
      joint_positions_radians.push_back(jt*(M_PI/180.0));

    bool within_bounds = move_group_interface_gripper_.setJointValueTarget(joint_positions_radians);
    if (!within_bounds)
    {
      RCLCPP_ERROR(node_->get_logger(), "Target joint configuration for gripper outside of limits");
      return false;
    }

    move_group_interface_gripper_.setMaxVelocityScalingFactor(0.05);
    move_group_interface_gripper_.setMaxAccelerationScalingFactor(0.05);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface_gripper_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      move_group_interface_.execute(plan);
      return true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed for gripper!");
      return false;
    }
  }

  bool RobotCommander::getTargetPose(geometry_msgs::msg::Pose& target_pose, std::string target_frame)
  {
    geometry_msgs::msg::Pose pose;
    getPose("gripper", "link6", pose);
    geometry_msgs::msg::TransformStamped t;
    getTransform("base_link", target_frame, t);
    target_pose = applyTransform(pose, t);
    return true;
  }

  bool RobotCommander::getManipulatorJointPositions(std::vector<double>& joint_positions)
  {
    const moveit::core::JointModelGroup* joint_model_group = 
      move_group_interface_.getCurrentState()->getJointModelGroup("arm_group");
    moveit::core::RobotStatePtr current_state = move_group_interface_.getCurrentState(10);
    current_state->copyJointGroupPositions(joint_model_group, joint_positions);
    std::transform(joint_positions.begin(), joint_positions.end(), joint_positions.begin(), [](double d) -> double { return d * (180.0/M_PI); });
    return true;
  }

  geometry_msgs::msg::Pose RobotCommander::applyRotation(const geometry_msgs::msg::Pose& pose, double roll, double pitch, double yaw)
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

  geometry_msgs::msg::Pose RobotCommander::applyTransform(const geometry_msgs::msg::Pose& pose,
                                          const geometry_msgs::msg::TransformStamped& t)
  {
    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.pose = pose;
    pose_in.header.frame_id = t.child_frame_id;
    tf2::doTransform(pose_in, pose_out, t);
    // std::cout<<"P: "<<pose_out.pose.position.x<<" "<<pose_out.pose.position.y<<" "<<pose_out.pose.position.z<<std::endl;
    return pose_out.pose;
  }

  bool RobotCommander::getTransform(std::string fromFrame, std::string toFrame, geometry_msgs::msg::TransformStamped& t)
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

  bool RobotCommander::getPose(std::string fromFrame, std::string toFrame, geometry_msgs::msg::Pose& pose)
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

}