
#include <cobot_demo/demo_panel.h>

namespace cobot_demo
{
DemoPanel::DemoPanel(QWidget * parent) 
    : rviz_common::Panel(parent), 
      ui_form_(new Ui::Form()),
      nh_{ std::make_shared<rclcpp::Node>("demo_panel") },
      rc_()
{
    ui_form_->setupUi(this);
    // 
    connect(ui_form_->largeDriveAutopushButton, SIGNAL(clicked(bool)), this, SLOT(moveToTarget(bool)));
    connect(ui_form_->smallDriveAutopushButton, SIGNAL(clicked(bool)), this, SLOT(moveToTarget(bool)));

    // Gripper
    connect(ui_form_->openGripperPushButton, SIGNAL(clicked(bool)), this, SLOT(openGripper(bool)));
    connect(ui_form_->closeGripperPushButton, SIGNAL(clicked(bool)), this, SLOT(closeGripper(bool)));
    connect(ui_form_->gripperSlider, SIGNAL(sliderReleased()), this, SLOT(controlGripperJaws()));
    
    // Predefined robot poses
    connect(ui_form_->centerLeaningPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->centerSeatedPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->centerStandingPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->leftLeaningPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->leftSeatedPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->leftStandingPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->rightLeaningPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->rightStandingPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->rightSeatedPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->wallPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->windowPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->preset1PushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));

    // Joint sliders
    connect(ui_form_->joint1Slider, SIGNAL(sliderReleased()), this, SLOT(jointSliderCallback()));
    connect(ui_form_->joint2Slider, SIGNAL(sliderReleased()), this, SLOT(jointSliderCallback()));
    connect(ui_form_->joint3Slider, SIGNAL(sliderReleased()), this, SLOT(jointSliderCallback()));
    connect(ui_form_->joint4Slider, SIGNAL(sliderReleased()), this, SLOT(jointSliderCallback()));
    connect(ui_form_->joint5Slider, SIGNAL(sliderReleased()), this, SLOT(jointSliderCallback()));
    connect(ui_form_->joint6Slider, SIGNAL(sliderReleased()), this, SLOT(jointSliderCallback()));

    // nh_ = std::make_shared<rclcpp::Node>("_", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    connect(ui_form_->gripperNegativePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint1NegativePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint2NegativePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint3NegativePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint4NegativePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint5NegativePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint6NegativePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->gripperPositivePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint1PositivePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint2PositivePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint3PositivePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint4PositivePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint5PositivePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
    connect(ui_form_->joint6PositivePushButton, SIGNAL(clicked(bool)), this, SLOT(jointPushButtonCallback(bool)));
}

void DemoPanel::onInitialize()
{
    nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
          "joint_states", 10, std::bind(&DemoPanel::jointStateCallback, this, std::placeholders::_1));

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(nh_->get_clock());

    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    drive_holder_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("drive_holder_mesh", 1);

}


DemoPanel::~DemoPanel()
{

}

void DemoPanel::publishMesh()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "aruco_36";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "hdd_caddy";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://cobot_demo/urdf/meshes/hdd_caddy.dae";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0254;
    marker.pose.position.y = 0.0254;
    marker.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0*(M_PI/180.0), 0*(M_PI/180.0), 90.0*(M_PI/180.0));
    q.normalize();
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q);
    marker.pose.orientation = msg_quat;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.1f;
    marker.color.g = 0.1f;
    marker.color.b = 0.1f;
    marker.color.a = 1.0;
    
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
    drive_holder_pub_->publish(marker);
}


void DemoPanel::jointStateCallback(const sensor_msgs::msg::JointState & msg)
{
    joint_msg_ = msg;
    publishMesh();
    for (int i = 0; i < msg.name.size(); ++i)
    {
        if(msg.name[i]=="gripper_controller")
        {
            ui_form_->gripperAngleLabel->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));
        }
        if(msg.name[i]=="joint1")
        {
            ui_form_->joint1Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));
        }
        if(msg.name[i]=="joint2")
        {
            ui_form_->joint2Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));
        }
        if(msg.name[i]=="joint3")
        {
            ui_form_->joint3Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));
        }
        if(msg.name[i]=="joint4")
        {
            ui_form_->joint4Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));
        }
        if(msg.name[i]=="joint5")
        {   
            ui_form_->joint5Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));
        }
        if(msg.name[i]=="joint6")
        {
            ui_form_->joint6Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));
        }
    }

    geometry_msgs::msg::TransformStamped t;
    try {
        // Gripper pose
        t = tf_buffer_->lookupTransform("base_link", "gripper", tf2::TimePointZero);
        ui_form_->xPoseLabel->setText(QString::number(t.transform.translation.x*1000.0, 'f', 1));
        ui_form_->yPoseLabel->setText(QString::number(t.transform.translation.y*1000.0, 'f', 1));
        ui_form_->zPoseLabel->setText(QString::number(t.transform.translation.z*1000.0, 'f', 1));
        tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ui_form_->rollPoseLabel->setText(QString::number(roll*(180.0/M_PI), 'f', 1));
        ui_form_->pitchPoseLabel->setText(QString::number(pitch*(180.0/M_PI), 'f', 1));
        ui_form_->yawPoseLabel->setText(QString::number(yaw*(180.0/M_PI), 'f', 1));

        // Gripper width
        t = tf_buffer_->lookupTransform("gripper", "gripper_right1", tf2::TimePointZero);
        ui_form_->gripperWidthLabel->setText(QString::number(t.transform.translation.x*1000.0, 'f', 1));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(nh_->get_logger(), "Could not transform %s to %s: %s", "base_link", "gripper", ex.what());
        return;
    }
}

void DemoPanel::updateJointSliders()
{
    for (int i = 0; i < joint_msg_.name.size(); ++i)
    {
        if(joint_msg_.name[i]=="gripper_controller")
        {
            ui_form_->gripperSlider->setValue(joint_msg_.position[i]*(180.0/M_PI));

        }
        if(joint_msg_.name[i]=="joint1")
        {
            ui_form_->joint1Slider->setValue(joint_msg_.position[i]*(180.0/M_PI)); // Set the slider's value
        }
        if(joint_msg_.name[i]=="joint2")
        {
            ui_form_->joint2Slider->setValue(joint_msg_.position[i]*(180.0/M_PI));
        }
        if(joint_msg_.name[i]=="joint3")
        {
            ui_form_->joint3Slider->setValue(joint_msg_.position[i]*(180.0/M_PI));
        }
        if(joint_msg_.name[i]=="joint4")
        {
            ui_form_->joint4Slider->setValue(joint_msg_.position[i]*(180.0/M_PI));
        }
        if(joint_msg_.name[i]=="joint5")
        {   
            ui_form_->joint5Slider->setValue(joint_msg_.position[i]*(180.0/M_PI));
        }
        if(joint_msg_.name[i]=="joint6")
        {
            ui_form_->joint6Slider->setValue(joint_msg_.position[i]*(180.0/M_PI));
        }
    }
}

void DemoPanel::moveToTarget(bool clicked)
{
    std::string target_frame;
    if (qobject_cast<QPushButton*>(sender()) == ui_form_->largeDriveAutopushButton)
    {
        target_frame = "large_drive";
    } 
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->smallDriveAutopushButton)
    {
        target_frame = "small_drive";
    } 
    std::thread t = std::thread([this, target_frame](){
        geometry_msgs::msg::Pose target_pose_approach;
        geometry_msgs::msg::Pose target_pose;
        rc_.getTargetPose(target_pose_approach, target_frame+"_approach");
        rc_.getTargetPose(target_pose, target_frame);

        rc_.addCollisionMesh();        

        rc_.planToCartesianPose(target_pose_approach);
        rclcpp::sleep_for(std::chrono::milliseconds(1500));
        rc_.executeTrajectory();
        updateJointSliders();

        rclcpp::sleep_for(std::chrono::milliseconds(1500));
        // rc_.planToCartesianPose(target_pose);
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);
        rc_.planCartesianPath(waypoints);
        
        rclcpp::sleep_for(std::chrono::milliseconds(1500));
        rc_.executeTrajectory();
        updateJointSliders();

        rc_.removeCollisionMesh();
    });

    t.detach();
}

void DemoPanel::openGripper(bool clicked)
{
    ui_form_->gripperSlider->setValue(100);
}

void DemoPanel::closeGripper(bool clicked)
{
    ui_form_->gripperSlider->setValue(0);
}

void DemoPanel::controlGripperJaws()
{
    std::thread t = std::thread([this]()
        {
            rc_.setGripperPosition({(ui_form_->gripperSlider->sliderPosition()*0.63)-63.0});
        });

    t.detach();
}

void DemoPanel::jointSliderCallback()
{
    int joint_idx;
    int joint_value;
    if (qobject_cast<QSlider*>(sender()) == ui_form_->joint1Slider)
    {
        joint_idx = 0;
        joint_value = -1.0 * ui_form_->joint1Slider->sliderPosition();
    }
    else if (qobject_cast<QSlider*>(sender()) == ui_form_->joint2Slider)
    {
        joint_idx = 1;
        joint_value = -1.0 * ui_form_->joint2Slider->sliderPosition();
    }
    else if (qobject_cast<QSlider*>(sender()) == ui_form_->joint3Slider)
    {
        joint_idx = 2;
        joint_value = -1.0 * ui_form_->joint3Slider->sliderPosition();
    }
    else if (qobject_cast<QSlider*>(sender()) == ui_form_->joint4Slider)
    {
        joint_idx = 3;
        joint_value = -1.0 * ui_form_->joint4Slider->sliderPosition();
    }
    else if (qobject_cast<QSlider*>(sender()) == ui_form_->joint5Slider)
    {
        joint_idx = 4;
        joint_value = -1.0 * ui_form_->joint5Slider->sliderPosition();
    }
    else if (qobject_cast<QSlider*>(sender()) == ui_form_->joint6Slider)
    {
        joint_idx = 5;
        joint_value = ui_form_->joint6Slider->sliderPosition();
    }
    std::vector<double> joints;
    rc_.getManipulatorJointPositions(joints);
    joints[joint_idx] = joint_value*1.0;

    std::thread t = std::thread([this, joints]()
    {
        rc_.planToJointPose(joints);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        rc_.executeTrajectory();
    });

    t.detach();
}



void DemoPanel::jointPushButtonCallback(bool clicked)
{
    double joint_delta = 5.0;

    if (qobject_cast<QPushButton*>(sender()) == ui_form_->gripperNegativePushButton)
    {
        ui_form_->gripperSlider->setValue(ui_form_->gripperSlider->value() - joint_delta);
        ui_form_->gripperSlider->setSliderDown(true);
        ui_form_->gripperSlider->setSliderDown(false);
    } 
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint1NegativePushButton)
    {
        ui_form_->joint1Slider->setValue(ui_form_->joint1Slider->value() - joint_delta);
        ui_form_->joint1Slider->setSliderDown(true);
        ui_form_->joint1Slider->setSliderDown(false);
    } 
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint2NegativePushButton)
    {
        ui_form_->joint2Slider->setValue(ui_form_->joint2Slider->value() - joint_delta);
        ui_form_->joint2Slider->setSliderDown(true);
        ui_form_->joint2Slider->setSliderDown(false);
    } 
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint3NegativePushButton)
    {
        ui_form_->joint3Slider->setValue(ui_form_->joint3Slider->value() - joint_delta);
        ui_form_->joint3Slider->setSliderDown(true);
        ui_form_->joint3Slider->setSliderDown(false);
    } 
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint4NegativePushButton)
    {
        ui_form_->joint4Slider->setValue(ui_form_->joint4Slider->value() - joint_delta);
        ui_form_->joint4Slider->setSliderDown(true);
        ui_form_->joint4Slider->setSliderDown(false);
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint5NegativePushButton)
    {
        ui_form_->joint5Slider->setValue(ui_form_->joint5Slider->value() - joint_delta);
        ui_form_->joint5Slider->setSliderDown(true);
        ui_form_->joint5Slider->setSliderDown(false);
    } 
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint6NegativePushButton)
    {
        ui_form_->joint6Slider->setValue(ui_form_->joint6Slider->value() - joint_delta);
        ui_form_->joint6Slider->setSliderDown(true);
        ui_form_->joint6Slider->setSliderDown(false);
    }  
    if (qobject_cast<QPushButton*>(sender()) == ui_form_->gripperPositivePushButton)
    {
        ui_form_->gripperSlider->setValue(ui_form_->gripperSlider->value() + joint_delta);
        ui_form_->gripperSlider->setSliderDown(true);
        ui_form_->gripperSlider->setSliderDown(false);
    } 
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint1PositivePushButton)
    {
        ui_form_->joint1Slider->setValue(ui_form_->joint1Slider->value() + joint_delta);
        ui_form_->joint1Slider->setSliderDown(true);
        ui_form_->joint1Slider->setSliderDown(false);
    } 
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint2PositivePushButton)
    {
        ui_form_->joint2Slider->setValue(ui_form_->joint2Slider->value() + joint_delta);
        ui_form_->joint2Slider->setSliderDown(true);
        ui_form_->joint2Slider->setSliderDown(false);
    } 
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint3PositivePushButton)
    {
        ui_form_->joint3Slider->setValue(ui_form_->joint3Slider->value() + joint_delta);
        ui_form_->joint3Slider->setSliderDown(true);
        ui_form_->joint3Slider->setSliderDown(false);
    } 
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint4PositivePushButton)
    {
        ui_form_->joint4Slider->setValue(ui_form_->joint4Slider->value() + joint_delta);
        ui_form_->joint4Slider->setSliderDown(true);
        ui_form_->joint4Slider->setSliderDown(false);
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint5PositivePushButton)
    {
        ui_form_->joint5Slider->setValue(ui_form_->joint5Slider->value() + joint_delta);
        ui_form_->joint5Slider->setSliderDown(true);
        ui_form_->joint5Slider->setSliderDown(false);
    } 
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->joint6PositivePushButton)
    {
        ui_form_->joint6Slider->setValue(ui_form_->joint6Slider->value() + joint_delta);
        ui_form_->joint6Slider->setSliderDown(true);
        ui_form_->joint6Slider->setSliderDown(false);
    } 
}


void DemoPanel::goToPredefinedPose(bool clicked)
{
    std::vector<double> joints;

    if (qobject_cast<QPushButton*>(sender()) == ui_form_->leftStandingPushButton)
    {
        joints = {69.0, -3.0, -3.0, -41.0, 95.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->centerStandingPushButton)
    {
        joints = {0.0, -3.0, -3.0, -41.0, 95.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->rightStandingPushButton)
    {
        joints = {-40.0, -4.0, -4.0, -42.0, 95.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->leftLeaningPushButton)
    {
        joints = {69.0, 38.0, -117.0, 39.0, 94.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->centerLeaningPushButton)
    {
        joints = {0.0, 40.0, -111.0, 30.0, 94.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->rightLeaningPushButton)
    {
        joints = {-40.0, 40.0, -111.0, 30.0, 94.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->leftSeatedPushButton)
    {
        joints = {40.0, 112.0, -111.0, -25.0, 94.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->centerSeatedPushButton)
    {
        joints = {0.0, 112.0, -111.0, -25.0, 94.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->rightSeatedPushButton)
    {
        joints = {-40.0, 112.0, -111.0, -25.0, 94.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->wallPushButton)
    {
        joints = {109.0, 34.0, -98.0, 45.0, 90.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->windowPushButton)
    {
        joints = {-1.0, -5.0, -5.0, 40.0, 94.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->preset1PushButton)
    {
        joints = {-39.5, 39.4, -112.2, 29.4, 94.4, 0.3};
    }
    else
    {
        RCLCPP_ERROR(nh_->get_logger(), "Invalid pose");
        return;
    }

    std::thread t = std::thread([this, joints]()
    {
        rc_.planToJointPose(joints);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        rc_.executeTrajectory();
        updateJointSliders();
    });

    t.detach();
}
void DemoPanel::releaseObject(bool clicked)
{

}

void DemoPanel::gripObject(bool clicked)
{

}

void DemoPanel::resetScene(bool clicked)
{   

}

void DemoPanel::pick(bool clicked)
{

}

void DemoPanel::save(rviz_common::Config config) const
{
    Panel::save(config);
}

void DemoPanel::load(const rviz_common::Config &conf)
{
    Panel::load(conf);
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cobot_demo::DemoPanel, rviz_common::Panel)

