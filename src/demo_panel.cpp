
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

    // Ensure joint slider positions match robot's current joint values
    std::vector<double> joint_positions;
    rc_.getManipulatorJointPositions(joint_positions);
    ui_form_->joint1Slider->setValue(joint_positions[0]);
    ui_form_->joint2Slider->setValue(joint_positions[1]);
    ui_form_->joint3Slider->setValue(joint_positions[2]);
    ui_form_->joint4Slider->setValue(joint_positions[3]);
    ui_form_->joint5Slider->setValue(joint_positions[4]);
    ui_form_->joint6Slider->setValue(joint_positions[5]);

    // 
    connect(ui_form_->largeDriveAutopushButton, SIGNAL(clicked(bool)), this, SLOT(moveToTarget(bool)));
    connect(ui_form_->smallDriveAutopushButton, SIGNAL(clicked(bool)), this, SLOT(moveToTarget(bool)));

    // Gripper
    connect(ui_form_->openGripperPushButton, SIGNAL(clicked(bool)), this, SLOT(openGripper(bool)));
    connect(ui_form_->closeGripperPushButton, SIGNAL(clicked(bool)), this, SLOT(closeGripper(bool)));
    connect(ui_form_->gripperSlider, SIGNAL(valueChanged(int)), this, SLOT(controlGripperJaws(int)));
    
    // Predefined robot poses
    connect(ui_form_->centerLeaningPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->centerSeatedPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->centerStandingPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->leftLeaningPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->leftSeatedPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->leftStandingPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->rightLeaningPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->rightStandingPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));
    connect(ui_form_->righttSeatedPushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));

    // Joint sliders
    connect(ui_form_->joint1Slider, SIGNAL(valueChanged(int)), this, SLOT(jointSliderCallback(int)));
    connect(ui_form_->joint2Slider, SIGNAL(valueChanged(int)), this, SLOT(jointSliderCallback(int)));
    connect(ui_form_->joint3Slider, SIGNAL(valueChanged(int)), this, SLOT(jointSliderCallback(int)));
    connect(ui_form_->joint4Slider, SIGNAL(valueChanged(int)), this, SLOT(jointSliderCallback(int)));
    connect(ui_form_->joint5Slider, SIGNAL(valueChanged(int)), this, SLOT(jointSliderCallback(int)));
    connect(ui_form_->joint6Slider, SIGNAL(valueChanged(int)), this, SLOT(jointSliderCallback(int)));

    // nh_ = std::make_shared<rclcpp::Node>("_", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  
}


void DemoPanel::onInitialize()
{
  nh_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&DemoPanel::jointStateCallback, this, std::placeholders::_1));

}


DemoPanel::~DemoPanel()
{

}



void DemoPanel::jointStateCallback(const sensor_msgs::msg::JointState & msg) const
{
    for (int i = 0; i < msg.name.size(); ++i)
    {
        if(msg.name[i]=="gripper_controller")
            ui_form_->gripperAngleLabel->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));
    
        if(msg.name[i]=="joint1")
            ui_form_->joint1Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));

        if(msg.name[i]=="joint2")
            ui_form_->joint2Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));

        if(msg.name[i]=="joint3")
            ui_form_->joint3Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));

        if(msg.name[i]=="joint4")
            ui_form_->joint4Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));

        if(msg.name[i]=="joint5")
            ui_form_->joint5Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));

        if(msg.name[i]=="joint6")
            ui_form_->joint6Label->setText(QString::number(msg.position[i]*(180.0/M_PI), 'f', 1));
    }
}



void DemoPanel::moveToTarget(bool clicked)
{
    std::thread t = std::thread([this](){
        geometry_msgs::msg::Pose target_pose;
        rc_.getTargetPose(target_pose);
        rc_.planToCartesianPose(target_pose);
        rclcpp::sleep_for(std::chrono::milliseconds(1500));
        rc_.executeTrajectory();
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

void DemoPanel::controlGripperJaws(int i)
{
    std::thread t = std::thread([this, i]()
        {
            rc_.setGripperPosition({(i*0.63)-63.0});
        });

    t.detach();
}

void DemoPanel::jointSliderCallback(int i)
{
    int joint_idx;
    if (qobject_cast<QSlider*>(sender()) == ui_form_->joint1Slider)
    {
        joint_idx = 0;
    }
    else if (qobject_cast<QSlider*>(sender()) == ui_form_->joint2Slider)
    {
        joint_idx = 1;
    }
    else if (qobject_cast<QSlider*>(sender()) == ui_form_->joint3Slider)
    {
        joint_idx = 2;
    }
    else if (qobject_cast<QSlider*>(sender()) == ui_form_->joint4Slider)
    {
        joint_idx = 3;
    }
    else if (qobject_cast<QSlider*>(sender()) == ui_form_->joint5Slider)
    {
        joint_idx = 4;
    }
    else if (qobject_cast<QSlider*>(sender()) == ui_form_->joint6Slider)
    {
        joint_idx = 5;
    }
    std::vector<double> joints;
    rc_.getManipulatorJointPositions(joints);
    joints[joint_idx] = i*1.0;

    std::thread t = std::thread([this, joints]()
    {
        rc_.planToJointPose(joints);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        rc_.executeTrajectory();
    });

    t.detach();
}

void DemoPanel::goToPredefinedPose(bool clicked)
{
    std::vector<double> joints;

    if (qobject_cast<QPushButton*>(sender()) == ui_form_->centerSeatedPushButton)//("Seated Center (Monitor)")
    {
        joints = {0.0, 110.0, -110.0, -35.0, 95.0, 0.0};
    }
    else if (qobject_cast<QPushButton*>(sender()) == ui_form_->leftSeatedPushButton)//("Seated Left Limit")
    {
        joints = {15.0, 110.0, -110.0, -35.0, 95.0, 0.0};
    }
    // else if (qobject_cast<QPushButton*>(sender()) == ui_form_.AAA)//("Seated Target")
    // {
    //     joints = {25.0, 110.0, -110.0, -35.0, 95.0, 0.0};
    // }
    // else if (qobject_cast<QPushButton*>(sender()) == ui_form_.AAA)//("Seated Crusher")
    // {
    //     joints = {-30.0, 110.0, -110.0, -35.0, 95.0, 0.0};
    // }
    // else if (qobject_cast<QPushButton*>(sender()) == ui_form_.AAA)//("Standing Target")
    // {
    //     joints = {25.0, 0.0, 0.0, -55.0, 90.0, 0.0};
    // }
    // else if (qobject_cast<QPushButton*>(sender()) == ui_form_.AAA)//("Standing Desk Overview")
    // {
    //     joints = {0.0, 0.0, 0.0, -55.0, 90.0, 0.0};
    // }
    // else if (qobject_cast<QPushButton*>(sender()) == ui_form_.AAA)//("Standing Crusher")
    // {
    //     joints = {-35.0, 0.0, 0.0, -55.0, 90.0, 0.0};
    // }
    // else if (qobject_cast<QPushButton*>(sender()) == ui_form_.AAA)//("Leaned in Target")
    // {
    //     joints = {30.0, 0.0, -90.0, 35.0, 90.0, 0.0};
    // }
    // else if (qobject_cast<QPushButton*>(sender()) == ui_form_.AAA)//("Leaned in TV")
    // {
    //     joints = {0.0, 0.0, -90.0, 50.0, 90.0, 0.0};
    // }
    // else if (qobject_cast<QPushButton*>(sender()) == ui_form_.AAA)//("Leaned in Keyboard")
    // {
    //     joints = {0.0, 0.0, -90.0, 0.0, 90.0, 0.0};
    // }
    // else if (qobject_cast<QPushButton*>(sender()) == ui_form_.AAA)//("Leaned in ESC Key")
    // {
    //     joints = {25.0, 0.0, -135.0, 65.0, 90.0, 0.0};
    // }
    else
    {
        joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    std::thread t = std::thread([this, joints]()
    {
        rc_.planToJointPose(joints);
        rclcpp::sleep_for(std::chrono::milliseconds(1500));
        rc_.executeTrajectory();
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

