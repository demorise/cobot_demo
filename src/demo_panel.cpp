
#include <cobot_demo/demo_panel.h>

namespace cobot_demo
{
    DemoPanel::DemoPanel(QWidget * parent) 
        : rviz_common::Panel(parent), 
          ui_form_(new Ui::Form()),
          rc_()
    {
        ui_form_->setupUi(this);
        ui_form_->predefinedPoseComboBox->addItem("Seated Center (Monitor)");
        ui_form_->predefinedPoseComboBox->addItem("Seated Left Limit");
        ui_form_->predefinedPoseComboBox->addItem("Seated Target");
        ui_form_->predefinedPoseComboBox->addItem("Seated Crusher");
        ui_form_->predefinedPoseComboBox->addItem("Standing Target");
        ui_form_->predefinedPoseComboBox->addItem("Standing Desk Overview");
        ui_form_->predefinedPoseComboBox->addItem("Standing Crusher");
        ui_form_->predefinedPoseComboBox->addItem("Leaned in Target");
        ui_form_->predefinedPoseComboBox->addItem("Leaned in TV");
        ui_form_->predefinedPoseComboBox->addItem("Leaned in Keyboard");
        ui_form_->predefinedPoseComboBox->addItem("Leaned in ESC Key");
        
        connect(ui_form_->goToTargetPushButton, SIGNAL(clicked(bool)), this, SLOT(moveToTarget(bool)));
        connect(ui_form_->goToPlayPosePushButton, SIGNAL(clicked(bool)), this, SLOT(moveToPlayPose(bool)));

        connect(ui_form_->openGripperPushButton, SIGNAL(clicked(bool)), this, SLOT(openGripper(bool)));
        connect(ui_form_->closeGripperPushButton, SIGNAL(clicked(bool)), this, SLOT(closeGripper(bool)));

        connect(ui_form_->predefinedPosePushButton, SIGNAL(clicked(bool)), this, SLOT(goToPredefinedPose(bool)));

        connect(ui_form_->gripperSlider, SIGNAL(valueChanged(int)), this, SLOT(controlGripperJaws(int)));

        nh_ = std::make_shared<rclcpp::Node>("_", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    }

    DemoPanel::~DemoPanel()
    {

    }

    void DemoPanel::moveToTarget(bool clicked)
    {
        std::thread t = std::thread([this]()
            {
                geometry_msgs::msg::Pose target_pose;
                rc_.getTargetPose(target_pose);
                rc_.planToCartesianPose(target_pose);
                rclcpp::sleep_for(std::chrono::milliseconds(1500));
                rc_.executeTrajectory();
            });

        t.detach();
    }

    void DemoPanel::moveToPlayPose(bool clicked)
    {
        std::thread t = std::thread([this]()
            {
                rc_.planToJointPose({62.0, 36.0, -43.0, 98.0, -91.0, -29.0});
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


    void DemoPanel::goToPredefinedPose(bool clicked)
    {
        std::vector<double> joints;
        std::string predefinedPose = ui_form_->predefinedPoseComboBox->currentText().toStdString();
        if (predefinedPose == "Seated Center (Monitor)")
        {
            joints = {0.0, 110.0, -110.0, -35.0, 95.0, 0.0};
        }
        else if (predefinedPose == "Seated Left Limit")
        {
            joints = {15.0, 110.0, -110.0, -35.0, 95.0, 0.0};
        }
        else if (predefinedPose == "Seated Target")
        {
            joints = {25.0, 110.0, -110.0, -35.0, 95.0, 0.0};
        }
        else if (predefinedPose == "Seated Crusher")
        {
            joints = {-30.0, 110.0, -110.0, -35.0, 95.0, 0.0};
        }
        else if (predefinedPose == "Standing Target")
        {
            joints = {25.0, 0.0, 0.0, -55.0, 90.0, 0.0};
        }
        else if (predefinedPose == "Standing Desk Overview")
        {
            joints = {0.0, 0.0, 0.0, -55.0, 90.0, 0.0};
        }
        else if (predefinedPose == "Standing Crusher")
        {
            joints = {-35.0, 0.0, 0.0, -55.0, 90.0, 0.0};
        }
        else if (predefinedPose == "Leaned in Target")
        {
            joints = {30.0, 0.0, -90.0, 35.0, 90.0, 0.0};
        }
        else if (predefinedPose == "Leaned in TV")
        {
            joints = {0.0, 0.0, -90.0, 50.0, 90.0, 0.0};
        }
        else if (predefinedPose == "Leaned in Keyboard")
        {
            joints = {0.0, 0.0, -90.0, 0.0, 90.0, 0.0};
        }
        else if (predefinedPose == "Leaned in ESC Key")
        {
            joints = {25.0, 0.0, -135.0, 65.0, 90.0, 0.0};
        }
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

    void DemoPanel::onInitialize()
    {

    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cobot_demo::DemoPanel, rviz_common::Panel)

