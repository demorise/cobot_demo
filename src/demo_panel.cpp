
#include <cobot_demo/demo_panel.h>

namespace cobot_demo
{
    DemoPanel::DemoPanel(QWidget * parent) 
        : rviz_common::Panel(parent), 
          ui_form_(new Ui::Form()),
          rc_()
    {
        ui_form_->setupUi(this);
        
        connect(ui_form_->goToTargetPushButton, SIGNAL(clicked(bool)), this, SLOT(moveToTarget(bool)));
        connect(ui_form_->goToPlayPosePushButton, SIGNAL(clicked(bool)), this, SLOT(moveToPlayPose(bool)));

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

