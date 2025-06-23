#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry> 
#include <opencv2/core/eigen.hpp>


class ArucoNode {
public:
  ArucoNode() : node_{ std::make_shared<rclcpp::Node>("aruco_node")}
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    thread_ = std::make_unique<std::thread>([&]() 
    {
        executor_->add_node(node_);
        executor_->spin();
        executor_->remove_node(node_);
    });

    std::string camera_topic = "/image_raw";
    std::string output_topic = "/aruco_image";
    marker_size_ = 0.0254*2.0;

    // Initialize ArUco detector
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    detector_params_ = cv::aruco::DetectorParameters::create();

    image_transport::ImageTransport it(node_);

    subscription_ = it.subscribe(camera_topic, 10, &ArucoNode::imageCallback, this);

    // Publish the image with axes
    publisher_ = it.advertise(output_topic, 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    // Camera calibration parameters (replace with your own)
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
        374.218296, 0.000000, 327.269541,
        0.000000, 372.491787, 255.165328,
        0.0, 0.0, 1.0);
    dist_coeffs_ = (cv::Mat_<double>(1, 5) << -0.212865, 0.052493, 0.000996, 0.001153, 0.000000);
  }

  ~ArucoNode()
  {
    executor_->cancel();
    thread_->join();
  }

private:
    void broadcastArucoTF(cv::Vec3d tvec, cv::Vec3d rvec, int marker_id)
    {
        // Translation
        Eigen::Vector3d eigen_trans;
        cv::cv2eigen(tvec, eigen_trans);

        // Rotatiom
        cv::Mat cv_rot_mat;
        cv::Rodrigues(rvec, cv_rot_mat);
        Eigen::Matrix3d eigen_rot_mat;
        cv::cv2eigen(cv_rot_mat, eigen_rot_mat);

        // Homogenous transformation matrix
        Eigen::Isometry3d T; 
        T.matrix().setIdentity(); 
        T.matrix().block<3,3>(0,0) = eigen_rot_mat;
        T.matrix().block<3,1>(0,3) = eigen_trans;

        // ROS Stamped transform
        geometry_msgs::msg::TransformStamped t_aruco = tf2::eigenToTransform (T);
        t_aruco.header.stamp = node_->get_clock()->now();
        t_aruco.header.frame_id = "camera";
        t_aruco.child_frame_id = "aruco";//+ std::to_string(i);
        // std::cout<<T.matrix()<<std::endl;
        if (marker_id==36)
        {
            t_aruco.child_frame_id = "aruco_36";
        }
        else if(marker_id==165)
        {
            t_aruco.child_frame_id = "aruco_165";
        }
        else
        {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Invalid aruco marker ID: " << marker_id);
            return;
        }
        tf_broadcaster_->sendTransform(t_aruco);

        geometry_msgs::msg::TransformStamped t_target;
        tf2::Quaternion q;

        t_target.header.frame_id = "aruco";
        t_target.child_frame_id = "target";
        t_target.transform.translation.x = -0.019;
        t_target.transform.translation.y = 0.013;
        t_target.transform.translation.z = -0.032;
        q.setRPY(170.743*(M_PI/180.0), 15.935*(M_PI/180.0), 98.021*(M_PI/180.0));
  
        if (marker_id==36)
        {
            t_target.header.frame_id = "aruco_36";
            t_target.child_frame_id = "large_drive";
            t_target.transform.translation.x = 0;
            t_target.transform.translation.y = 0.06985;
            t_target.transform.translation.z = 0.0381*0.2;
            q.setRPY(-90.0*(M_PI/180.0), 0*(M_PI/180.0), 0*(M_PI/180.0));
        }
        else if(marker_id==165)
        {
            t_target.header.frame_id = "aruco_165";
            t_target.child_frame_id = "small_drive";
            t_target.transform.translation.x = 0;
            t_target.transform.translation.y = 0;
            t_target.transform.translation.z = 0;
            q.setRPY(0*(M_PI/180.0), 0*(M_PI/180.0), 0*(M_PI/180.0));
        }
        q.normalize();
        geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q);
        t_target.header.stamp = node_->get_clock()->now();
        t_target.transform.rotation = msg_quat;
        tf_broadcaster_->sendTransform(t_target);


        // approach target
        t_target.header.frame_id = t_target.child_frame_id;
        t_target.child_frame_id = t_target.child_frame_id + "_approach";
        t_target.transform.translation.x = 0;
        t_target.transform.translation.y = 0;
        t_target.transform.translation.z = -0.03-0.0508;
        q.setRPY(0*(M_PI/180.0), 0*(M_PI/180.0), 0*(M_PI/180.0));
        q.normalize();
        msg_quat = tf2::toMsg(q);
        t_target.header.stamp = node_->get_clock()->now();
        t_target.transform.rotation = msg_quat;
        tf_broadcaster_->sendTransform(t_target);


        // retract
        t_target.child_frame_id = t_target.header.frame_id + "_retract";
        t_target.transform.translation.x = 0;
        t_target.transform.translation.y = -0.05;
        t_target.transform.translation.z = 0;
        q.setRPY(0*(M_PI/180.0), 0*(M_PI/180.0), 0*(M_PI/180.0));
        q.normalize();
        msg_quat = tf2::toMsg(q);
        t_target.header.stamp = node_->get_clock()->now();
        t_target.transform.rotation = msg_quat;
        tf_broadcaster_->sendTransform(t_target);
    };

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        cv_bridge::CvImagePtr cv_ptr;
        try 
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } 
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<cv::Vec3d> rvecs, tvecs;

        // Detect ArUco markers
        cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, detector_params_);

        // Estimate pose if markers are detected
        if (marker_ids.size() > 0) 
        {
            cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            // Draw axes for each marker
            for (size_t i = 0; i < marker_ids.size(); ++i) 
            {
                cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_size_ * 0.5);
                broadcastArucoTF(tvecs[i], rvecs[i], marker_ids[i]);
            }
        }

        // Publish the modified image
        cv_ptr->image = image;
        publisher_.publish(cv_ptr->toImageMsg());
    }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr executor_;
  image_transport::Subscriber subscription_;
  image_transport::Publisher publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::Mat camera_matrix_, dist_coeffs_;
  double marker_size_;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    std::unique_ptr<ArucoNode> aruco_node =  std::make_unique <ArucoNode>();

    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok())
        loop_rate.sleep();

    // rclcpp::spin(std::make_shared<ArucoNode>());
    rclcpp::shutdown();
    return 0;
}