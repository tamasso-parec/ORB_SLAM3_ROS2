#ifndef __SIMULATION_RGBD_NODE_HPP__
#define __SIMULATION_RGBD_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

#include "px4_msgs/msg/vehicle_odometry.hpp"  


#include "message_filters/subscriber.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/latest_time.h"
#include "message_filters/sync_policies/exact_time.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <uncertain_pointcloud_msgs/msg/uncertain_point_cloud.hpp>

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class FakeRGBDSlamNode : public rclcpp::Node
{
public:

    FakeRGBDSlamNode();

    ~FakeRGBDSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using PclMsg = sensor_msgs::msg::PointCloud2;


    typedef Eigen::Matrix<float, 6,6> poseCov_t;
    typedef Eigen::Matrix<float, 3,3> landmarkCov_t;
    
    // void GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD);
    void PointcloudCallback(const PclMsg::SharedPtr msgPcl);
    // void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const PclMsg::SharedPtr msgD);

    void trackedPoseCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msgPose){
        std::cout << "Pose received" << std::endl;
        // Rotate the quaternion from the ROS frame to the optical frame
        Eigen::Quaternionf q_body_NED(msgPose->q[0], msgPose->q[1], msgPose->q[2], msgPose->q[3]); //R_NED_B

        Eigen::Quaternionf q_NED_to_slam = (Eigen::Quaternionf(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitX())) * Eigen::Quaternionf(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()))); //R_S_NED

        Eigen::Quaternionf q_body_to_slam = q_NED_to_slam * q_body_NED; // R_S_B

        Eigen::Quaternionf q_body_to_cam = q_NED_to_slam; // R_C_B

        Eigen::Quaternionf q_cam_to_slam = q_body_to_slam * Eigen::Quaternionf(q_body_to_cam.toRotationMatrix().transpose()); // R_S_C = R_S_B * R_B_C


        // Eigen::Quaternionf q_cam_to_slam_map =  Eigen::Quaternionf(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ())) * q_ros;
        Twc = Sophus::SE3f(q_cam_to_slam,
                                Eigen::Vector3f(msgPose->position[1],  msgPose->position[2], msgPose->position[0])); 
                                // Eigen::Vector3f(msgPose->position[0],  msgPose->position[1], msgPose->position[2])); 
        mLastPose = Twc.inverse();  // Tcw
        publishTrackedPose();
    }

    void convertPcl2cv(const PclMsg::SharedPtr msgPcl);

    void timerCallback()
    {
        fake_lba_flag = true;
    }
    

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    
    rclcpp::Subscription<PclMsg>::SharedPtr pcl_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr pose_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    // Bundle Adjustment results
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr localBApublisher_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
    rclcpp::Publisher<uncertain_pointcloud_msgs::msg::UncertainPointCloud>::SharedPtr point_cloud_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> orb_to_map_broadcaster_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;


    std::map<int, poseCov_t> poseCovs;
    std::map<int, landmarkCov_t> landmarkCovs;
    std::map<int, Sophus::SE3f> mlocalPoses;
    std::map<int, Eigen::Vector3f> mlocalLandmarks;

    Sophus::SE3f mLastPose, Twc;

    bool fake_lba_flag = false;

    void publishTrackedPose();

    void publishPoseWithCovariance(const Sophus::SE3f &Tcw, const Eigen::Matrix<float, 6, 6> &covariance);

    void publishLandmarks(const PclMsg::SharedPtr msgPcl);



};

#endif