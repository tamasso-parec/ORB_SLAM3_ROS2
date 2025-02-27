#ifndef __REALSENSE_D455_SLAM_NODE_HPP__
#define __REALSENSE_D455_SLAM_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

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

class RealsenseD455SlamNode : public rclcpp::Node
{
public:

    RealsenseD455SlamNode(ORB_SLAM3::System* pSLAM);

    ~RealsenseD455SlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;
    typedef message_filters::sync_policies::LatestTime<ImageMsg, ImageMsg> latest_sync_policy;
    typedef message_filters::sync_policies::ExactTime<ImageMsg, ImageMsg> exact_sync_policy;

    typedef Eigen::Matrix<float, 6,6> poseCov_t;
    typedef Eigen::Matrix<float, 3,3> landmarkCov_t;
    
    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
    std::shared_ptr<message_filters::Synchronizer<latest_sync_policy> > syncLatest;
    std::shared_ptr<message_filters::Synchronizer<exact_sync_policy> > syncExact;
   
    // Bundle Adjustment results
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr localBApublisher_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
    rclcpp::Publisher<uncertain_pointcloud_msgs::msg::UncertainPointCloud>::SharedPtr point_cloud_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::map<int, poseCov_t> poseCovs;
    std::map<int, landmarkCov_t> landmarkCovs;
    std::map<int, Sophus::SE3f> mlocalPoses;
    std::map<int, Eigen::Vector3f> mlocalLandmarks;

    void publishPose();

    void publishPoseWithCovariance(const Sophus::SE3f &Tcw, const Eigen::Matrix<float, 6, 6> &covariance);

    void publishLandmarks();



};

#endif
