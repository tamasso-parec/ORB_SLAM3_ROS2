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

class SimulationRGBDSlamNode : public rclcpp::Node
{
public:

    SimulationRGBDSlamNode(ORB_SLAM3::System* pSLAM);

    ~SimulationRGBDSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using PclMsg = sensor_msgs::msg::PointCloud2;

    typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg> approximate_sync_policy;
    // typedef message_filters::sync_policies::ApproximateTime<ImageMsg, PclMsg> approximate_sync_policy;

    typedef message_filters::sync_policies::LatestTime<ImageMsg, ImageMsg> latest_sync_policy;

    typedef message_filters::sync_policies::ExactTime<ImageMsg, ImageMsg> exact_sync_policy;
    // typedef message_filters::sync_policies::ExactTime<ImageMsg, PclMsg> exact_sync_policy;

    typedef Eigen::Matrix<float, 6,6> poseCov_t;
    typedef Eigen::Matrix<float, 3,3> landmarkCov_t;
    
    void GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD);
    // void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const PclMsg::SharedPtr msgD);

    void convertPcl2cv(const PclMsg::SharedPtr msgPcl);
    

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;
    std::shared_ptr<message_filters::Subscriber<PclMsg> > pcl_sub;


    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;
    std::shared_ptr<message_filters::Synchronizer<latest_sync_policy> > syncLatest;
    std::shared_ptr<message_filters::Synchronizer<exact_sync_policy> > syncExact;
   
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

    Sophus::SE3f mLastPose;

    void publishTrackedPose();

    void publishPoseWithCovariance(const Sophus::SE3f &Tcw, const Eigen::Matrix<float, 6, 6> &covariance);

    void publishLandmarks();



};

#endif