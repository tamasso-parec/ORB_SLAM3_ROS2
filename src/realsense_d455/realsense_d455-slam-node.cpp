#include "realsense_d455-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RealsenseD455SlamNode::RealsenseD455SlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/color/image_raw");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/aligned_depth_to_color/image_raw");
    localBApublisher_ = this->create_publisher<std_msgs::msg::String>("/localBA", 10);

    
    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose_with_covariance", 10);

    point_cloud_pub_ = this->create_publisher<uncertain_pointcloud_msgs::msg::UncertainPointCloud>("/slam/uncertain_point_cloud", 10);
    
    // TF2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    syncExact = std::make_shared<message_filters::Synchronizer<exact_sync_policy> >(exact_sync_policy(10), *rgb_sub, *depth_sub);

    syncExact->registerCallback(&RealsenseD455SlamNode::GrabRGBD, this);


}

RealsenseD455SlamNode::~RealsenseD455SlamNode()
{

    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

}

void RealsenseD455SlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));

    std::map<int, poseCov_t> pCovs;
    std::map<int, landmarkCov_t> lCovs;
    std::map<int, Sophus::SE3f> ps;
    std::map<int, Eigen::Vector3f> ls;


    // Check if local BA was performed
    if(m_SLAM->GetCovariances(pCovs, lCovs))
    {
        m_SLAM->getLocalPosesAndLandmarks( ps,  mlocalLandmarks);

        Sophus::SE3f latestPose = ps.rbegin()->second;
        Eigen::Matrix<float, 6, 6> latestCov = pCovs.rbegin()->second;

        publishPoseWithCovariance(latestPose, latestCov);
        std_msgs::msg::String msg;
        msg.data = "Local BA performed";
        localBApublisher_->publish(msg);

        publishLandmarks();

        m_SLAM->reset_new_lba_2_publish_flag();
    }
}

void RealsenseD455SlamNode::publishPoseWithCovariance(const Sophus::SE3f &Tcw, const Eigen::Matrix<float, 6, 6> &covariance) 
{

        // Create PoseWithCovarianceStamped message
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";  // Adjust per your TF tree

        // Extract translation
        Eigen::Vector3f t = Tcw.translation();
        pose_msg.pose.pose.position.x = t.x();
        pose_msg.pose.pose.position.y = t.y();
        pose_msg.pose.pose.position.z = t.z();

        // Convert rotation matrix to quaternion
        Eigen::Quaternionf q(Tcw.rotationMatrix());
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();

        // Convert 6x6 Eigen covariance matrix to ROS array format
        for (int i = 0; i < 6; ++i) 
        {
            for (int j = 0; j < 6; ++j) 
            {
                pose_msg.pose.covariance[i * 6 + j] = covariance(i, j);
            }
        }

        // Publish pose with covariance
        pose_cov_pub_->publish(pose_msg);

        // Also publish TF transform
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = this->now();
        transform_msg.header.frame_id = "map";
        transform_msg.child_frame_id = "base_link";

        transform_msg.transform.translation.x = t.x();
        transform_msg.transform.translation.y = t.y();
        transform_msg.transform.translation.z = t.z();

        transform_msg.transform.rotation.x = q.x();
        transform_msg.transform.rotation.y = q.y();
        transform_msg.transform.rotation.z = q.z();
        transform_msg.transform.rotation.w = q.w();

        // Send TF transform
        tf_broadcaster_->sendTransform(transform_msg);
    }


void RealsenseD455SlamNode::publishLandmarks()
{
    // Create UncertainPointCloud message
    uncertain_pointcloud_msgs::msg::UncertainPointCloud point_cloud_msg;
    point_cloud_msg.header.stamp = this->now();
    point_cloud_msg.header.frame_id = "map";  // Adjust per your TF tree

    point_cloud_msg.points.reserve(mlocalLandmarks.size());

    // Iterate over all landmarks
    for (const auto& landmark : mlocalLandmarks) 
    {
        // Create PointWithCovariance message
        uncertain_pointcloud_msgs::msg::UncertainPoint point_msg;

        // Extract landmark position
        Eigen::Vector3f p = landmark.second;
        point_msg.x = p.x();
        point_msg.y = p.y();
        point_msg.z = p.z();
        
        // Extract landmark covariance
        Eigen::Matrix<float, 3, 3>& cov = landmarkCovs[landmark.first];
        
        point_msg.covariance[0] = cov(0, 0);
        point_msg.covariance[1] = cov(0, 1);
        point_msg.covariance[2] = cov(0, 2);
        point_msg.covariance[3] = cov(1, 1);
        point_msg.covariance[4] = cov(1, 2);
        point_msg.covariance[5] = cov(2, 2);

        // Add point to point cloud
        point_cloud_msg.points.push_back(point_msg);

        
    }

    // Publish point cloud
    point_cloud_pub_->publish(point_cloud_msg);
}