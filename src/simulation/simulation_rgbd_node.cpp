
#include "simulation_rgbd_node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

SimulationRGBDSlamNode::SimulationRGBDSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),

    m_SLAM(pSLAM)
{

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/rgbd_camera/image");
    
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/rgbd_camera/depth_image");

    // pcl_sub   = std::make_shared<message_filters::Subscriber<PclMsg> >(this, "/rgbd_camera/points");

    localBApublisher_ = this->create_publisher<std_msgs::msg::String>("/localBA", 10);

    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose_with_covariance", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/tracked_pose", 10);

    point_cloud_pub_ = this->create_publisher<uncertain_pointcloud_msgs::msg::UncertainPointCloud>("/slam/uncertain_point_cloud", 10);
    
    // TF2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    orb_to_map_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    syncExact = std::make_shared<message_filters::Synchronizer<exact_sync_policy> >(exact_sync_policy(10), *rgb_sub, *depth_sub);


    // syncExact = std::make_shared<message_filters::Synchronizer<exact_sync_policy> >(exact_sync_policy(10), *rgb_sub, *pcl_sub);
    // syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *pcl_sub);

    syncExact->registerCallback(&SimulationRGBDSlamNode::GrabRGBD, this);
    // syncApproximate->registerCallback(&SimulationRGBDSlamNode::GrabRGBD, this);


}

SimulationRGBDSlamNode::~SimulationRGBDSlamNode()
{

    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

}

void SimulationRGBDSlamNode::GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
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


    // Convert the PointCloud2 message to cv::Mat

    mLastPose = m_SLAM->TrackRGBD(cv_ptrRGB->image, 1000*cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));

    publishTrackedPose();

    std::map<int, poseCov_t> pCovs;
    std::map<int, landmarkCov_t> lCovs;
    std::map<int, Sophus::SE3f> ps;
    std::map<int, Eigen::Vector3f> ls;


    // Check if local BA was performed
    if(m_SLAM->GetCovariances(pCovs, lCovs))
    {
        m_SLAM->getLocalPosesAndLandmarks( ps, mlocalLandmarks);

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

void SimulationRGBDSlamNode::convertPcl2cv(const PclMsg::SharedPtr cloud)
{
    
    // Initialize an OpenCV Mat with the same height and width as the PointCloud2
    cv::Mat img(cloud->height, cloud->width, CV_32FC1, cv::Scalar(0));

    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

    for (size_t i{0}; i < cloud->width * cloud->height; ++i, ++iter_z)
    {
        if (std::isnan(*iter_z))
        {
            img.at<float>(i / cloud->width, i % cloud->width) = 0;
        }
        else
        {
            img.at<float>(i / cloud->width, i % cloud->width) = *iter_z;
        }
    }

    // // Iterate through the organized point cloud
    // for (size_t row = 0; row < cloud->height; ++row)
    // {
    //     for (size_t col = 0; col < cloud->width; ++col)
    //     {
    //         const auto& point = cloud->data[row * cloud->width + col];
    //         size_t u = col;
    //         size_t v = cloud->height-row-1; //image coordinates are flipped
    //         // Check if the point is valid (not NaN)
    //         // if (pcl::isFinite(point))
    //         {
    //             // Calculate the range value (distance from the origin)
    //             // float range = point.z;
                
    //             img.at<float>(v, u) = point.z; // access as [row, col]
    //         }
    //         // else
    //         // {
    //         //     img.at<float>(v, u) = 0; // Set invalid points to 0
    //         // }
    //     }
    // }
    cv_ptrD = std::make_shared<const cv_bridge::CvImage>(cv_bridge::CvImage(cloud->header, sensor_msgs::image_encodings::TYPE_32FC1, img));
}

void SimulationRGBDSlamNode::publishPoseWithCovariance(const Sophus::SE3f &Tcw, const Eigen::Matrix<float, 6, 6> &covariance) 
{

        // Create PoseWithCovarianceStamped message
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";  // Adjust per your TF tree

        // Invert the transformation
        Sophus::SE3f Twc = Tcw.inverse();

        // Extract translation
        Eigen::Vector3f t = Twc.translation();
        pose_msg.pose.pose.position.x = t.x();
        pose_msg.pose.pose.position.y = t.y();
        pose_msg.pose.pose.position.z = t.z();

        // Convert rotation matrix to quaternion
        Eigen::Quaternionf q(Twc.rotationMatrix());
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
        // geometry_msgs::msg::TransformStamped transform_msg;

        // Sophus::SE3f Twc = Tcw.inverse();
        // Eigen::Vector3f t_inv = Twc.translation();
        // Eigen::Quaternionf q_inv(Twc.rotationMatrix());

        // transform_msg.header.stamp = this->now();
        // transform_msg.header.frame_id = "camera_depth_optical_frame";
        // transform_msg.child_frame_id = "map";  // Adjust per your TF tree

        // // Eigen::Matrix3f R_base_to_cam;
        // // R_base_to_cam = Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()); // 90° rotation around Y-axis

        // // Eigen::Quaternionf q_base_to_cam(R_base_to_cam);

        // // q = q_base_to_cam * q;
        
        // transform_msg.transform.translation.x = t_inv.x();
        // transform_msg.transform.translation.y = t_inv.y();
        // transform_msg.transform.translation.z = t_inv.z();

        // transform_msg.transform.rotation.x = q_inv.x();
        // transform_msg.transform.rotation.y = q_inv.y();
        // transform_msg.transform.rotation.z = q_inv.z();
        // transform_msg.transform.rotation.w = q_inv.w();

        // // Send TF transform
        // tf_broadcaster_->sendTransform(transform_msg);
}


void SimulationRGBDSlamNode::publishLandmarks()
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

void SimulationRGBDSlamNode::publishTrackedPose()
{
    // Create PoseWithCovarianceStamped message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "camera_depth_optical_frame";  // Adjust per your TF tree

    // Extract translation
    Eigen::Vector3f t = mLastPose.translation();
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();

    // Convert rotation matrix to quaternion
    Eigen::Quaternionf q(mLastPose.rotationMatrix());
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();


    // Publish pose with covariance
    pose_pub_->publish(pose_msg);

    // Also publish TF transform
    geometry_msgs::msg::TransformStamped transform_msg;

    // Sophus::SE3f Twc = mLastPose.inverse();
    // Sophus::SE3f Twc = mLastPose;
    // Eigen::Vector3f t_inv = mLastPose.translation();
    // Eigen::Quaternionf q_inv(mLastPose.rotationMatrix());


    transform_msg.header.stamp = this->now();
    transform_msg.header.frame_id = "camera_depth_optical_frame";
    transform_msg.child_frame_id = "map";  

    // Eigen::Matrix3f R_base_to_cam;
    // R_base_to_cam = Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()); // 90° rotation around Y-axis

    // Eigen::Quaternionf q_base_to_cam(R_base_to_cam);

    // q = q_base_to_cam * q;
    
    transform_msg.transform.translation.x = t.x();
    transform_msg.transform.translation.y = t.y();
    transform_msg.transform.translation.z = t.z();

    transform_msg.transform.rotation.x = q.x();
    transform_msg.transform.rotation.y = q.y();
    transform_msg.transform.rotation.z = q.z();
    transform_msg.transform.rotation.w = q.w();

    // Send TF transform
    tf_broadcaster_->sendTransform(transform_msg);

    transform_msg.header.stamp = this->now();
    transform_msg.header.frame_id = "map";
    transform_msg.child_frame_id = "world";  // Adjust per your TF tree

    // Eigen::Matrix3f R_base_to_cam;
    // R_base_to_cam = Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()); // 90° rotation around Y-axis

    // Eigen::Quaternionf q_base_to_cam(R_base_to_cam);

    // q = q_base_to_cam * q;
    Eigen::Quaternionf q_world_to_map(0.5, -0.5, 0.5, -0.5);
    Eigen::Quaternionf q_map_to_world = q_world_to_map.inverse();


    transform_msg.transform.translation.x = 0;
    transform_msg.transform.translation.y = 0;
    transform_msg.transform.translation.z = 0;

    transform_msg.transform.rotation.x = q_map_to_world.x();
    transform_msg.transform.rotation.y = q_map_to_world.y();
    transform_msg.transform.rotation.z = q_map_to_world.z();
    transform_msg.transform.rotation.w = q_map_to_world.w();

    // Send TF transform
    orb_to_map_broadcaster_->sendTransform(transform_msg);
}
