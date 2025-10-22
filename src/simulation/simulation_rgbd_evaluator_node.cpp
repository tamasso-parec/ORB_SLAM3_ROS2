
#include "simulation_rgbd_evaluator_node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

SimulationRGBDEvaluationNode::SimulationRGBDEvaluationNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),

    m_SLAM(pSLAM)
{

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/rgbd_camera/image");
    
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/rgbd_camera/depth_image");

    // pcl_sub   = std::make_shared<message_filters::Subscriber<PclMsg> >(this, "/rgbd_camera/points");

    localBApublisher_ = this->create_publisher<std_msgs::msg::String>("/localBA", 10);

    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam_evaluation/pose_with_covariance", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam_evaluation/tracked_pose", 10);

    point_cloud_pub_ = this->create_publisher<uncertain_pointcloud_msgs::msg::UncertainPointCloud>("/slam_evaluation/uncertain_point_cloud", 10);

    // TF2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    orb_to_map_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    syncExact = std::make_shared<message_filters::Synchronizer<exact_sync_policy> >(exact_sync_policy(10), *rgb_sub, *depth_sub);

    // syncExact = std::make_shared<message_filters::Synchronizer<exact_sync_policy> >(exact_sync_policy(10), *rgb_sub, *pcl_sub);
    // syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *pcl_sub);

    syncExact->registerCallback(&SimulationRGBDEvaluationNode::GrabRGBD, this);
    // syncApproximate->registerCallback(&SimulationRGBDEvaluationNode::GrabRGBD, this);

    pose_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/slam/pose_with_covariance", 10,
        std::bind(&SimulationRGBDEvaluationNode::gtPoseCallback, this, _1)
    );

    gt_pose_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/slam_evaluation/gt_pose_marker", 10);


}

SimulationRGBDEvaluationNode::~SimulationRGBDEvaluationNode()
{

    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

}

void SimulationRGBDEvaluationNode::GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
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

    mLastPose = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));

    Eigen::Matrix<float, 6, 6> pose_cov; 
    pose_cov.setIdentity();

    publishTrackedPose();
    publishPoseWithCovariance(mLastPose, pose_cov);

}

void SimulationRGBDEvaluationNode::convertPcl2cv(const PclMsg::SharedPtr cloud)
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

void SimulationRGBDEvaluationNode::publishPoseWithCovariance(const Sophus::SE3f &Tcw, const Eigen::Matrix<float, 6, 6> &covariance) 
{

        // Create PoseWithCovarianceStamped message
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "slam_map";  // Adjust per your TF tree

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

       
}


void SimulationRGBDEvaluationNode::publishLandmarks()
{
    // Create UncertainPointCloud message
    uncertain_pointcloud_msgs::msg::UncertainPointCloud point_cloud_msg;
    point_cloud_msg.header.stamp = this->now();
    point_cloud_msg.header.frame_id = "slam_map";  // Adjust per your TF tree

    point_cloud_msg.points.reserve(mlocalLandmarks.size());

    // Iterate over all landmarks
    for (const auto& landmark : mlocalLandmarks) 
    {
        // Create PointWithCovariance message
        uncertain_pointcloud_msgs::msg::UncertainPoint point_msg;


        // Extract landmark position
        Eigen::Vector3f p = landmark.second;

        // Check if position contains NaN or infinity
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
            RCLCPP_WARN(this->get_logger(), "Skipping landmark with invalid position.");
            continue;
        }

        point_msg.x = p.x();
        point_msg.y = p.y();
        point_msg.z = p.z();
        
        // Extract landmark covariance
        Eigen::Matrix<float, 3, 3>& cov = landmarkCovs[landmark.first];

        // Check if covariance contains NaN or infinity
        if (!std::isfinite(cov(0, 0)) || !std::isfinite(cov(0, 1)) || !std::isfinite(cov(0, 2)) ||
            !std::isfinite(cov(1, 1)) || !std::isfinite(cov(1, 2)) || !std::isfinite(cov(2, 2))) {
            cov = Eigen::Matrix3f::Identity();  // Set to identity if invalid

        }

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

void SimulationRGBDEvaluationNode::publishTrackedPose()
{
    // Create PoseWithCovarianceStamped message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "slam_map";  // Adjust per your TF tree

    // Extract translation
    // Invert the transformation
    Sophus::SE3f Twc = mLastPose.inverse();

    Eigen::Vector3f t = Twc.translation();
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();

    // Convert rotation matrix to quaternion
    Eigen::Quaternionf q(Twc.rotationMatrix());
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();


    // Publish pose with covariance
    pose_pub_->publish(pose_msg);

    
}


void SimulationRGBDEvaluationNode::gtPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    Eigen::Quaternionf q_gt(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);

    Eigen::Vector3f t_gt(msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.position.z);

    gtLastPose = Sophus::SE3f(q_gt.toRotationMatrix(), t_gt);

    publishGTPoseMarker();




}

void SimulationRGBDEvaluationNode::publishGTPoseMarker(){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "slam_map";
    marker.header.stamp = this->now();
    marker.ns = "gt_pose";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set the start and end points of the arrow
    Eigen::Vector3f t = gtLastPose.translation();
    marker.points.resize(2);
    marker.points[0].x = t.x();
    marker.points[0].y = t.y();
    marker.points[0].z = t.z();

    Eigen::Matrix3f R = gtLastPose.rotationMatrix();
    Eigen::Vector3f arrow_dir = R * Eigen::Vector3f(0.0, 0, 1.0); // Arrow length of 0.2 meters

    marker.points[1].x = t.x() + arrow_dir.x();
    marker.points[1].y = t.y() + arrow_dir.y();
    marker.points[1].z = t.z() + arrow_dir.z();

    // Set the scale of the arrow
    marker.scale.x = 0.05; // Shaft diameter
    marker.scale.y = 0.1;  // Head diameter
    marker.scale.z = 0.1;  // Head length

    // Set the color of the arrow (e.g., red)
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // Publish the marker
    gt_pose_marker_pub_->publish(marker);
}