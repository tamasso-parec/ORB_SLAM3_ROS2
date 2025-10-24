
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

    estimated_pose_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/slam_evaluation/estimated_pose_marker", 10);

    // Declare a string parameter for output filename (default "KeyFrameTrajectory.txt")
    this->declare_parameter<std::string>("gt_filename", "gt_trajectory.txt");
    gt_filename = this->get_parameter("gt_filename").as_string();
    RCLCPP_INFO(this->get_logger(), "Ground truth filename parameter: %s", gt_filename.c_str());

    this->declare_parameter<std::string>("est_filename", "est_trajectory.txt");
    est_filename = this->get_parameter("est_filename").as_string();
    RCLCPP_INFO(this->get_logger(), "Estimated filename parameter: %s", est_filename.c_str());

    rclcpp::on_shutdown([this]() { this->writeLogsToFile(); });


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

    est_poses_log.push_back(mLastPose);
    auto time = this->now();
    est_time_log.push_back(time.seconds() + time.nanoseconds() * 1e-9);

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

    //  TODO: We need to rotate it so that it matches the slam frame

    gtLastPose = Sophus::SE3f(q_gt.toRotationMatrix(), t_gt);

    gt_poses_log.push_back(gtLastPose);
    auto time = this->now();
    gt_time_log.push_back(time.seconds() + time.nanoseconds() * 1e-9);

    publishEstimatedPoseMarker();

}

void SimulationRGBDEvaluationNode::publishEstimatedPoseMarker(){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "slam_map";
    marker.header.stamp = this->now();
    marker.ns = "slam_pose";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.lifetime = rclcpp::Duration(0, 0);  // forever

    // Set the start and end points of the arrow
    Eigen::Vector3f t = mLastPose.inverse().translation();

    
    marker.points.resize(2);
    marker.points[0].x = t.x();
    marker.points[0].y = t.y();
    marker.points[0].z = t.z();

    Eigen::Matrix3f R = mLastPose.inverse().rotationMatrix();
    Eigen::Vector3f arrow_dir = R * Eigen::Vector3f(0.0, 0, 0.5); // Arrow length of 1.0 meters

    marker.points[1].x = t.x() + arrow_dir.x();
    marker.points[1].y = t.y() + arrow_dir.y();
    marker.points[1].z = t.z() + arrow_dir.z();

    // Set the scale of the arrow
    marker.scale.x = 0.05; // Shaft diameter
    marker.scale.y = 0.1;  // Head diameter
    marker.scale.z = 0.1;  // Head length

    // Set the color of the arrow (e.g., red) rgb(248, 123, 27)
    marker.color.r = 248.0f / 255.0f;
    marker.color.g = 123.0f / 255.0f;
    marker.color.b = 27.0f / 255.0f;
    marker.color.a = 1.0f;

    // Publish the marker
    estimated_pose_marker_pub_->publish(marker);

    // Add a line strip
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "slam_map";
    line_strip.header.stamp = this->now();
    line_strip.ns = "slam_trajectory";
    line_strip.id = 1;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;

    line_strip.lifetime = rclcpp::Duration(0, 0);  // forever

    // Set the start and end points of the arrow
    
    line_strip.points.resize(2);

    line_strip.points[0].x = previous_point.x();
    line_strip.points[0].y = previous_point.y();
    line_strip.points[0].z = previous_point.z();

    line_strip.points[1].x = t.x();
    line_strip.points[1].y = t.y();
    line_strip.points[1].z = t.z();

    // Set the scale of the arrow
    line_strip.scale.x = 0.05; // Shaft diameter
    line_strip.scale.y = 0.1;  // Head diameter
    line_strip.scale.z = 0.1;  // Head length

    // Set the color of the arrow (e.g., red) rgb(248, 123, 27)
    line_strip.color.r = 248.0f / 255.0f;
    line_strip.color.g = 123.0f / 255.0f;
    line_strip.color.b = 27.0f / 255.0f;
    line_strip.color.a = 1.0f;

    // Publish the marker
    estimated_pose_marker_pub_->publish(line_strip);

    previous_point = t;

}


void SimulationRGBDEvaluationNode::writeLogsToFile()
{
    RCLCPP_INFO(this->get_logger(), "Writing trajectory logs to files...");
    // Write ground truth poses to file
    std::ofstream gt_file(gt_filename);
    if (gt_file.is_open()) {
        for (size_t i = 0; i < gt_poses_log.size(); ++i) {
            const Sophus::SE3f& pose = gt_poses_log[i];
            Eigen::Quaternionf quat = Eigen::Quaternionf(pose.rotationMatrix());
            float timestamp = gt_time_log[i];
            gt_file << std::to_string(timestamp) <<  " "
                    << pose.translation().x() <<  " "
                    << pose.translation().y() <<  " "
                    << pose.translation().z() <<  " "
                    << quat.x() <<  " "
                    << quat.y() <<  " "
                    << quat.z() <<  " "
                    << quat.w() << "\n";
        }
        gt_file.close();
        RCLCPP_INFO(this->get_logger(), "Ground truth trajectory saved to %s", gt_filename.c_str());
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Unable to open file %s for writing", gt_filename.c_str());
    }

    // Write estimated poses to file
    std::ofstream est_file(est_filename);
    if (est_file.is_open()) {
        for (size_t i = 0; i < est_poses_log.size(); ++i) {
            const Sophus::SE3f& pose = est_poses_log[i];
            Eigen::Quaternionf quat = Eigen::Quaternionf(pose.rotationMatrix());
            float timestamp = est_time_log[i];
            est_file << std::to_string(timestamp)  <<  " "
                     << pose.translation().x() <<  " "
                     << pose.translation().y() <<  " "
                     << pose.translation().z() <<  " "
                     << quat.x() <<  " "
                     << quat.y() <<  " "
                     << quat.z() <<  " "
                     << quat.w() << "\n";
        }
        est_file.close();
        // RCLCPP_INFO(this->get_logger(), "Estimated trajectory saved to %s", est_filename.c_str());
    } 
    else {
       RCLCPP_ERROR(this->get_logger(), "Unable to open file %s for writing", est_filename.c_str());
    }
}