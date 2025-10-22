
#include "fake_rgbd_node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

FakeRGBDSlamNode::FakeRGBDSlamNode()
:   Node("ORB_SLAM3_ROS2")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Create a timer to periodically perform tasks
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // Adjust the interval as needed
        std::bind(&FakeRGBDSlamNode::timerCallback, this));

    pcl_sub   = this->create_subscription<PclMsg>(
        "/x500_realsense/point_cloud", 10, std::bind(&FakeRGBDSlamNode::PointcloudCallback, this, _1));

    pose_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos, std::bind(&FakeRGBDSlamNode::trackedPoseCallback, this, _1));


    localBApublisher_ = this->create_publisher<std_msgs::msg::String>("/localBA", 10);

    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose_with_covariance", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/tracked_pose", 10);

    point_cloud_pub_ = this->create_publisher<uncertain_pointcloud_msgs::msg::UncertainPointCloud>("/slam/uncertain_point_cloud", 10);
    
    // TF2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    orb_to_map_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



}

FakeRGBDSlamNode::~FakeRGBDSlamNode()
{

    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

}

void FakeRGBDSlamNode::PointcloudCallback(const PclMsg::SharedPtr msgPcl)
{




    // Check if local BA was performed
    if(fake_lba_flag)
    {
        

        Sophus::SE3f latestPose = mLastPose;
        
        Eigen::Matrix<float, 6, 6> latestCov = Eigen::Matrix<float, 6, 6>::Identity();

        
        publishPoseWithCovariance(latestPose, latestCov);
        // std_msgs::msg::String msg;
        // msg.data = "Local BA performed";
        // localBApublisher_->publish(msg);

        publishLandmarks(msgPcl);

        fake_lba_flag = false;

    }
}

void FakeRGBDSlamNode::convertPcl2cv(const PclMsg::SharedPtr cloud)
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

void FakeRGBDSlamNode::publishPoseWithCovariance(const Sophus::SE3f &Tcw, const Eigen::Matrix<float, 6, 6> &covariance) 
{

        // Create PoseWithCovarianceStamped message
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "slam_map";  // Adjust per your TF tree

        // TODO: rotation has to be in slam_map frame

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
        // transform_msg.child_frame_id = "slam_map";  // Adjust per your TF tree

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


void FakeRGBDSlamNode::publishLandmarks(const PclMsg::SharedPtr msgPcl)
{
    // Create UncertainPointCloud message
    uncertain_pointcloud_msgs::msg::UncertainPointCloud point_cloud_msg;
    point_cloud_msg.header.stamp = this->now();
    point_cloud_msg.header.frame_id = "slam_map";  

    point_cloud_msg.points.reserve(mlocalLandmarks.size());

    // Iterate through the PointCloud2 message
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msgPcl, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msgPcl, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msgPcl, "z");

    int point_counter = 0;

    std::deque<Eigen::Vector3f> previous_points;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) 
    {
        if (point_counter++ % 400 != 0) 
        {
            continue;
        }
        // Check if the point is valid
        if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) 
        {
            uncertain_pointcloud_msgs::msg::UncertainPoint point_msg;

            

            Eigen::Vector4f homogeneous_point_cam(-*iter_y, -*iter_z,*iter_x,  1.0f);

            float depth = *iter_x;

            float sigma_z = depth * depth *0.01f / (0.5f ); // Adjust the scaling factor as needed

            Eigen::Matrix3f cov_world;
            cov_world.setZero();
            cov_world.diagonal() = 0.01f * Eigen::Vector3f::Random() + Eigen::Vector3f::UnitZ() * sigma_z;

            // Eigen::Vector4f homogeneous_point_cam(-*iter_y,  *iter_z,*iter_x,  1.0f);
            Eigen::Vector3f homogeneous_point_world = (Twc * homogeneous_point_cam).head<3>();
            
            // Rotate the points to be in the camera optical frame
            point_msg.x = homogeneous_point_world[0];          // x in camera frame corresponds to -y in drone frame
            point_msg.y = homogeneous_point_world[1];          // y in camera frame corresponds to -z in drone frame
            point_msg.z = homogeneous_point_world[2];           // z in camera frame corresponds to x in drone frame


            if (previous_points.size() > 2)
            {
                Eigen::MatrixXf diffs(3, previous_points.size());
                for (size_t i = 0; i < previous_points.size(); ++i)
                {
                    diffs.col(i) = homogeneous_point_cam.head<3>() - previous_points[i];
                }

                Eigen::Vector3f means = diffs.rowwise().mean();
                Eigen::MatrixXf centered = diffs.colwise() - means;
                Eigen::Matrix3f cov = 0.01f*(centered * centered.transpose()) / float(previous_points.size() - 1);


                cov_world = Twc.rotationMatrix() * cov * Twc.rotationMatrix().transpose();
            }

            // Set the covariance matrix
            // Set default covariance as identity if not provided
            point_msg.covariance[0] = cov_world(0,0);  // xx
            point_msg.covariance[1] = cov_world(0,1);  // xy
            point_msg.covariance[2] = cov_world(0,2);  // xz
            point_msg.covariance[3] = cov_world(1,1);  // yy
            point_msg.covariance[4] = cov_world(1,2);  // yz
            point_msg.covariance[5] = cov_world(2,2);  // zz
            
            previous_points.push_back(homogeneous_point_cam.head<3>());
            if (previous_points.size() > 10)
            {
                previous_points.pop_front();
            }

            // Add point to the point cloud message
            point_cloud_msg.points.push_back(point_msg);
        }
    }

    

    // Publish point cloud
    point_cloud_pub_->publish(point_cloud_msg);
}

void FakeRGBDSlamNode::publishTrackedPose()
{
    // Create PoseWithCovarianceStamped message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "slam_map";  // Adjust per your TF tree

    // Extract translation
    // Invert the transformation

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

    // Also publish TF transform
    geometry_msgs::msg::TransformStamped transform_msg;

    // Sophus::SE3f Twc = mLastPose.inverse();
    // Sophus::SE3f Twc = mLastPose;
    // Eigen::Vector3f t_inv = mLastPose.translation();
    // Eigen::Quaternionf q_inv(mLastPose.rotationMatrix());


    transform_msg.header.stamp = this->now();
    transform_msg.header.frame_id = "slam_map";
    transform_msg.child_frame_id = "camera_color_optical_frame";  

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

    // transform_msg.header.stamp = this->now();
    // transform_msg.header.frame_id = "slam_map";
    // transform_msg.child_frame_id = "world";  // Adjust per your TF tree

    // // Eigen::Matrix3f R_base_to_cam;
    // // R_base_to_cam = Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitY()); // 90° rotation around Y-axis

    // // Eigen::Quaternionf q_base_to_cam(R_base_to_cam);

    // // q = q_base_to_cam * q;
    // Eigen::Quaternionf q_world_to_map(0.5, -0.5, 0.5, -0.5);
    // Eigen::Quaternionf q_map_to_world = q_world_to_map.inverse();


    // transform_msg.transform.translation.x = 0;
    // transform_msg.transform.translation.y = 0;
    // transform_msg.transform.translation.z = 0;

    // transform_msg.transform.rotation.x = q_map_to_world.x();
    // transform_msg.transform.rotation.y = q_map_to_world.y();
    // transform_msg.transform.rotation.z = q_map_to_world.z();
    // transform_msg.transform.rotation.w = q_map_to_world.w();

    // // Send TF transform
    // orb_to_map_broadcaster_->sendTransform(transform_msg);
}


void FakeRGBDSlamNode::trackedPoseCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msgPose){
        std::cout << "Pose received" << std::endl;

        geometry_msgs::msg::TransformStamped t_drone_cam, t_map_slammap;

        try {
          t_drone_cam = tf_buffer_->lookupTransform(
            "drone", "x500_realsense/realsense_d435/base_link/realsense_d435",
            tf2::TimePointZero);

            t_map_slammap = tf_buffer_->lookupTransform(
            "map", "slam_map",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "drone", "x500_realsense/realsense_d435/base_link/realsense_d435", ex.what());
          return;
        }

        Eigen::Vector3f t_MAP_DRONE(msgPose->position[0],  -msgPose->position[1], -msgPose->position[2]);

        
        Eigen::Quaternionf q_MAP_SLAMMAP(t_map_slammap.transform.rotation.w,
                                        t_map_slammap.transform.rotation.x,
                                        t_map_slammap.transform.rotation.y,
                                        t_map_slammap.transform.rotation.z);

        Eigen::Quaternionf q_SLAMMAP_MAP = q_MAP_SLAMMAP.inverse();



        Eigen::Quaternionf q_DRONE_CAM(t_drone_cam.transform.rotation.w,
                                        t_drone_cam.transform.rotation.x,
                                        t_drone_cam.transform.rotation.y,
                                        t_drone_cam.transform.rotation.z);

        Eigen::Matrix4f T_DRONE_CAM = Eigen::Matrix4f::Identity();
        T_DRONE_CAM.block<3,3>(0,0) = q_DRONE_CAM.toRotationMatrix();


        Eigen::Quaternionf q_CAM_DRONE = q_DRONE_CAM.inverse();
        Eigen::Quaternionf q_SLAMMAP_ROTATEDMAP = q_CAM_DRONE;
        Eigen::Quaternionf q_MAP_ROTATEDSLAMMAP = q_DRONE_CAM;

        Eigen::Matrix4f T_SLAMMAP_MAP = Eigen::Matrix4f::Identity();
        T_SLAMMAP_MAP.block<3,1>(0,3) = Eigen::Vector3f(t_drone_cam.transform.translation.x,
                                                        t_drone_cam.transform.translation.y,
                                                        t_drone_cam.transform.translation.z);
        T_SLAMMAP_MAP.block<3,3>(0,0) = q_SLAMMAP_MAP.toRotationMatrix();

        // Rotate the quaternion from the ROS frame to the optical frame
        Eigen::Quaternionf q_MAP_DRONE(msgPose->q[0], msgPose->q[1], -msgPose->q[2], -msgPose->q[3]); 
        
        Eigen::Matrix4f T_MAP_DRONE = Eigen::Matrix4f::Identity();
        T_MAP_DRONE.block<3, 1>(0, 3) = t_MAP_DRONE;
        T_MAP_DRONE.block<3, 3>(0, 0) = q_MAP_DRONE.toRotationMatrix(); // Convert quaternion to rotation matrix

        Eigen::Matrix4f T_SLAMMAP_CAM = T_SLAMMAP_MAP * T_MAP_DRONE * T_DRONE_CAM;



        // Eigen::Quaternionf q_SLAMMAP_CAM = q_SLAMMAP_MAP * q_MAP_DRONE * q_DRONE_CAM;

        


        // Eigen::Vector3f t_SLAMMAP_CAM =  q_SLAMMAP_MAP * t_MAP_DRONE 

        



        // Eigen::Quaternionf q_cam_to_slam_map =  Eigen::Quaternionf(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitZ())) * q_ros;
        Twc = Sophus::SE3f(T_SLAMMAP_CAM); 

                                // Eigen::Vector3f(msgPose->position[0],  msgPose->position[1], msgPose->position[2])); 
        mLastPose = Twc;  // Tcw 

        publishTrackedPose();


    }