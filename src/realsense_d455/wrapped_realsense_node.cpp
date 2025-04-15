#include "wrapped_realsense_node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

WrappedRealsenseD455SlamNode::WrappedRealsenseD455SlamNode(std::string vocabulary_file, std::string settings_file, std::string file_name)

:   Node("ORB_SLAM3_ROS2"), vocabularyFile(vocabulary_file), settingsFile(settings_file), outFile(file_name)
{

    // rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/color/image_raw");
    
    // depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/aligned_depth_to_color/image_raw");

    localBApublisher_ = this->create_publisher<std_msgs::msg::String>("/localBA", 10);

    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose_with_covariance", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/tracked_pose", 10);

    point_cloud_pub_ = this->create_publisher<uncertain_pointcloud_msgs::msg::UncertainPointCloud>("/slam/uncertain_point_cloud", 10);
    
    // TF2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    orb_to_map_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // syncExact = std::make_shared<message_filters::Synchronizer<exact_sync_policy> >(exact_sync_policy(10), *rgb_sub, *depth_sub);

    // syncExact->registerCallback(&WrappedRealsenseD455SlamNode::GrabRGBD, this);

    run();


}

WrappedRealsenseD455SlamNode::WrappedRealsenseD455SlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),

    m_SLAM(pSLAM)
{

    // rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/color/image_raw");
    
    // depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/aligned_depth_to_color/image_raw");

    localBApublisher_ = this->create_publisher<std_msgs::msg::String>("/localBA", 10);

    pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose_with_covariance", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/tracked_pose", 10);

    point_cloud_pub_ = this->create_publisher<uncertain_pointcloud_msgs::msg::UncertainPointCloud>("/slam/uncertain_point_cloud", 10);
    
    // TF2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    orb_to_map_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // syncExact = std::make_shared<message_filters::Synchronizer<exact_sync_policy> >(exact_sync_policy(10), *rgb_sub, *depth_sub);

    // syncExact->registerCallback(&WrappedRealsenseD455SlamNode::GrabRGBD, this);

    run();


}

WrappedRealsenseD455SlamNode::~WrappedRealsenseD455SlamNode()
{

    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

}

void WrappedRealsenseD455SlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
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

void WrappedRealsenseD455SlamNode::publishPoseWithCovariance(const Sophus::SE3f &Tcw, const Eigen::Matrix<float, 6, 6> &covariance) 
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


void WrappedRealsenseD455SlamNode::publishLandmarks()
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

void WrappedRealsenseD455SlamNode::publishTrackedPose()
{
    // Create PoseWithCovarianceStamped message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "camera_color_optical_frame";  // Adjust per your TF tree

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
    transform_msg.header.frame_id = "camera_color_optical_frame";
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


void WrappedRealsenseD455SlamNode::run()
{
    // Initialize the camera
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
    }
    else
        selected_device = devices[0];

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    int index = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors)
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
            ++index;
            if (index == 1) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,50000);
                sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1); // emitter on for depth information
            }
            // std::cout << "  " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            get_sensor_option(sensor);
            if (index == 2){
                // RGB camera
                sensor.set_option(RS2_OPTION_EXPOSURE,80.f);

            }

            if (index == 3){
                sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION,0);
            }

        }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // RGB stream
    cfg.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_RGB8, 30);

    // Depth stream
    // cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH,640, 480, RS2_FORMAT_Z16, 30);

    // IMU stream
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F); //, 250); // 63
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F); //, 400);

    // IMU callback
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    vector<double> v_accel_timestamp;
    vector<rs2_vector> v_accel_data;
    vector<double> v_gyro_timestamp;
    vector<rs2_vector> v_gyro_data;

    double prev_accel_timestamp = 0;
    rs2_vector prev_accel_data;
    double current_accel_timestamp = 0;
    rs2_vector current_accel_data;
    vector<double> v_accel_timestamp_sync;
    vector<rs2_vector> v_accel_data_sync;

    cv::Mat imCV, depthCV;
    int width_img, height_img;
    double timestamp_image = -1.0;
    bool image_ready = false;
    int count_im_buffer = 0; // count dropped frames

    // start and stop just to get necessary profile
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);
    pipe.stop();

    // Align depth and RGB frames
    //Pipeline could choose a device that does not have a color stream
    //If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(pipe_profile.get_streams());

    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    //The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);
    rs2::frameset fsSLAM;

    auto imu_callback = [&](const rs2::frame& frame)
    {
        std::unique_lock<std::mutex> lock(imu_mutex);

        if(rs2::frameset fs = frame.as<rs2::frameset>())
        {
            count_im_buffer++;

            double new_timestamp_image = fs.get_timestamp()*1e-3;
            if(abs(timestamp_image-new_timestamp_image)<0.001){
                count_im_buffer--;
                return;
            }

            if (profile_changed(pipe.get_active_profile().get_streams(), pipe_profile.get_streams()))
            {
                //If the profile was changed, update the align object, and also get the new device's depth scale
                pipe_profile = pipe.get_active_profile();
                align_to = find_stream_to_align(pipe_profile.get_streams());
                align = rs2::align(align_to);
            }

            //Align depth and rgb takes long time, move it out of the interruption to avoid losing IMU measurements
            fsSLAM = fs;

            

            timestamp_image = fs.get_timestamp()*1e-3;
            image_ready = true;

            while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size())
            {
                int index = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];

                v_accel_data_sync.push_back(current_accel_data);
                v_accel_timestamp_sync.push_back(target_time);
            }

            lock.unlock();
            cond_image_rec.notify_all();
        }
    };

    pipe_profile = pipe.start(cfg, imu_callback);

    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);


    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_cam.width;
    height_img = intrinsics_cam.height;
    std::cout << " fx = " << intrinsics_cam.fx << std::endl;
    std::cout << " fy = " << intrinsics_cam.fy << std::endl;
    std::cout << " cx = " << intrinsics_cam.ppx << std::endl;
    std::cout << " cy = " << intrinsics_cam.ppy << std::endl;
    std::cout << " height = " << intrinsics_cam.height << std::endl;
    std::cout << " width = " << intrinsics_cam.width << std::endl;
    std::cout << " Coeff = " << intrinsics_cam.coeffs[0] << ", " << intrinsics_cam.coeffs[1] << ", " <<
    intrinsics_cam.coeffs[2] << ", " << intrinsics_cam.coeffs[3] << ", " << intrinsics_cam.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_cam.model << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocabularyFile ,  settingsFile,ORB_SLAM3::System::RGBD, true, 0, outFile);
    m_SLAM = &SLAM;
    float imageScale = m_SLAM->GetImageScale();

    double timestamp;
    cv::Mat im, depth;

    double t_resize = 0.f;
    double t_track = 0.f;
    rs2::frameset fs;

    while (!m_SLAM->isShutDown())
    {
        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            if(!image_ready)
            {
                cond_image_rec.wait(lk);
                // std::cout << "Waiting for image\n";
            }

            fs = fsSLAM;

            if(count_im_buffer>1)
                cout << count_im_buffer -1 << " dropped frs\n";
            count_im_buffer = 0;

            timestamp = timestamp_image;
            im = imCV.clone();
            depth = depthCV.clone();

            image_ready = false;
        }

        // std::cout << "Running SLAM\n";
        // Perform alignment here
        auto processed = align.process(fs);

        // Trying to get both other and aligned depth frames
        rs2::video_frame color_frame = processed.first(align_to);
        rs2::depth_frame depth_frame = processed.get_depth_frame();

        im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
        depth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);


        /*cv::Mat depthCV_8U;
        depthCV.convertTo(depthCV_8U,CV_8U,0.01);
        cv::imshow("depth image", depthCV_8U);*/

        if(imageScale != 1.f)
        {

            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(depth, depth, cv::Size(width, height));
        }
        // Pass the image to the SLAM system
        mLastPose = m_SLAM->TrackRGBD(im, depth, timestamp); //, vImuMeas); depthCV

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
    cout << "System shutdown!\n";
}




rs2_stream WrappedRealsenseD455SlamNode::find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}


bool WrappedRealsenseD455SlamNode::profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}