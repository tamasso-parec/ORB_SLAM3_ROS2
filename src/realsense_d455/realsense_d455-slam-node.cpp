#include "realsense_d455-slam-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RealsenseD455SlamNode::RealsenseD455SlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    // rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/color/image_raw");
    // depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/depth/image_rect_raw");
    // rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/camera/camera/color/image_raw");
    // depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "/camera/camera/depth/image_rect_raw");

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/color/image_raw");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/camera/camera/aligned_depth_to_color/image_raw");
    publisher_ = this->create_publisher<std_msgs::msg::String>("/RGBD_header", 10);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    // syncLatest = std::make_shared<message_filters::Synchronizer<latest_sync_policy> >(latest_sync_policy(), *rgb_sub, *depth_sub);

    syncApproximate->registerCallback(&RealsenseD455SlamNode::GrabRGBD, this);
    // syncLatest->registerCallback(&RealsenseD455SlamNode::GrabRGBD, this);


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
    auto message = std_msgs::msg::String();
    message.data = "Timestamp: " + std::to_string(msgRGB->header.stamp.sec);
    publisher_->publish(message);
    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
}
