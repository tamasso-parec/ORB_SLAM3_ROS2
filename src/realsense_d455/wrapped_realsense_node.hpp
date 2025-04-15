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


#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

class WrappedRealsenseD455SlamNode : public rclcpp::Node
{
public:

    WrappedRealsenseD455SlamNode(ORB_SLAM3::System* pSLAM);

    WrappedRealsenseD455SlamNode(std::string vocabulary_file, std::string settings_file, std::string file_name);

    ~WrappedRealsenseD455SlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;
    typedef message_filters::sync_policies::LatestTime<ImageMsg, ImageMsg> latest_sync_policy;
    typedef message_filters::sync_policies::ExactTime<ImageMsg, ImageMsg> exact_sync_policy;

    typedef Eigen::Matrix<float, 6,6> poseCov_t;
    typedef Eigen::Matrix<float, 3,3> landmarkCov_t;

    // REALSENSE
    void run();

    
    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    ORB_SLAM3::System* m_SLAM;

    std::string vocabularyFile, settingsFile, outFile;

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



    rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
    bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

    void interpolateData(const std::vector<double> &vBase_times,
                        std::vector<rs2_vector> &vInterp_data, std::vector<double> &vInterp_times,
                        const rs2_vector &prev_data, const double &prev_time);

    rs2_vector interpolateMeasure(const double target_time,
                                const rs2_vector current_data, const double current_time,
                                const rs2_vector prev_data, const double prev_time);

    static rs2_option get_sensor_option(const rs2::sensor& sensor)
    {
        // Sensors usually have several options to control their properties
        //  such as Exposure, Brightness etc.

        std::cout << "Sensor supports the following options:\n" << std::endl;

        // The following loop shows how to iterate over all available options
        // Starting from 0 until RS2_OPTION_COUNT (exclusive)
        for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
        {
            rs2_option option_type = static_cast<rs2_option>(i);
            //SDK enum types can be streamed to get a string that represents them
            std::cout << "  " << i << ": " << option_type;

            // To control an option, use the following api:

            // First, verify that the sensor actually supports this option
            if (sensor.supports(option_type))
            {
                std::cout << std::endl;

                // Get a human readable description of the option
                const char* description = sensor.get_option_description(option_type);
                std::cout << "       Description   : " << description << std::endl;

                // Get the current value of the option
                float current_value = sensor.get_option(option_type);
                std::cout << "       Current Value : " << current_value << std::endl;

                //To change the value of an option, please follow the change_sensor_option() function
            }
            else
            {
                std::cout << " is not supported" << std::endl;
            }
        }

        uint32_t selected_sensor_option = 0;
        return static_cast<rs2_option>(selected_sensor_option);
    }



};

#endif