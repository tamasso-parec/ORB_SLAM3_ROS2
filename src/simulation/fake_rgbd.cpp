#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "fake_rgbd_node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    
    std::shared_ptr<FakeRGBDSlamNode> node;

    node = std::make_shared<FakeRGBDSlamNode>();


    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}