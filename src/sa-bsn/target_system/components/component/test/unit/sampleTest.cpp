#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/master.h"
#include <chrono>
#include <thread>
TEST(SimpleROS, InitAndSpin)
{
    ros::NodeHandle nh;

    for (int i = 0; i < 10; ++i)
    { // Limit retries to 10 attempts
        if (!ros::master::check())
        {
            FAIL() << "ROS master is not available. Please start roscore.";
            return;
        }
        if (!ros::ok())
        {
            FAIL() << "ROS shutdown detected.";
            return;
        }
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

TEST(FailingTest, AlwaysFails)
{
    ASSERT_TRUE(false) << "This test is designed to fail.";
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_ros_minimal");

    if (!ros::master::check())
    {
        std::cerr << "ROS master is not running. Please start roscore." << std::endl;
        return EXIT_FAILURE;
    }

    return RUN_ALL_TESTS();
}