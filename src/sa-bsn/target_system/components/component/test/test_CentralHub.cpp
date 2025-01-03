#include <gtest/gtest.h>
#include <ros/ros.h>
#include "component/CentralHub.hpp"

// Mock class for testing
class MockBattery : public bsn::resource::Battery {
public:
    MOCK_METHOD(double, getCurrentLevel, (), (const, override));
    MOCK_METHOD(void, generate, (double), (override));
};

// Fixture for testing CentralHub
class CentralHubRunTest : public ::testing::Test {
protected:
    int argc = 0;
    char** argv = nullptr;
    std::string name = "CentralHubRunTest";
    bool active = false;
    MockBattery battery;
    CentralHub* centralHub;

    void SetUp() override {
        centralHub = new CentralHub(argc, argv, name, active, battery);
    }

    void TearDown() override {
        delete centralHub;
    }
};

TEST_F(CentralHubTest, IsActiveInitiallyFalse) {
    EXPECT_FALSE(centralHub->isActive());
}

// Test the `run` method
TEST_F(CentralHubRunTest, RunMethodTest) {
    // Mock battery behavior
    EXPECT_CALL(battery, getCurrentLevel())
        .Times(::testing::AtLeast(1)) // Expect it to be called multiple times
        .WillRepeatedly(::testing::Return(50.0)); // Mock sufficient battery level

    // Start a thread to simulate the `run` method
    std::thread run_thread([this]() {
        ASSERT_NO_THROW(centralHub->run());
    });

    // Allow some time for `run` to execute
    ros::Duration(1.0).sleep();

    // Shut down ROS to stop the `run` loop
    ros::shutdown();

    // Join the thread to ensure proper cleanup
    run_thread.join();
}

// Test for turnOn()
TEST_F(CentralHubTest, TurnOnActivatesCentralHub) {
    centralHub->turnOn();
    EXPECT_TRUE(centralHub->isActive());
}

// Test for turnOff()
TEST_F(CentralHubTest, TurnOffDeactivatesCentralHub) {
    centralHub->turnOn(); // First turn on
    centralHub->turnOff(); // Then turn off
    EXPECT_FALSE(centralHub->isActive());
}

// Test for recharge()
TEST_F(CentralHubTest, RechargeIncreasesBatteryLevel) {
    EXPECT_CALL(battery, getCurrentLevel())
        .WillOnce(::testing::Return(50.0)); // Mock current battery level
    EXPECT_CALL(battery, generate(::testing::DoubleEq(5.0 / 20.0 / 10.0))); // Mock frequency = 10 Hz

    // Simulate a recharge call
    centralHub->recharge();
}

// Test for body() when inactive and battery is low
TEST_F(CentralHubTest, BodyThrowsWhenInactiveAndBatteryLow) {
    EXPECT_CALL(battery, getCurrentLevel())
        .WillOnce(::testing::Return(1.0)); // Mock low battery

    EXPECT_THROW(centralHub->body(), std::domain_error);
}

// Test for body() when active and battery is sufficient
TEST_F(CentralHubTest, BodyProcessesWhenActiveAndBatterySufficient) {
    EXPECT_CALL(battery, getCurrentLevel())
        .WillOnce(::testing::Return(50.0)); // Mock sufficient battery

    centralHub->turnOn(); // Activate the CentralHub

    EXPECT_NO_THROW(centralHub->body());
}

// Test reconfigure() with valid input
TEST_F(CentralHubTest, ReconfigureUpdatesFrequency) {
    archlib::AdaptationCommand::ConstPtr msg(new archlib::AdaptationCommand());
    msg->action = "freq=20.0";

    EXPECT_NO_THROW(centralHub->reconfigure(msg));
    // You can add further checks if you have getters for frequency.
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_central_hub_run");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
