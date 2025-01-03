#include <gtest/gtest.h>
#include <ros/ros.h>
#include <memory>
#include "component/Sensor.hpp"

// Mock classes
class MockBattery : public bsn::resource::Battery
{
public:
    MOCK_METHOD(double, getCurrentLevel, (), (const, override));
    MOCK_METHOD(void, generate, (double), (override));
};

// Fixture for testing Sensor
class SensorTest : public ::testing::Test
{
protected:
    int argc = 0;
    char **argv = nullptr;
    std::string name = "SensorTest";
    std::string type = "test_sensor";
    bool active = false;
    double noise_factor = 0.0;
    MockBattery battery;
    bool instant_recharge = false;
    std::unique_ptr<Sensor> sensor;

    void SetUp() override
    {
        sensor = std::make_unique<Sensor>(argc, argv, name, type, active, noise_factor, battery, instant_recharge);
    }

    void TearDown() override
    {
        sensor.reset();
    }
};

// Test isActive()
TEST_F(SensorTest, IsActiveTest)
{
    EXPECT_FALSE(sensor->isActive());
    sensor->turnOn();
    EXPECT_TRUE(sensor->isActive());
    sensor->turnOff();
    EXPECT_FALSE(sensor->isActive());
}

// Test turnOn() and turnOff()
TEST_F(SensorTest, TurnOnAndOffTest)
{
    sensor->turnOn();
    EXPECT_TRUE(sensor->isActive());
    sensor->turnOff();
    EXPECT_FALSE(sensor->isActive());
}

// Test apply_noise()
TEST_F(SensorTest, ApplyNoiseTest)
{
    double data = 100.0;
    noise_factor = 0.1; // 10% noise
    sensor->apply_noise(data);
    EXPECT_NEAR(data, 100.0, 10.0); // Ensure data is within +/- 10% range
}

// Test recharge() without instant recharge
TEST_F(SensorTest, RechargeTest)
{
    EXPECT_CALL(battery, getCurrentLevel()).WillOnce(::testing::Return(50.0));
    EXPECT_CALL(battery, generate(1)).Times(1);
    sensor->recharge();
}

// Test recharge() with instant recharge
TEST_F(SensorTest, InstantRechargeTest)
{
    instant_recharge = true;
    sensor = std::make_unique<Sensor>(argc, argv, name, type, active, noise_factor, battery, instant_recharge);

    EXPECT_CALL(battery, generate(100)).Times(1);
    sensor->recharge();
}

// Test reconfigure()
TEST_F(SensorTest, ReconfigureTest)
{
    archlib::AdaptationCommand::Ptr msg(new archlib::AdaptationCommand);
    msg->action = "freq=5.0,replicate_collect=10";

    sensor->reconfigure(msg);

    // Verify frequency and replicate_collect changes
    EXPECT_EQ(sensor->rosComponentDescriptor.getFreq(), 5.0);
    EXPECT_EQ(sensor->replicate_collect, 10);
}

// Test injectUncertainty()
TEST_F(SensorTest, InjectUncertaintyTest)
{
    archlib::Uncertainty::Ptr msg(new archlib::Uncertainty);
    msg->content = "noise_factor=0.2";

    sensor->injectUncertainty(msg);

    EXPECT_DOUBLE_EQ(sensor->noise_factor, 0.2);
}
