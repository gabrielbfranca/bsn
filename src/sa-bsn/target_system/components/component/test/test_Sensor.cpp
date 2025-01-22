/*
#include <gtest/gtest.h>
#include "component/g3t1_3/G3T1_3.hpp"
#include "ros/ros.h"
#include <stdexcept>

// Mock ROS NodeHandle
class MockNodeHandle : public ros::NodeHandle
{
public:
    bool getParam(const std::string &param_name, std::string &param_value)
    {
        if (param_name == "test_param")
        {
            param_value = "mock_value";
            return true;
        }
        if (param_name == "start")
        {
            param_value = "true";
            return true;
        }
        if (param_name == "temperature_LowRisk")
        {
            param_value = "36.5,37.5";
            return true;
        }
        if (param_name == "temperature_MidRisk0" || param_name == "temperature_MidRisk1")
        {
            param_value = "37.5,38.0";
            return true;
        }
        if (param_name == "temperature_HighRisk0" || param_name == "temperature_HighRisk1")
        {
            param_value = "38.0,39.0";
            return true;
        }
        if (param_name == "lowrisk")
        {
            param_value = "0,50";
            return true;
        }
        if (param_name == "midrisk")
        {
            param_value = "50,80";
            return true;
        }
        if (param_name == "highrisk")
        {
            param_value = "80,100";
            return true;
        }
        if (param_name == "instant_recharge")
        {
            param_value = "true";
            return true;
        }
        return false;
    }
};

// Test Fixture
class G3T1_3Fixture : public ::testing::Test
{
protected:
    int argc = 0;
    char **argv = nullptr;
    G3T1_3 *sensor;

    G3T1_3Fixture() : argc(0), argv(nullptr)
    {
        ros::NodeHandle *mock_handle = new MockNodeHandle();
        sensor = new G3T1_3(argc, argv, "test_sensor");
        sensor->handle = *mock_handle; // Inject the mocked handle
    }

    ~G3T1_3Fixture()
    {
        delete sensor;
    }

    void SetUp() override
    {
        sensor->setUp();
    }

    void TearDown() override
    {
        sensor->tearDown();
    }
};

// Test: setUp and tearDown
TEST_F(G3T1_3Fixture, TestSetUpAndTearDown)
{
    EXPECT_NO_THROW(sensor->setUp());
    EXPECT_NO_THROW(sensor->tearDown());
}

// Test: collect
TEST_F(G3T1_3Fixture, TestCollect)
{
    double data = 0;
    EXPECT_NO_THROW(data = sensor->collect());
    EXPECT_GE(data, 0); // Data should be non-negative
}

// Test: process
TEST_F(G3T1_3Fixture, TestProcess)
{
    double raw_data = 37.0; // Simulated raw data
    double filtered_data = 0;
    EXPECT_NO_THROW(filtered_data = sensor->process(raw_data));
    EXPECT_GT(filtered_data, 0); // Processed data should be greater than 0
}

// Test: transfer
TEST_F(G3T1_3Fixture, TestTransfer)
{
    double valid_data = 37.5; // Simulated valid data
    EXPECT_NO_THROW(sensor->transfer(valid_data));

    // Test transfer with invalid data
    double invalid_data = -1.0; // Out of bounds risk
    EXPECT_THROW(sensor->transfer(invalid_data), std::domain_error);
}

// Test: Label function (indirectly tested in collect and transfer)
/*TEST_F(G3T1_3Fixture, TestLabel)
{
    double risk_low = 45.0;
    double risk_mid = 60.0;
    double risk_high = 90.0;
    std::string label;

    EXPECT_NO_THROW(label = sensor->label(risk_low));
    EXPECT_EQ(label, "low");

    EXPECT_NO_THROW(label = sensor->label(risk_mid));
    EXPECT_EQ(label, "moderate");

    EXPECT_NO_THROW(label = sensor->label(risk_high));
    EXPECT_EQ(label, "high");
}

TEST_F(G3T1_3Fixture, TestCollectWithLabel)
{
    double data = sensor->collect();
    ASSERT_NO_THROW(sensor->transfer(data)); // Indirectly validates `label`
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_g3t1_3");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
#include <gtest/gtest.h>
#include "G3T1_3.hpp"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "services/PatientData.h"

// Mock service callback for "temp"
bool mockTempCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = true;
    res.message = "37.0"; // Simulate temperature data
    return true;
}

// Mock service callback for "getPatientData"
bool mockPatientDataCallback(services::PatientData::Request &req, services::PatientData::Response &res)
{
    ROS_INFO_STREAM("mockPatientDataCallback called with vitalSign: " << req.vitalSign);

    if (req.vitalSign == "temperature")
    {
        res.data = 37.0;
    }
    else if (req.vitalSign == "heartRate")
    {
        res.data = 75.0;
    }
    else
    {
        res.data = -1.0;
    }

    ROS_INFO_STREAM("mockPatientDataCallback returning data: " << res.data);
    return true;
}

// Test Fixture
class G3T1_3Fixture : public ::testing::Test
{
protected:
    int argc = 0;
    char **argv = nullptr;
    G3T1_3 *sensor;
    ros::AsyncSpinner *spinner;

    G3T1_3Fixture()
    {
        sensor = new G3T1_3(argc, argv, "test_sensor");
        spinner = new ros::AsyncSpinner(1); // Use 1 thread for spinning
        spinner->start();
    }

    ~G3T1_3Fixture()
    {
        spinner->stop();
        delete spinner;
        delete sensor;
    }

    void SetUp() override
    {
        ros::param::set("start", true);
        ros::NodeHandle nh;

        ROS_INFO("Advertising mock services...");
        nh.advertiseService("temp", mockTempCallback);
        nh.advertiseService("getPatientData", mockPatientDataCallback);
        ROS_INFO("Mock services advertised successfully.");

        sensor->setUp();
    }
    void TearDown() override
    {
        sensor->tearDown();
    }
};

// Test: setUp and tearDown
/*TEST_F(G3T1_3Fixture, TestSetUpAndTearDown)
{
    EXPECT_NO_THROW(sensor->setUp());
    EXPECT_NO_THROW(sensor->tearDown());
}
TEST_F(G3T1_3Fixture, TestGetPatientData)
{
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<services::PatientData>("getPatientData");
    services::PatientData srv;

    // Ensure service is available before calling
    ASSERT_TRUE(client.waitForExistence(ros::Duration(5.0))) << "Service 'getPatientData' not available";

    srv.request.vitalSign = "temperature";
    ASSERT_TRUE(client.call(srv)) << "Service call failed for 'temperature'";
    EXPECT_EQ(srv.response.data, 37.0);

    srv.request.vitalSign = "heartRate";
    ASSERT_TRUE(client.call(srv)) << "Service call failed for 'heartRate'";
    EXPECT_EQ(srv.response.data, 75.0);

    srv.request.vitalSign = "unknown";
    ASSERT_TRUE(client.call(srv)) << "Service call failed for 'unknown'";
    EXPECT_EQ(srv.response.data, -1.0);
}
// Test: collect
TEST_F(G3T1_3Fixture, TestCollect)
{
    double data = 0;
    EXPECT_NO_THROW(data = sensor->collect());
    EXPECT_GE(data, 0); // Data should be non-negative
}

// Test: process
TEST_F(G3T1_3Fixture, TestProcess)
{
    double raw_data = 37.0; // Simulated raw data
    double filtered_data = 0;
    EXPECT_NO_THROW(filtered_data = sensor->process(raw_data));
    EXPECT_GT(filtered_data, 0); // Processed data should be greater than 0
}

// Test: transfer
TEST_F(G3T1_3Fixture, TestTransfer)
{
    double valid_data = 37.5; // Simulated valid data
    EXPECT_NO_THROW(sensor->transfer(valid_data));

    double invalid_data = -1.0; // Out of bounds risk
    EXPECT_THROW(sensor->transfer(invalid_data), std::domain_error);
}

// Main
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_g3t1_3");

    ros::AsyncSpinner spinner(1); // Start a spinner for ROS callbacks
    spinner.start();

    int result = RUN_ALL_TESTS();

    ros::shutdown(); // Ensure ROS is properly shut down
    return result;
}
*/
#include "component/g3t1_3/G3T1_3.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <messages/SensorData.h>
#include <services/PatientData.h>
#include <unistd.h>

void dummyCallback(const ros::MessageEvent<archlib::Event const> &event)
{
    // Dummy function that does nothing but ensures subscribers exist
}
class G3T1_3Tests : public ::testing::Test
{
protected:
    G3T1_3 *sensor_node;

    // Override the default SetUp function
    void SetUp() override
    {
        ros::param::set("/temperature_LowRisk", "36.0,37.0");
        ros::param::set("/temperature_MidRisk0", "37.1,38.0");
        ros::param::set("/temperature_HighRisk0", "38.1,39.0");
        ros::param::set("/temperature_MidRisk1", "39.1,40.0");
        ros::param::set("/temperature_HighRisk1", "40.1,41.0");

        ros::param::set("/lowrisk", "0,20");
        ros::param::set("/midrisk", "21,65");
        ros::param::set("/highrisk", "66,100");

        ros::param::set("/instant_recharge", true);
        ros::param::set("/start", true);

        int argc = 0;
        char **argv = nullptr;
        sensor_node = new G3T1_3(argc, argv, "test_g3t1_3");
        ROS_INFO("g3t1 initialized");
        // Mock subscribers to prevent blocking in Component::setUp
        ros::NodeHandle nh;
        nh.subscribe<archlib::Event>("collect_event", 10, &dummyCallback);                // Corrected type: archlib::Event
        nh.subscribe<archlib::Status>("collect_status", 10, &dummyCallback);              // Corrected type: archlib::Status
        nh.subscribe<archlib::EnergyStatus>("collect_energy_status", 10, &dummyCallback); // Corrected type: archlib::EnergyStatus

        // Allow some time for subscribers to connect
        ros::spinOnce(); // Process any pending messages
        usleep(1000);    // Allow time for subscribers to connect

        sensor_node->setUp(); // Initialize the node
        ROS_INFO("g3t1 setup");
    }

    // Tear down resources
    void TearDown() override
    {
        sensor_node->tearDown(); // Clean up the node
        delete sensor_node;
    }
    bool mockPatientDataService(services::PatientData::Request &req, services::PatientData::Response &res)
    {
        if (req.vitalSign == "temperature")
        {
            res.data = 37.5; // Simulated patient temperature
            return true;
        }
        return false;
    }
};
/*
TEST_F(G3T1_3Tests, TestCollectPatientData)
{
    ros::NodeHandle nh;

    // Mock the services/PatientData service
    auto patient_data_srv = nh.advertiseService<services::PatientData::Request, services::PatientData::Response>(
        "getPatientData",
        [this](services::PatientData::Request &req, services::PatientData::Response &res)
        {
            return this->mockPatientDataService(req, res);
        });

    ros::spinOnce();

    // Call the `collect` method of the G3T1_3 node
    double collected_data = sensor_node->collect();

    // Verify the result
    EXPECT_DOUBLE_EQ(collected_data, 37.5); // Assert that the collected data matches the mock response
}
*/
TEST_F(G3T1_3Tests, TestProcess)
{
    ROS_INFO("in testCASE");
    double input_data = 36.5;
    double processed_data = sensor_node->process(input_data);
    EXPECT_NEAR(processed_data, input_data, 0.1); // Verify processed data is close to input
}
/*
TEST_F(G3T1_3Tests, TestTransfer)
{
    ros::NodeHandle nh;
    auto sensor_pub = nh.advertise<messages::SensorData>("thermometer_data", 10);

    double input_data = 36.5;
    ASSERT_NO_THROW(sensor_node->transfer(input_data));

    // Sleep to allow message propagation
    usleep(10000);
    ros::spinOnce();

    // Here, you can use a subscriber to verify the message was published correctly if needed
}

TEST_F(G3T1_3Tests, TestRiskLabeling)
{
    double risk_low = 10.0;
    double risk_mid = 50.0;
    double risk_high = 90.0;

    std::string low_label = sensor_node->label(risk_low);
    EXPECT_EQ(low_label, "low");

    std::string mid_label = sensor_node->label(risk_mid);
    EXPECT_EQ(mid_label, "moderate");

    std::string high_label = sensor_node->label(risk_high);
    EXPECT_EQ(high_label, "high");
}
*/
// Main function to run all the tests
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_g3t1_3");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
