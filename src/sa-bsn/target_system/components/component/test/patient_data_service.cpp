#include <ros/ros.h>
#include <services/PatientData.h> // Replace with the correct header if necessary

// Callback function for the 'getPatientData' service
bool mockPatientDataService(services::PatientData::Request &req, services::PatientData::Response &res)
{
    if (req.vitalSign == "temperature")
    {
        res.data = 41.0; // Simulated high-risk temperature
        ROS_INFO("Returning simulated temperature: %.2f", res.data);
        return true;
    }
    // Handle other vital signs here if needed
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "patient_data_service");
    ros::NodeHandle nh;

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("getPatientData", mockPatientDataService);
    ROS_INFO("Patient data service ready to provide data...");

    ros::spin(); // Keep the node running
    return 0;
}
