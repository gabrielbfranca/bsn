Feature: ROS Component Testing

	Scenario: Verify message publishing
		Given the sensor is initialized
		When I call the setup function
		Then the sensor should be properly configured
