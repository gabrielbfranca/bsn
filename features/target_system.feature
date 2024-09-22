Feature: Ensure that central hub receives data from body sensors and publishes to data access node

	Scenario: Check if sensors are publishing data
		Given the ROS environment is up
		When I check if the sensors are publishing data
		And I check if respective topics have data
		And I check if /g4t1 is receiving data
		Then the sensors should publish data on their respective topics

	Scenario: Check if the central hub (/g4t1) is publishing to /data_access
		Given the ROS environment is up
		When the /g4t1 node is publishing data
		And respective topics have data
		Then the /data_access node should received it
