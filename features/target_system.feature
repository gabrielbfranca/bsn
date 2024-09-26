Feature: Ensure that central hub receives data from body sensors and publishes to data access node

	Scenario: Check if sensors are publishing data to /collector and to /g4t1
		Given the ROS environment is on
		When I check if the sensors are publishing data
		And I check if respective topics have data
		Then /g4t1 should receive data

	Scenario: Check if the central hub (/g4t1) is publishing to /data_access
		Given the ROS environment is on
		When the /g4t1 node is publishing data
		And respective topics have data
		Then the /data_access node should received it

	Scenario: Check if /param_adapter node communicate with the g3t1_n, g4t1 and /logger nodes
		Given the /param_adapter node is online
		When I check if topic /reconfigure is inbound from /logger
		and I check if topics /reconfigure_/g3t1_n are outbound to /g3t1_n
		and I check if topic /reconfigure_/g4t1 are outbound to /g4t1
		Then the /param_adapter node is functional