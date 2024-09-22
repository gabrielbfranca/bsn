Feature: Ensure that data_access receives data from /g4t1 and /logger nodes

	Scenario: Ensure /data_access receives data from /g4t1 node
		Given the ROS environment is up
		When I check if /g4t1 is publishing data
		Then /data_access should receive data from /g4t1

	Scenario: Ensure /data_access receives data from /logger node
		Given the ROS environment is up
		When I check if /logger is publishing data
		Then /data_access should receive data from /logger
