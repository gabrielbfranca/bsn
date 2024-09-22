Feature: Ensure that /enactor receives and sends data from loggers, publishes data to /reli_engine, and ensures that reli engine receives data from enactor and publishes back to enactor

	Scenario: Check if /enactor receives data from loggers
		Given the ROS environment is up
		When I check if /loggers are publishing data
		Then /enactor should receive data from /loggers

	Scenario: Check if /enactor publishes data to /reli_engine
		Given the /enactor is receiving data from /loggers
		When /enactor processes the data
		Then /enactor should publish data to /reli_engine

	Scenario: Check if /reli_engine receives data from /enactor
		Given /enactor has published data to /reli_engine
		When I check if /reli_engine is receiving data
		Then /reli_engine should receive data from /enactor

	Scenario: Check if /reli_engine publishes data back to /enactor
		Given /reli_engine has received data from /enactor
		When /reli_engine processes the data
		Then /reli_engine should publish data back to /enactor
