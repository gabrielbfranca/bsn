Feature: Ensure managing system and logger components are communicating correctly

	Scenario: Check if /enactor communicates with /reli_engine and /logger
		Given the /enactor node is online
		When I check if topics /strategy are inbound and /exception are outbound to /reli_engine
		And topics /event are inbound and /log_adapt are outbound to /logger
		Then /enactor connections should have the respective topics

	Scenario: Check if /reli_engine communicates with /enactor
		Given the /reli_engine node is online
		When I check if topics /exception are inbound and /strategy are outbound to /enactor
		Then /reli_engine connections should have the respective topics

	Scenario: Check if /logger communicates with /collector, /param_adapter, /enactor, /data_access, /injector
		Given the /logger node is online
		When I check if topics /log_adapt are inbound and /event are outbound to /enactor
		And I check if topics / are inbound from /collector
		And I check if topics /log_uncertainty are inbound from /injector
		And I check if topics /reconfigure are outbound to /param_adapter
		And i check if topics /persist are outbound to /Data_access
		Then /logger node is connected appropriately
