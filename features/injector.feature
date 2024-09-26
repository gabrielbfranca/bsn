Feature: ensure simulation components communicate properly

	Scenario: Ensure the data is being injected into nodes /logger, /g3t1_6, /g3t1_5, /g3t1_4, /g3t1_3, /g3t1_2, /g3t1_1
		Given the /injector node is online
		When I check if topics /uncertainty_/node_name,/log_uncertainty are outbound to /param_adapter
		Then /injector node is connected appropriately
