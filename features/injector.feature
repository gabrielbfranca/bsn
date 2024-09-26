Feature: ensure simulation components communicate properly

	Scenario: Ensure the data is being injected into nodes /logger, /g3t1_6, /g3t1_5, /g3t1_4, /g3t1_3, /g3t1_2, /g3t1_1
		Given the /injector node is on
		When i check if each topic /uncertainty_/node_name and /log_uncertainty outbound to their nodes
		Then /injector is working properly
