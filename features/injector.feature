Feature: Injection communication

	Scenario: Ensure the data is being injected into nodes /logger, /g3t1_6, /g3t1_5, /g3t1_4, /g3t1_3, /g3t1_2, /g3t1_1
		Given the /logger node is on
		When I check the topics being published
		Then the nodes /logger, /g3t1_6, /g3t1_5, /g3t1_4, /g3t1_3, /g3t1_2, /g3t1_1 should receive data
