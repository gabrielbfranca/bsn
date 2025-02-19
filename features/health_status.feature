Feature: Patient Health Status (BSN-P10) - Whether the bodyhub has processed some data, it eventually will detect a new patient health status.

	@reduced_system
	Scenario: Successful Health Status (Happy Path)
		Given nodes are online:
			| Nodes         |
			| collector     |
			| param adapter |
			| g3t1_3     	|
			| g4t1          |
			|patient data service|
		When I listen to topics:
			| Topic Name        |
			| thermometer data  |
			| Target System Data|
		Then g4t1 will detect new patient health status

	@reduced_system
	Scenario: Failure to Detect Health Status (Sad Path)
		Given nodes are online:
			| Nodes         |
			| g4t1      	|
			| collector     |
			| param adapter |
			| g3t1_3     	|
		When I listen to topics:
			| Topic Name        |
			| thermometer data  |
			| Target System Data|
		And the bodyhub has processed patient data
		When g3t1_3 sends a collection of data
		And an internal processing error occurs in g4t1
		Then g4t1 will fail to detect the new patient health status