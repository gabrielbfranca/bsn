Feature: Patient Health Status (BSN-P10) - Whether the bodyhub has processed some data, it eventually will detect a new patient health status.

	Scenario: Successful Health Status (Happy Path)
		Given nodes are online:
			| Nodes         |
			| g4t1      	|
			| collector     |
			| param adapter |
			| g3t1_3     	|
		When g3t1_3 sends a collection of data
		Then g4t1 will detect new patient health status

	Scenario: Failure to Detect Health Status (Sad Path)
		Given nodes are online:
			| Nodes         |
			| g4t1      	|
			| collector     |
			| param adapter |
			| g3t1_3     	|
		And the bodyhub has processed patient data
		When g3t1_3 sends a collection of data
		And an internal processing error occurs in g4t1
		Then g4t1 will fail to detect the new patient health status
Feature: BSN-P03: Whenever the patients' health status is on high risk and an
	emergency has been detected it implies that is less or equal 250 (ms)

	Scenario: Successful Sensor Execution
		Given nodes are online:
			| Nodes         |
			| g4t1      	|
			| collector     |
			| param adapter |
			| g3t1_3     	|
		When g3t1_3 sends data with high risk
		Then g4t1 will detect an emergency in less than 250 ms

	Scenario: Overloaded sensor data
		Given nodes are online:
			| Nodes         |
			| g4t1      	|
			| collector     |
			| param adapter |
			| g3t1_3     	|
		And the node Patient Data is inactive
		When g3t1_3 sends low-risk data at 10x normal frequency and then suddenly sends high-risk data
		Then g4t1 might experiernce delayed emergency detection