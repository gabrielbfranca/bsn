Feature: BSN-P03: Whenever the patients' health status is on high risk and an emergency has been detected it implies that is less or equal 250 (ms)

	@reduced_system
	Scenario: Successful Sensor Execution
		# Given that nodes thermometer and central hub are online
		# When I listen to thermometer
		# And thermometer sends data with high risk
		# Then central hub will detect an emergency in less than 250 ms
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
		When g3t1_3 sends data with high risk
		Then g4t1 will detect an emergency in less than 250 ms

	@High_frequency_sensor_system
	Scenario: Overloaded sensor data
		# Given that nodes thermometer and central hub are online
		# When i listen to thermometer
		# And themometer sends low-risk data with high frequency
		# But thermometer sends data with high risk
		# Then Central Hub will experience delayed emergency detection
		Given nodes are online:
			| Nodes         |
			| g4t1      	|
			| collector     |
			| param adapter |
			| g3t1_3     	|
		And the node Patient Data is inactive
		When g3t1_3 sends low-risk data at 10x normal frequency and then suddenly sends high-risk data
		Then g4t1 might experience delayed emergency detection