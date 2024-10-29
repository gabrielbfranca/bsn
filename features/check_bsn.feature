Feature: Check for bsn features

	Scenario: BSN-P05 If a sensor reports a health status risk, an emergency will be detected in body hub
		Given the /TargetSystemData topic is online
		When I listen to topics:
			| Topic Name         | Data Type             |
			| /thermometer_data  | messages/SensorData   |
			| /ecg_data          | messages/SensorData   |
			| /oximeter_data     | messages/SensorData   |
			| /abps_data         | messages/SensorData   |
			| /abpd_data         | messages/SensorData   |
			| /glucosemeter_data | messages/SensorData   |
			| /TargetSystemData  | messages/SensorData   |
		Then sensors will process the risks
		And /TargetSystemData will receive the risks from sensors

	Scenario: BSN-P11 If data has been sent by the sensor node, the BodyHub is able to process it
		Given the /TargetSystemData topic is online
		When I listen to topics:
			| Topic Name         | Data Type             |
			| /thermometer_data  | messages/SensorData   |
			| /ecg_data          | messages/SensorData   |
			| /oximeter_data     | messages/SensorData   |
			| /abps_data         | messages/SensorData   |
			| /abpd_data         | messages/SensorData   |
			| /glucosemeter_data | messages/SensorData   |
			| /TargetSystemData  | messages/SensorData   |
		Then sensors will process the data
		And /TargetSystemData will receive the data from sensors
