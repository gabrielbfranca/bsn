Feature: Check for bsn features

	Scenario: BSN-P11 If data has been sent by the sensor node, the BodyHub is able to process it as low, moderate or high risk vital sign data.
		Given the /TargetSystemData topic is online
		When I listen to topics:
			| Topic Name         |
			| /thermometer_data  |
			| /ecg_data          |
			| /oximeter_data     |
			| /abps_data         |
			| /abpd_data         |
			| /glucosemeter_data |
			| /TargetSystemData  |
		Then sensors will process the risks
		And /TargetSystemData will receive the risks from sensors

	Scenario: BSN-P09 If data has been sent by the sensor node, the BodyHub is able to process it
		Given the /TargetSystemData topic is online
		When I listen to topics:
			| Topic Name         |
			| /thermometer_data  |
			| /ecg_data          |
			| /oximeter_data     |
			| /abps_data         |
			| /abpd_data         |
			| /glucosemeter_data |
			| /TargetSystemData  |
		Then sensors will process the data
		And /TargetSystemData will receive the data from sensors
