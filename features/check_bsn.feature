Feature: Check for bsn features

	@full_system
	Scenario: BSN-P09 If data has been sent by the sensor node, the BodyHub is able to process it as low, moderate or high risk vital sign data.
 	# Scenario: sucessful process
  		# Given that all sensors and central hub nodes are online
  		# When I listen to sensors data
    		# Then Sensors will process the risks
      		# And central hub will process the risk
		Given the Target System Data topic is online
		When I listen to topics:
			| Topic Name        |
			| thermometer data  |
			| ecg data          |
			| oximeter data     |
			| abps data         |
			| abpd data         |
			| glucosemeter data |
			| Target System Data|
		Then sensors will process the risks
		And Target System Data will receive the risks from sensors and detect patient's status
		
	@full_system
	Scenario: BSN-P08 If data has been sent by the sensor node, the BodyHub is able to process it
 	# Scenario: sucessful process
  		# Given that all sensors and central hub nodes are online
  		# When I listen to sensors data
    		# Then Sensors will process the data
      		# And central hub will will receive data from sensors
		Given the Target System Data topic is online
		When I listen to topics:
			| Topic Name        |
			| thermometer data  |
			| ecg data          |
			| oximeter data     |
			| abps data         |
			| abpd data         |
			| glucosemeter data |
			| Target System Data|
		Then sensors will process the data
		And Target System Data will receive the data from sensors

	@inactive_central_hub
	Scenario: BSN-P09 - Sad Path: central hub is inactive
 	# Scenario: central hub is inactive (Sad Path)
  		# Given that all sensors and central hub nodes are online
    		# And Central hub is inactive
  		# When I listen to sensors data
    		# Then Sensors will process the risks
      		# But central hub will not process the risk
		Given the Target System Data topic is online
		And the node g4t1 is inactive
		When I listen to topics:
			| Topic Name           |
			| thermometer data      |
			| ecg data              |
			| Target System Data     |
		Then sensors will process the risks
		And Target System Data should not receive any risk data or detect patient's status

	@inactive_central_hub
	Scenario: BSN-P08 - Sad Path: central hub is inactive
 	# Scenario: central hub is inactive (Sad Path)
  		# Given that all sensors and central hub nodes are online
    		# And Central hub is inactive
  		# When I listen to sensors data
    		# Then Sensors will process the data
      		# But central hub will not process the data
		Given the Target System Data topic is online
		And the node g4t1 is inactive
		When I listen to topics:
			| Topic Name            |
			| thermometer data      |
			| ecg data              |
			| Target System Data    |
		Then sensors will process the risks
		And Target System Data should not receive any risk data or detect patient's status
