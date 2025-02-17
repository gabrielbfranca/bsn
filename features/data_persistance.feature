Feature: Data Persistence (BSN-P08) - Whether the sensor node has collected some data, eventually the bodyhub will persist it.

	Scenario: Data Persisted Successfully (Happy Path)
        
		Given the Logger, g4t1, collector, param adapter and g3t_ ?, data access node is online
		When collector receives collect_? topic
		Then the data will eventually be received in logger
		And the data will be in persist topic

	Scenario: Data Not Persisted (Sad Path)
		Given the Logger, g4t1, collector, param adapter and g3t_ ? and data access node is online
		And the sensor node has collected data
		When collector receives collect_? topic
		And a database error prevents persistence
		Then the data will not be persisted
		And the system must log a persistence failure
		And the system may attempt a retry