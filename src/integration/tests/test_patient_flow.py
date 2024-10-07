#!/usr/bin/env python
import rospy
import unittest

class TestExample(unittest.TestCase):
    def test_node_existence(self):
        # Check if the ROS node is active (or ROS master is running)
        rospy.init_node('test_node', anonymous=True)
        self.assertTrue(rospy.is_shutdown() is False, "ROS master is not running!")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('my_test_package', 'test_example', TestExample)

"""
import rospy
import unittest
import rostest
from std_srvs.srv import Empty
from services.srv import PatientData
from std_msgs.msg import String

class TestPatientFlow(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_patient_flow')
        # Subscribe to g4t1 topic
        self.g4t1_data = None
        rospy.Subscriber("/g4t1", String, self.g4t1_callback)
        
        # Wait for PatientData service
        rospy.wait_for_service('/Patient/getPatientData')
        self.patient_service = rospy.ServiceProxy('/Patient/getPatientData', PatientData)

    def g4t1_callback(self, msg):
        self.g4t1_data = msg.data

    def test_patient_data_flow(self):
        # Call the PatientData service with a mock request
        request = PatientData.Request()
        request.vitalSign = "heart_rate"  # Example vital sign request
        response = self.patient_service(request)
        
        # Assert we got data from the service
        self.assertIsNotNone(response, "No response from PatientData service")
        
        # Give G4T1 time to process and publish data
        rospy.sleep(2)

        # Check that g4t1 node published processed data
        self.assertIsNotNone(self.g4t1_data, "No data published on /g4t1 topic")
        self.assertIn("risk", self.g4t1_data, "Risk data not correctly processed")

if __name__ == '__main__':
    rostest.rosrun('integration', 'test_patient_flow', TestPatientFlow)
"""
