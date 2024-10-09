#!/usr/bin/env python
import rospy
import unittest
from services.srv import PatientData


class TestPatientModule(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_patient_module', anonymous=True)
        rospy.wait_for_service('/getPatientData')

    def test_get_patient_data(self):
        try:
            get_patient_data = rospy.ServiceProxy('/getPatientData', PatientData)

            response = get_patient_data("heart_rate")

            # Ensure that the response is a float, or implement more specific checks based on your data
            self.assertTrue(isinstance(response.data, float))
            
            rospy.loginfo("Received response: {}".format(response.data))

        except rospy.ServiceException as e:
            self.fail("Service call failed: {}".format(e))

if __name__ == '__main__':
    import rostest
    rostest.rosrun('patient', 'test_patient_module', TestPatientModule)
