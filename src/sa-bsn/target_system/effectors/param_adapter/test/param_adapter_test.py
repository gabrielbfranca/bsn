#!/usr/bin/env python

import rospy
import unittest
from archlib.srv import EffectorRegister
from archlib.msg import AdaptationCommand

class TestParamAdapter(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_param_adapter', anonymous=True)
        cls.service_name = EffectorRegister
        # Wait for the service to be available
        try:
            rospy.wait_for_service(cls.service_name, timeout=10)  # Increase the timeout if needed
        except rospy.ROSException as e:
            rospy.logerr("Service %s is not available: %s", cls.service_name, str(e))
            raise e

    
    def test_effector_register_service(self):
        """Test if the EffectorRegister service is correctly advertised"""
        service_names = rospy.get_service_names()
        self.assertIn(self.service_name, service_names, "EffectorRegister service is not advertised")

    def test_adaptation_command_handling(self):
        """Test the handling of AdaptationCommand messages"""
        pub = rospy.Publisher('reconfigure', AdaptationCommand, queue_size=10)
        command = AdaptationCommand()
        command.target = "target_name"
        pub.publish(command)
        rospy.sleep(1)  # Give some time for the message to be processed
        # Implement checks to verify the behavior after message handling

if __name__ == '__main__':
    import rostest
    rostest.rosrun('effector', 'test_param_adapter', TestParamAdapter)
