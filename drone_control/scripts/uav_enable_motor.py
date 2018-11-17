#!/usr/bin/env python
from hector_uav_msgs.srv import EnableMotors
import rospy

if __name__ == '__main__':
    print('waiting for service')
    rospy.wait_for_service('enable_motors')
    print("sending service")
    call_service = rospy.ServiceProxy('enable_motors', EnableMotors)
    response = call_service(True)
    print(response)
