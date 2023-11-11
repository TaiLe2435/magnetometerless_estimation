#! /usr/bin/python3
import json

import sys
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Int64, String, Bool
from box import Box
import numpy as np

from sphero_mini.simple_moves import forward, initSphero
from sphero_mini.core import SpheroMini

from sphero_mini.srv import StraightMotion
from sphero_mini.srv import StraightMotionRequest
from sphero_mini.srv import StraightMotionResponse

#################################################################################
# Description: Client that sends request to server for straight motion
# Input: distance in cm (int), heading in deg (int)
# Output: confirmation message (string)
#################################################################################

def straight_motion_client(dist, dir):
    rospy.wait_for_service('straight_motion')
    try:
        straight_motion = rospy.ServiceProxy('straight_motion', StraightMotion)
        resp1 = straight_motion(dist, dir)
        return resp1.confirmation
    except rospy.ServiceException(e):
        print ("Serice call failed : %s"%e)

if __name__ == "__main__":
    if len(sys.argv) == 3:
        dist = int(sys.argv[1])
        dir = int(sys.argv[2])
    else:
        print ("%s [x y]"%sys.argv[0])
        sys.exit(1)
print ("Requesting dist=%s dir=%s "%(dist, dir))
s = straight_motion_client(dist, dir)
print ("%s %s %s" %(dist, dir, s))