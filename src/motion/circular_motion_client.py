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

from sphero_mini.srv import CircularMotion
from sphero_mini.srv import CircularMotionRequest
from sphero_mini.srv import CircularMotionResponse

def circular_motion_client(v, r, dt):
    rospy.wait_for_service('circular_motion')
    try:
        circular_motion = rospy.ServiceProxy('circular_motion', CircularMotion)
        resp1 = circular_motion(v, r, dt)
        return resp1.confirmation
    except rospy.ServiceException(e):
        print ("Serice call failed : %s"%e)

if __name__ == "__main__":
    if len(sys.argv) == 4:
        v = int(sys.argv[1])
        r = int(sys.argv[2])
        dt = float(sys.argv[3])
    else:
        print ("%s [v r dt]"%sys.argv[0])
        sys.exit(1)
print ("Requesting v=%s r=%s dt=%s"%(v, r, dt))
s = circular_motion_client(v, r, dt)
print ("%s %s %s %s" %(v, r, dt, s))