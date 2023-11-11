#! /usr/bin/python3
import json

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

def connect():
    mac_address = rospy.get_param('/sphero/mac_address', default='E0:D2:31:6B:74:3C')
    # conf_file_path = rospy.get_param("/conf_file_path")
    # with open(conf_file_path, 'r') as f:
        # cfg = Box(json.load(f))

    # Connect:
    # sphero = SpheroMini(cfg.MAC_ADDRESS, verbosity = 4)
    sphero = SpheroMini(mac_address, verbosity = 4)
    # battery voltage
    sphero.getBatteryVoltage()
    print(f"Battery voltage: {sphero.v_batt}v")

    # firmware version number
    sphero.returnMainApplicationVersion()
    print(f"Firmware version: {'.'.join(str(x) for x in sphero.firmware_version)}")
    return sphero


def disconnect(sphero):
    sphero.sleep()
    sphero.disconnect()

#################################################################################
# Description: Callback function for straight motion server
# Input: distance in cm (int), heading in deg (int)
# Output: confirmation message (string)
#################################################################################

def handle_straight_motion(req):
    print("Distance: %s || Direction: %s" %(req.dist, req.dir))
    resp = "Moving ..."
    sphero.roll(req.dist, req.dir)
    sphero.wait(1)
    sphero.roll(0, 0)
    return(resp)

#################################################################################
# Description: Server for straight motion request
# Input: None
# Output: None
#################################################################################

def straight_motion_server():
    rospy.init_node('straight_motion_server')
    s = rospy.Service('straight_motion', StraightMotion, handle_straight_motion)
    print("Ready to move.")
    rospy.spin()

if __name__ == "__main__":
    global sphero
    sphero = connect()    
    initSphero(sphero)

    straight_motion_server()
    
    disconnect(sphero)