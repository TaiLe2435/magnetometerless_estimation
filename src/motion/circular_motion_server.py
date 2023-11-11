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

from sphero_mini.srv import CircularMotion
from sphero_mini.srv import CircularMotionRequest
from sphero_mini.srv import CircularMotionResponse

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

def handle_circular_motion(req):
    # move robot with constant linear and angular velocity
    lin_vel = req.v
    print("lin_vel: %s" %lin_vel)
    ang_vel = lin_vel * req.r
    print("ang_vel: %s" %ang_vel)
    heading = 0
    flag = 0
    
    while heading < 360*3:
        heading += ang_vel * req.dt

        if heading > 360:
            heading = 0
            flag += 1
        
        if flag == 3:
            break

        sphero.roll(lin_vel, int(heading))
        sphero.wait(req.dt)
        print("heading: %s" %heading)

    sphero.roll(0,0)
    sphero.wait(1)
    print("v = %s | r = %s | dt = %s" %(req.v, req.r, req.dt) )
    resp = "Circles" 
    return(resp)

def circular_motion_server():
    rospy.init_node('circular_motion_server')
    s = rospy.Service('circular_motion', CircularMotion, handle_circular_motion)
    print("Ready to move.")
    rospy.spin()

if __name__ == "__main__":
    global sphero
    sphero = connect()    
    initSphero(sphero)

    circular_motion_server()
    
    disconnect(sphero)