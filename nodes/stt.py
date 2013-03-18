#!/usr/bin/env python
import roslib; roslib.load_manifest("kitchencommunication")
import rospy
import time
import subprocess

from std_msgs.msg import String, Int16
from kitchencommunication.srv import *

#pub = rospy.Publisher("robot_response", String)

# 0 = inactive, 1 = active
talk_status = 0

def record_speech(x):
    p = subprocess.Popen("gspeett", stdout=subprocess.PIPE)
    txt, err = p.communicate()
    print txt
    return txt

def receive_command():
    rospy.init_node('stt')
    s = rospy.Service('record_speech', stt, record_speech)
    rospy.spin()

def activate_speech(onoff):
    global talk_status
    talk_status = onoff

if __name__ == "__main__":
    rospy.Subscriber("activate_speech", Int16, activate_speech)
    receive_command()
