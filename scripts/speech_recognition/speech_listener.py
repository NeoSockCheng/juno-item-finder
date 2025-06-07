#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo("Heard: %s", msg.data)

def speech_listener():
    rospy.init_node('speech_listener', anonymous=True)
    rospy.Subscriber("user_voice_input", String, callback)
    rospy.loginfo("Speech listener is running. Waiting for speech...")
    rospy.spin()

if __name__ == "__main__":
    try:
        speech_listener()
    except rospy.ROSInterruptException:
        pass
