#!/usr/bin/env python

"""
	A simple chatbot python file which implements Google TTS and Google SR
	- roslaunch usb_cam usb_cam-test.launch
	- rosrun rchomeedu_vision take_photo_sub.py
	- rosrun robotedge_speech google_sr.py
	- rosrun robotedge_speech simple_chatbot.py
	
"""

import rospy
from std_msgs.msg import String
import sys
from gtts import gTTS
import os

class SpeechRecognition:
	def __init__(self):
		rospy.init_node('speechrecognition')
		rospy.sleep(1)
		
		rospy.loginfo("Ready, waiting for commands...")

        # Google SR subsriber
		self.google_sr = rospy.Subscriber("googlesr", String, self.talkback)

	def loggging(self, msg):
		# Print the recognized words on the screen
		# msg.data=msg.data.lower()
		rospy.loginfo(msg.data)
		
if __name__=="__main__":
	try:
		SpeechRecognition()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Chatbot node terminated.")
