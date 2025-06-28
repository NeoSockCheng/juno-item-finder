#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from gtts import gTTS
import os

def callback(data):
    # Convert received text to speech and play audio output
    rospy.loginfo("Input: %s", data.data)

    text = data.data
    tts = gTTS(text, lang="en-US")
    
    # Save and play the generated speech audio
    tts.save("speech.mp3")
    os.system("mpg321 speech.mp3")
    
def googletts():
    # Initialize ROS node and subscribe to response messages for TTS
    rospy.init_node('google_tts', anonymous=True)

    rospy.Subscriber("item_finder_response", String, callback)

    rospy.spin()

if __name__ == "__main__":
    googletts()