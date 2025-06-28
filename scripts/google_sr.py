#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import speech_recognition as sr
import sounddevice

def googlesr():
    # Initialize ROS node and set up publishers/subscribers for speech recognition
    rospy.init_node('google_sr', anonymous=True)
    rospy.Subscriber("item_finder_sr_termination", String, callback)
    pub = rospy.Publisher('item_finder_input', String, queue_size=10)

    while not rospy.is_shutdown():
        # Capture audio from microphone and process speech
        r = sr.Recognizer()
        
        with sr.Microphone(device_index=1) as source:
            print(">>> Tell me what you want to find!")
            r.adjust_for_ambient_noise(source)
            audio = r.listen(source)
            print("Audio captured, processing...")
            
        # Convert speech to text using Google Speech Recognition API
        try:
            result = "I heard you are saying: " + r.recognize_google(audio, language="en-MY") ###################
            print(result)
            pub.publish(result)
        except sr.UnknownValueError:
            result = "I could not understand your audio, please try again."
            print(result)
        except sr.RequestError as e:
            result = "Could not request results from Google Speech Recognition service."
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
            
def callback(msg):
    # Handle termination signal to stop speech recognition
    if msg=="stop":
        rospy.loginfo("[Speech Listener] Object extraction complete. Stopping speech recognition...")
        rospy.signal_shutdown("Object extraction complete.")

if __name__ == "__main__":
    try:
        print(f"Available mics: {sr.Microphone.list_microphone_names()}")
        rospy.loginfo(f"Available mics: {sr.Microphone.list_microphone_names()}")
        googlesr()
    except rospy.ROSInterruptException:
        pass