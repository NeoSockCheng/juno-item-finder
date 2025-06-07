#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import speech_recognition as sr
import sounddevice

def googlesr():
    rospy.init_node('google_sr', anonymous=True)
    pub = rospy.Publisher('item_finder_input', String, queue_size=10)

    while not rospy.is_shutdown():
        # obtain audio from the microphone
        r = sr.Recognizer()
        
        with sr.Microphone(device_index=0) as source:
            print(">>> Tell me what you want to find!")
            r.adjust_for_ambient_noise(source)
            audio = r.listen(source)
            # audio = r.record(source, duration=5)
            
        # recognize speech using Google Speech Recognition
        try:
            result = "I heard you are saying: " + r.recognize_google(audio, language="en-US")
            print(result)
            pub.publish(result)
        except sr.UnknownValueError:
            result = "I could not understand your audio, please try again."
            print(result)
            pub.publish(result)
        except sr.RequestError as e:
            result = "Could not request results from Google Speech Recognition service."
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
            pub.publish(result)

if __name__ == "__main__":
    try:
        print(f"Available mics: {sr.Microphone.list_microphone_names()}")
        # rospy.loginfo(f"Available mics: {sr.Microphone.list_microphone_names()}")
        googlesr()
    except rospy.ROSInterruptException:
        pass