#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import google.genai as genai
from dotenv import load_dotenv

def setup_gemini():
    """
    Load API key from .env and configure Gemini model.
    """
    env_path = os.path.join(os.path.dirname(__file__), ".env")
    load_dotenv(dotenv_path=env_path)

    api_key = os.getenv("GEMINI_API_KEY")
    if not api_key:
        raise EnvironmentError("GEMINI_API_KEY not found in .env file.")
    client = genai.Client(api_key=api_key)
    return client

# Initialize Gemini model
client = setup_gemini()

# Publishers
object_pub = rospy.Publisher("item_finder_object", String, queue_size=10)
response_pub = rospy.Publisher("item_finder_response", String, queue_size=10)

def publish_log(message):
    """
    Log the message and also publish to item_finder_response topic.
    """
    rospy.loginfo(message)
    response_pub.publish(message)

def extract_object_from_text(text):
    """
    Use Gemini to extract the main object from user input.
    """
    prompt = (
        f"From the following sentence, extract the main object, item, or thing the user is referring to. "
        f"Sentence: \"{text}\". Reply with only the object name, nothing else."
    )

    try:
        response = client.models.generate_content(
            model="gemini-2.0-flash",
            contents=prompt,
        )
        return response.text.strip()
    except Exception as e:
        error_msg = f"Gemini API Error: {e}"
        rospy.logerr(error_msg)
        response_pub.publish(error_msg)
        return ""

def callback(msg):
    user_input = msg.data
    publish_log(f"[User said] {user_input}")

    extracted = extract_object_from_text(user_input)

    if extracted:
        publish_log(f"[Gemini extracted] Object: {extracted}")
        object_pub.publish(extracted)
    else:
        publish_log("[Gemini response] No object extracted.")

def speech_listener():
    rospy.init_node('speech_listener_gemini', anonymous=True)
    rospy.Subscriber("item_finder_input", String, callback)
    publish_log("Gemini speech listener running...")
    rospy.spin()

if __name__ == "__main__":
    try:
        speech_listener()
    except rospy.ROSInterruptException:
        publish_log("Gemini speech listener node stopped.")
