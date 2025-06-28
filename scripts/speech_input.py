#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import os
import google.genai as genai
from dotenv import load_dotenv
import json

depth_busy = False  # Global flag to block user input during depth processing

def load_object_list_from_json(filename):
    # Load YOLO object classes from JSON file for validation
    path = os.path.join(os.path.dirname(os.path.dirname(__file__)), filename)
    with open(path, "r") as f:
        label_dict = json.load(f)
    return list(label_dict.values())

# List of objects dynamically from JSON
object_list = load_object_list_from_json("yolo_object_list.json")

def setup_gemini():
    """
    Load API key from .env and configure Gemini model.
    """
    # Initialize Gemini AI client with API key from environment
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
stop_sr_pub = rospy.Publisher("item_finder_sr_termination", String, queue_size=10)

def publish_log(message):
    """
    Log the message and also publish to item_finder_response topic.
    """
    # Send message to both log and TTS system
    response_pub.publish(message)

def extract_object_from_text(text):
    """
    Use Gemini to extract the main object from user input.
    """
    # Create prompt to extract object names using Gemini AI
    prompt = (
        f"From the following sentence, extract the main object, item, or thing the user is referring to. "
        f"Sentence: \"{text}\". "
        f"Reply with only the object name if it's one of the following types: {', '.join(object_list)}. "
        f"For example, if the user mentions 'iphone', respond with 'phone', or if they mention 'macbook', respond with 'laptop'. "
        f"If no object from the list or related type is mentioned, reply with 'No object extracted'."
    )

    try:
        # Send prompt to Gemini and return extracted object
        response = client.models.generate_content(
            model="gemini-2.5-flash-preview-05-20",
            contents=prompt,
        )
        return response.text.strip()
    except Exception as e:
        error_msg = f"Gemini API Error: {e}"
        rospy.logerr(error_msg)
        response_pub.publish(error_msg)
        return ""
    
def depth_status_callback(msg):
    # Handle depth processing status to manage speech recognition flow
    global depth_busy
    if msg.data == "depth_started":
        depth_busy = True
        rospy.loginfo("[Speech Listener] Depth processing started. Blocking input.")
        stop_sr_pub.publish("stop")   # stop speech recognition during depth processing
    elif msg.data == "depth_done":
        depth_busy = False
        rospy.loginfo("[Speech Listener] Depth processing complete. Ready for new input.")
        rospy.loginfo("Tell me what you want to find...")
        rospy.sleep(5.0)
        publish_log("Tell me what you want to find...")  # prompt user again
        stop_sr_pub.publish("start")  # resume speech recognition

def item_callback(msg):
    # Process user speech input and extract target object using Gemini AI
    global depth_busy
    if depth_busy:
        rospy.loginfo("[Speech Listener] Depth busy. Ignoring input.")
        response_pub.publish("Processing depth, please wait for response.")
        return
    
    user_input = msg.data
    rospy.loginfo(f"[User said] {user_input}")

    # Extract the object name based on the user input
    extracted = extract_object_from_text(user_input)

    # Check if the object is valid (it shouldn't be 'No object extracted' or an empty string)
    if extracted and extracted != "No object extracted" and extracted in object_list:
        rospy.loginfo(f"[Gemini extracted] Object: {extracted}")
        object_pub.publish(extracted)
        response_pub.publish(f"Finding Object: {extracted}")
        stop_sr_pub.publish(f"stop")

        # Stop listening after extracting the object
        rospy.loginfo("[Speech Listener] Object extraction complete. Stopping listener...")
    else:
        rospy.loginfo("[Gemini response] No object extracted.")
        rospy.loginfo("[Speech Listener] No valid object found. Continuing to listen...")

def speech_listener():
    # Initialize ROS node and set up all subscribers for speech processing
    rospy.init_node('speech_listener_gemini', anonymous=True)
    rospy.Subscriber("item_finder_input", String, item_callback)
    rospy.Subscriber("depth_status", String, depth_status_callback)
    
    publish_log("Tell me what you want to find...")
    rospy.spin()

if __name__ == "__main__":
    try:
        speech_listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Gemini speech listener node stopped.")
