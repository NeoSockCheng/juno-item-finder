#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import requests
import numpy as np
import json
from std_msgs.msg import String
import base64


# === CONFIG ===
API_URL = "https://yzh70-depth-pro.hf.space/depth"
API_KEY = "mysecureapikey"

depth_result_pub = rospy.Publisher('/depth_estimation_result', String, queue_size=10)
tts_pub = rospy.Publisher('item_finder_response', String, queue_size=10)
depth_status_pub = rospy.Publisher('depth_status', String, queue_size=10)
# bridge = CvBridge()
target_object = "object"
latest_detection = None
latest_image = None #CHANGED

# === Subscriber callback for object name ===
def object_name_callback(msg):
    rospy.loginfo(f"todel object_name_callback called")
    global target_object
    target_object = msg.data
    rospy.loginfo(f"Target object set to: {target_object}")

# def detection_callback(msg):
#     rospy.loginfo(f"todel detection_callback called")
#     global latest_detection
#     try:
#         latest_detection = json.loads(msg.data)
#         rospy.loginfo(f"Received detection: {latest_detection}")
#     except Exception as e:
#         rospy.logerr(f"Failed to parse detection: {e}")

def detection_callback(msg): #CHANGED
    rospy.loginfo("detection_callback called")
    global latest_detection
    try:
        latest_detection = json.loads(msg.data)
        rospy.loginfo(f"Received detection: {latest_detection}")
        try_trigger_depth()
    except Exception as e:
        rospy.logerr(f"Failed to parse detection: {e}")

# def compressed_image_callback(msg):
#     rospy.loginfo(f"todel compressed_image_callback called")
#     global target_object, latest_detection

#     if not latest_detection:
#         rospy.logwarn("No detection data available yet.")
#         return
    
#     try:
#         # Convert CompressedImage to OpenCV image
#         np_arr = np.frombuffer(msg.data, np.uint8)
#         cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#         # Encode image again (as JPEG)
#         ret, jpeg = cv2.imencode('.jpg', cv_image)
#         if not ret:
#             rospy.logerr("JPEG encoding failed")
#             return
        
#          # Publish depth started
#         status_msg = String()
#         status_msg.data = "depth_started"
#         depth_status_pub.publish(status_msg)
#         rospy.loginfo("Depth process started – notifying speech input to block input.")

#         image_bytes = jpeg.tobytes()
#         image_base64 = base64.b64encode(image_bytes).decode('utf-8')

#         form_data = {
#             "image_base64": image_base64,
#             "detections": json.dumps([latest_detection])
#         }

#         # Prepare HTTP request
#         headers = {"x-api-key": API_KEY}
#         # files = {"file": ("frame.jpg", jpeg.tobytes(), "image/jpeg")} #####
#         # params = {"object_name": target_object}

#         speech_msg = String()
#         speech_msg.data = f"{target_object.capitalize()} detected. Estimating distance. Please stay still for up to 2 minutes."
#         tts_pub.publish(speech_msg)


#         response = requests.post(API_URL, headers=headers, data=form_data)

#         if response.ok:
#             result = response.json()
#             rospy.loginfo(f"Depth Estimation Result: {result}")

#             # Extract distance from the detected object
#             depth_results = result.get("depth_results", [])
#             if depth_results:
#                 distance = depth_results[0].get("distance_m")
#                 if distance is not None:
#                     speak_result = String()
#                     speak_result.data = f"The {target_object} is approximately {distance:.1f} meters away."
#                 else:
#                     speak_result = String(data=f"Unable to estimate the distance of {target_object}.")
#             else:
#                 speak_result = String(data=f"I couldn't find the {target_object}. Please try again.")
#             # Extract distance from the first detected object if available
#             # if 'detections' in result and len(result['detections']) > 0:
#             #     distance = result['detections'][0].get('distance_m', None)
#             #     if distance is not None:
#             #         rospy.loginfo(f"{target_object} is {distance:.2f} meters away")
#             #         speak_result = String()
#             #         speak_result.data = f"The {target_object} is approximately {distance:.1f} meters away."
               
#             #     else:
#             #         rospy.loginfo(f"Distance of {target_object} is not detected.")
#             #         speak_result = String()
#             #         speak_result.data = f"Unable to estimate the distance of {target_object}. Please try again."
                 
#             # else:
#             #     rospy.loginfo(f"No detections found for {target_object}.")
#             #     speak_result = String()
#             #     speak_result.data = f"I couldn't find the {target_object} to estimate the distance. Please try again."
                
#             tts_pub.publish(speak_result)
#             rospy.sleep(5.0)
#             # Publish full JSON result of the depth estimation
#             result_msg = String()
#             result_msg.data = json.dumps(result)
#             depth_result_pub.publish(result_msg)

#         else:
#             rospy.logerr(f"HTTP Error {response.status_code}: {response.text}")
           
#         status_msg.data = "depth_done"
#         depth_status_pub.publish(status_msg)
#         rospy.loginfo("Depth process complete – notifying speech input to resume input.")
#     except Exception as e:
#         rospy.logerr(f"Error processing image: {e}")

def try_trigger_depth(): #CHANGED
    global latest_image, latest_detection, target_object

    if not latest_image or not latest_detection:
        return

    rospy.loginfo("Both image and detection received — starting depth estimation")

    msg = latest_image  # use and clear it
    latest_image = None
    detection = latest_detection
    latest_detection = None

    try:
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        ret, jpeg = cv2.imencode('.jpg', cv_image)
        if not ret:
            rospy.logerr("JPEG encoding failed")
            return

        status_msg = String(data="depth_started")
        depth_status_pub.publish(status_msg)

        tts_pub.publish(String(data=f"{target_object.capitalize()} detected. Estimating distance. Please stay still for up to 2 minutes."))

        image_bytes = jpeg.tobytes()
        image_base64 = base64.b64encode(image_bytes).decode('utf-8')
        form_data = {
            "image_base64": image_base64,
            "detections": json.dumps([detection])
        }

        headers = {"x-api-key": API_KEY}
        response = requests.post(API_URL, headers=headers, data=form_data)

        if response.ok:
            result = response.json()
            depth_results = result.get("depth_results", [])
            if depth_results and (distance := depth_results[0].get("distance_m")) is not None:
                speak_result = f"The {target_object} is approximately {distance:.1f} meters away."
            else:
                speak_result = f"I couldn't estimate the distance to the {target_object}."

            tts_pub.publish(String(data=speak_result))
            rospy.sleep(5.0)
            depth_result_pub.publish(String(data=json.dumps(result)))

        else:
            rospy.logerr(f"HTTP Error {response.status_code}: {response.text}")

        depth_status_pub.publish(String(data="depth_done"))
    except Exception as e:
        rospy.logerr(f"Error processing depth: {e}")


def compressed_image_callback(msg): #CHANGED
    rospy.loginfo("compressed_image_callback called")
    global latest_image
    latest_image = msg
    try_trigger_depth()


def main():
    rospy.init_node('depth_estimation_node')
    rospy.Subscriber('/detected_object_image/compressed', CompressedImage, compressed_image_callback)
    rospy.Subscriber('item_finder_object', String, object_name_callback)
    rospy.Subscriber('detected_object_bbox', String, detection_callback)
    rospy.loginfo("Subscribed and ready for depth estimation")
    rospy.spin()

if __name__ == '__main__':
    main()