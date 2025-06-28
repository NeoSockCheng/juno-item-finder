#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
import time
import json

model = YOLO('yolov8n.pt')

# Global state
target_object = None
last_detection_time = None
waiting_for_detection = False
object_captured = False 

# Publisher for cropped detected target object image (compressed using JPEG)
detected_obj_pub = rospy.Publisher('/detected_object_image/compressed', CompressedImage, queue_size=10)
tts_pub = rospy.Publisher('item_finder_response', String, queue_size=10)
bbox_pub = rospy.Publisher('detected_object_bbox', String, queue_size=10)

def keyword_callback(msg):
    # Set target object for detection and reset detection state
    global target_object, last_detection_time, waiting_for_detection, object_captured
    target_object = msg.data
    last_detection_time = time.time()
    waiting_for_detection = True
    object_captured = False
    rospy.loginfo(f"Target object set to: {target_object}")

def ros_img_to_cv2(msg):
    # Convert ROS Image message to OpenCV format for processing
    try:
        # Convert ROS Image message to numpy array manually
        dtype = np.uint8
        img_np = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width, 3))
        if msg.encoding == 'rgb8':
            img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
        return img_np
    except Exception as e:
        rospy.logerr(f"Failed to convert image: {e}")
        return None

def publish_compressed_image(image):
    # Compress and publish detected object image for depth estimation
    try:
        # Encode image to JPEG
        success, encoded_img = cv2.imencode('.jpg', image)
        if not success:
            rospy.logerr("JPEG encoding failed.")
            return

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = encoded_img.tobytes()

        detected_obj_pub.publish(msg)
        rospy.loginfo("Published target object image (compressed).")
    except Exception as e:
        rospy.logerr(f"Error publishing compressed image: {e}")

def image_callback(msg):
    # Process camera frames with YOLO detection and handle target object detection
    global target_object, last_detection_time, waiting_for_detection, object_captured

    frame = ros_img_to_cv2(msg)
    if frame is None:
        return

    # Run YOLO detection on current frame
    results = model(frame, verbose=False)
    detected = False

    for result in results:
        for box in result.boxes:
            cls_id = int(box.cls)
            confidence = float(box.conf)
            label = model.model.names[cls_id]

            if target_object and label == target_object and confidence > 0.7:
                detected = True
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {confidence:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Only capture and publish once
                if not object_captured:
                    rospy.loginfo("Target detected.")
                    rospy.loginfo(f"Target '{target_object}' detected with confidence {confidence:.2f}")
                    publish_compressed_image(frame)
                    bbox_data = {
                        "class_name": label,
                        "bounding_box": {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
                    }
                    bbox_msg = String()
                    bbox_msg.data = json.dumps(bbox_data)
                    bbox_pub.publish(bbox_msg)

                    object_captured = True
                    waiting_for_detection = False

    # Check for timeout
    if waiting_for_detection and (time.time() - last_detection_time > 20):
        rospy.logwarn("Timeout: Target object not found in 20 seconds.")

        # Tell the user to try again via TTS
        msg = String()
        msg.data = "I couldn't find the object. Please say the object you want me to find again."
        tts_pub.publish(msg)

        # Reset state
        target_object = None
        waiting_for_detection = False
        object_captured = False

    if detected:
        cv2.imshow("YOLOv8 Detection", frame) 
        cv2.waitKey(1)                         

def object_detection():
    # Initialize ROS node and set up subscribers for object detection
    rospy.init_node('object_detection_node', anonymous=True)
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    rospy.Subscriber('item_finder_object', String, keyword_callback)
    rospy.loginfo("YOLOv8 Detector Node Started. Waiting for images...")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        object_detection()
    except rospy.ROSInterruptException:
        rospy.loginfo("YOLOv8 detector node stopped.")