#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from deepface import DeepFace
import numpy as np
from collections import Counter
import os
from datetime import datetime

# Custom emotion mapping
EMOTION_MAP = {
    'neutral': 'curious',
    'happy': 'engaged',
    'sad': 'bored',
    'angry': 'confused',
    'fear': 'confused',
    'surprise': 'interested',
}


class EmotionDetectorNode:
    def __init__(self):
        rospy.init_node('emotion_detector_node')
        self.bridge = CvBridge()
        self.detection_requested = False
        self.latest_frame = None

        # ROS setup
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.result_pub = rospy.Publisher("/emotion_result", String, queue_size=10)
        self.request_sub = rospy.Subscriber("/emotion_request", String, self.request_callback)
        self.overlay_pub = rospy.Publisher("/emotion_overlay_image", Image, queue_size=10)

        rospy.loginfo("EmotionDetectorNode started. Waiting for image data...")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_frame = frame
            if self.detection_requested:
                self.detection_requested = False  # reset flag immediately
                summary = self.analyze_frame(frame)
                rospy.loginfo(f"[Emotion Summary] {summary}")
                # self.result_pub.publish(summary)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def analyze_frame(self, frame):
        try:
            results = DeepFace.analyze(frame, actions=['emotion'], enforce_detection=False)

            if not isinstance(results, list):
                results = [results]

            emotions = []
            for face in results:
                raw_emotion = face['dominant_emotion']
                emotion = EMOTION_MAP.get(raw_emotion, raw_emotion)
                emotions.append(emotion)

                # Draw bounding box & label
                if 'region' in face:
                    x, y, w, h = face['region']['x'], face['region']['y'], face['region']['w'], face['region']['h']
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, emotion, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Publish annotated frame
            overlay_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.overlay_pub.publish(overlay_msg)

            # Save overlay image
            image_path = self.save_overlay_image(frame)

            return self.summarize_emotions(emotions)

        except Exception as e:
            rospy.logwarn(f"DeepFace error: {e}")
            return "Emotion detection failed."

    def summarize_emotions(self, emotion_list):
        total = len(emotion_list)
        if total == 0:
            return "No faces detected."

        counter = Counter(emotion_list)
        parts = [f"{count} is {emo}" for emo, count in counter.items()]
        summary = f"I see {total} student: " + ", ".join(parts) + "."
        return summary
    
    def request_callback(self, msg):
        if msg.data.strip().lower() == "start_emotion_detection":
            if self.latest_frame is not None:
                rospy.loginfo("[EmotionDetector] Triggered via request. Analyzing...")
                self.detection_requested = True
                summary = self.analyze_frame(self.latest_frame)
                self.result_pub.publish(summary)
            else:
                rospy.logwarn("No frame available for analysis.")

    def save_overlay_image(self, frame):
        save_dir = os.path.join(os.getcwd(), "results", "overlay_images")
        os.makedirs(save_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"overlay_{timestamp}.jpg"
        full_path = os.path.join(save_dir, filename)
        cv2.imwrite(full_path, frame)
        rospy.loginfo(f"[Overlay Saved] {full_path}")
        return full_path


if __name__ == '__main__':
    try:
        EmotionDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("EmotionDetectorNode stopped.")
