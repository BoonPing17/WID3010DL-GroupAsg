#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import csv
import datetime
import os

class MainNode:
    def __init__(self):
        rospy.init_node('main_node')
        self.LOG_FILE = os.path.join(os.getcwd(), "results", "class_engagement_log.csv")

        # Publishers
        self.emotion_request_pub = rospy.Publisher('/emotion_request', String, queue_size=10)
        self.tts_pub = rospy.Publisher('/tts_text', String, queue_size=10)

        # Subscribers
        rospy.Subscriber('/speech_text', String, self.handle_speech_input)
        rospy.Subscriber('/emotion_result', String, self.handle_emotion_result)

        self.latest_instructor_query = ""

    def handle_speech_input(self, msg):
        self.latest_instructor_query = msg.data
        rospy.loginfo(f"Received instructor query: {self.latest_instructor_query}")

        # Trigger emotion detection
        rospy.loginfo("Triggering emotion analysis...")
        self.emotion_request_pub.publish("start_emotion_detection")

    def handle_emotion_result(self, msg):
        rospy.loginfo(f"Received emotion data: {msg.data}")

        feedback_text = f"Noted. {msg.data}"

        # Log the result
        self.log_engagement(msg.data)

        # Publish feedback to TTS
        self.tts_pub.publish(feedback_text)

    def log_engagement(self, emotion_result):
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # Ensure directory exists
        os.makedirs(os.path.dirname(self.LOG_FILE), exist_ok=True)

        # Create log file with header if it doesn't exist
        file_exists = os.path.isfile(self.LOG_FILE)
        with open(self.LOG_FILE, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(['Timestamp', 'Emotion Result'])
            writer.writerow([timestamp, emotion_result])

        rospy.loginfo(f"Logged engagement data at {timestamp} to {self.LOG_FILE}")

if __name__ == '__main__':
    try:
        MainNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Main Node terminated.")


