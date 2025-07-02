#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import pyttsx3

class TTSNode:
    def __init__(self):
        rospy.init_node('tts_node')
        rospy.Subscriber('/tts_text', String, self.callback)
        rospy.loginfo("TTS Node started. Waiting for text...")
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)  # speed
        self.engine.setProperty('volume', 1.0)  # max volume

    def speak(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def callback(self, msg):
        text = msg.data
        rospy.loginfo(f"Received text for TTS: {text}")
        self.speak(text)

if __name__ == '__main__':
    try:
        TTSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TTS Node terminated.")
