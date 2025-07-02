#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import speech_recognition as sr


class STTNode:

    def __init__(self):
        rospy.init_node('stt_node')
        self.stt_publisher = rospy.Publisher('/speech_text', String, queue_size=10)
        rospy.loginfo("STT Node started. Waiting for instruction...")

        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()

        self.get_instruction()
    
    def get_instruction(self):
        rate = rospy.Rate(1)  # 1 Hz (adjust as needed)

        while not rospy.is_shutdown():
            with self.mic as source:
                print("Listening...")
                self.recognizer.adjust_for_ambient_noise(source)
                try:
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=7)
                except sr.WaitTimeoutError:
                    rospy.logwarn("No speech detected within timeout.")
                    continue

            try:
                text = self.recognizer.recognize_google(audio)
                rospy.loginfo("Recognized: %s", text)
                self.stt_publisher.publish(text)  # Send text to ROS topic
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand audio")
            except sr.RequestError as e:
                rospy.logerr("STT service error: {0}".format(e))

            rate.sleep()
    
if __name__ == '__main__':
    try:
        STTNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("STT Node is stopped")
