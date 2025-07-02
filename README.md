# Classroom Emotion Detection Robot 🤖🎓

This project is a ROS-based multimodal assistant that listens to the lecturer, detects students' emotions using computer vision, and gives real-time spoken feedback. It also logs each session with a visual overlay and emotion summary.

---

## 🧠 What It Does

1. **STT Node**: Listens for lecturer instructions (e.g., “What’s the emotion of the class?”).
2. **Emotion Detection Node**: Uses a webcam to detect students' faces and emotions (from DeepFace) to educational insights.
3. **Main Node**: Orchestrates emotion analysis and logs results.
4. **TTS Node**: Speaks out the summary (e.g., “I see 3 students: 2 are engaged, 1 is curious”).
5. **Logs**: Saves results in `results/class_engagement_log.csv` and annotated images in `results/overlay_images`.

---

## 📦 Requirements

- ROS Noetic (Ubuntu 20.04)
- Python 3.8
- Python packages:
  - `speechrecognition`
  - `pyttsx3`
  - `opencv-python`
  - `deepface`
  - `cv_bridge`
  - `numpy`
  - `rospy`, `std_msgs`, `sensor_msgs` (ROS-native)

---

## 🚀 How to Launch

### 1. Install Dependencies

#### System Dependencies
```bash
sudo apt update
sudo apt install ros-noetic-usb-cam ros-noetic-cv-bridge python3-pip
```

#### Python packages
```bash
pip3 install deepface opencv-python pyttsx3 SpeechRecognition
```


### 2. Make Python Files Executable
```bash
chmod +x ~/catkin_ws/src/group6/src/*.py
```

### 3. Launch the Entire System
```bash
roslaunch group6 classroom_robot.launch
```

This will:
- Start the USB camera
- Launch all nodes in your project
- Begin listening for instructor voice prompts (e.g., “What’s the emotion of the class right now?”)

### 4. Output
- **CSV logs** are saved to: results/class_engagement_log.csv
- **Overlay images** are saved to: results/overlay_images/