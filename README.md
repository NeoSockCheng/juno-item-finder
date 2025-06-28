# Juno Vision Guide

A sophisticated ROS-based vision assistant for intelligent object detection and distance estimation, built for integration with the Juno robot platform. This project combines Google Gemini AI, YOLOv8 object detection, and depth estimation to provide voice-controlled object finding capabilities with natural language interaction.

##  System Overview

The Juno Vision Guide implements a distributed ROS architecture with 5 interconnected nodes:

1. **Speech Recognition** - Captures voice commands using Google Speech Recognition
2. **AI Speech Processing** - Uses Google Gemini to extract object names from natural language
3. **Object Detection** - Real-time YOLOv8-based detection with 80+ object classes
4. **Depth Estimation** - Distance calculation using external Depth Pro API
5. **Text-to-Speech** - Provides voice feedback using Google TTS

###  Key Features
- **Voice-controlled object finding** - "Find my phone", "Where is my laptop?"
- **Real-time visual detection** - Live camera feed with bounding box overlays
- **Distance estimation** - Accurate depth measurements in meters
- **Natural language processing** - Understands conversational requests
- **Hands-free operation** - Complete audio interaction workflow

##  Development Environment

- OS: Ubuntu 18.04  
- ROS: Noetic Ninjemys  
- Python: >= 3.10  
- Environment: Anaconda Virtual Environment  
- Editor: Visual Studio Code  

##  Environment Setup

### 1. Install ROS Noetic on Ubuntu 18.04
Follow the official guide: http://wiki.ros.org/noetic/Installation/Ubuntu

### 2. Create a New ROS Workspace
To avoid conflicts with the default workspace:
```bash
$ mkdir -p ~/catkin_ws_2/src
$ cd ~/catkin_ws_2
$ catkin_make
```

### 3. Create the ROS Package
```bash
$ cd ~/catkin_ws_2/src/
$ catkin_create_pkg juno_vision_guide rospy roscpp std_msgs
```

### 4. Build and Source the Workspace
```bash
$ cd ~/catkin_ws_2
$ catkin_make
$ echo "source ~/catkin_ws_2/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

## 🚀 Project Setup

### 1. Clone the Repository

```bash
$ cd ~/catkin_ws_2/src/
$ git clone https://github.com/NeoSockCheng/juno-vision-guide.git
$ cd juno-vision-guide
```

### 2. Install Anaconda (Skip if already installed)
Download from: https://www.anaconda.com/products/distribution

Install:
```bash
$ bash ~/Downloads/anaconda_distribution.sh
```

Add Anaconda to PATH:
```bash
$ echo "export PATH=/home/<your-username>/anaconda3/bin:$PATH" >> ~/.bashrc
$ source ~/.bashrc
```

### 3. Create and Activate Conda Environment
```bash
$ conda -V # Check conda installation
$ conda env create -f environment.yml
$ conda activate juno_vision_guide
```

### 4. Build Catkin Workspace
```bash
$ cd ~/catkin_ws_2
$ catkin_make
```

## 🔑 API Keys Setup

The system requires Google Gemini API key (free of charge) for full functionality:

1. Visit https://aistudio.google.com/app/apikey
2. Sign in with your Google account
3. Generate an API key and copy it
4. Replace `your-gemini-api-key-placeholder` in the `.env` file with your actual key

**Depth Pro Hosting**: We host the Depth Pro model on Hugging Face because it requires GPU to run:
https://huggingface.co/spaces/yzh70/depth-pro/tree/main.

## 🚀 Usage

### Quick Start
1. **Start ROS core** (Terminal 1):
```bash
$ roscore
```

2. **Launch the complete system** (Terminal 2):
```bash
$ cd ~/catkin_ws_2
$ source devel/setup.bash
$ roslaunch juno_vision_guide juno_vision_guide.launch
```

3. **Start using voice commands**:
   - Wait for the prompt: "Tell me what you want to find..."
   - Say something like: "Find my phone" or "Where is my laptop?"
   - The system will detect, locate, and estimate distance to the object

### Voice Command Examples
- "Find my phone" → Detects cell phone
- "Where is my laptop?" → Detects laptop  
- "Show me the bottle" → Detects bottle
- "Find the chair" → Detects chair
Full object list can be found in `yolo_object_list.json`

### System Workflow
1. **Voice Input** - Speak your request naturally
2. **AI Processing** - Gemini extracts the target object
3. **Visual Detection** - YOLOv8 finds the object in camera feed
4. **Distance Calculation** - Depth Pro estimates distance
5. **Voice Response** - System announces results
6. **Loop to Next Query** - Once complete, the system prompts for the next object to find automatically

## 🔧 Configuration

### Camera Setup
- Default camera device index: `1` (configured in `google_sr.py`)
- Modify `device_index` parameter if using different camera
- Ensure USB camera is connected and accessible

### Audio Setup
- Microphone device index: `1` (configured in `google_sr.py`)
- Check available microphones with: `python -c "import speech_recognition as sr; print(sr.Microphone.list_microphone_names())"`
- Audio output via `mpg321` - ensure speakers/headphones are connected

### Detection Parameters
- **Confidence threshold**: 70% (adjustable in `object_detection.py`)
- **Detection timeout**: 20 seconds
- **Supported objects**: 80 YOLO classes (see `yolo_object_list.json`)

## 📊 System Architecture

### ROS Topics
- `item_finder_input` - Raw speech recognition results
- `item_finder_object` - Extracted target object names
- `item_finder_response` - System responses for TTS
- `detected_object_bbox` - Object detection bounding boxes
- `detected_object_image/compressed` - Detected object images
- `depth_status` - Depth processing state management
- `item_finder_sr_termination` - Speech recognition control

### Node Communication Flow
```
Speech Recognition → Speech Processing (Gemini AI) → Object Detection (YOLOv8) → Depth Estimation → Text-to-Speech
```

## 🛠️ Troubleshooting

### Common Issues

**Camera not detected:**
- Check USB camera connection
- Verify camera device index in `google_sr.py`
- Test camera with: `rostopic echo /usb_cam/image_raw`

**Audio issues:**
- Verify microphone permissions
- Check audio device indices with `speech_recognition`
- Ensure `mpg321` is installed for audio playback

**API errors:**
- Verify `.env` file contains valid API keys
- Check internet connection for API access
- Monitor API rate limits and quotas

**Object not detected:**
- Ensure object is in YOLO's 80-class list
- Improve lighting conditions
- Adjust confidence threshold if needed
- Check camera focus and positioning

## Project Structure

```
juno-vision-guide/
├── launch/
│   └── juno_vision_guide.launch    # ROS launch configuration
├── scripts/
│   ├── google_sr.py               # Speech recognition node
│   ├── google_tts.py              # Text-to-speech node  
│   ├── speech_input.py            # AI speech processing node
│   ├── object_detection.py        # YOLOv8 detection node
│   ├── object_depth_estimation.py # Depth estimation node
│   └── .env                       # API keys for Gemini and Depth Pro
├── CMakeLists.txt                 # CMake build configuration
├── package.xml                    # ROS package metadata
├── environment.yml               # Conda environment dependencies
├── yolo_object_list.json         # YOLO class mappings
├── yolov8n.pt                    # YOLOv8 model weights
└── README.md                     # This file
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- **YOLOv8** by Ultralytics for object detection
- **Google Gemini AI** for natural language processing
- **ROS Community** for the robotics framework
- **OpenCV** for computer vision capabilities