# Juno Item Finder
A ROS-based vision assistant for item recognition, built for integration with the Juno robot platform. This project uses Google Gemini API and Python to process visual input and assist in object detection or guidance.

# Juno Item Finder

A ROS-based vision assistant for item recognition, built for integration with the Juno robot platform. This project uses Google Gemini API and Python to process visual input and assist in object detection or guidance.

## 🛠️ Development Environment

- OS: Ubuntu 18.04  
- ROS: Noetic Ninjemys  
- Python: >= 3.10  
- Environment: Anaconda Virtual Environment  
- Editor: Visual Studio Code  

## ⚙️ Environment Setup

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
$ catkin_create_pkg juno_item_finder rospy roscpp std_msgs
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
$ cd ~/catkin_ws_2/src/juno_item_finder
$ git clone https://github.com/NeoSockCheng/juno-vision-guide.git
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
$ conda create -n yourenvname python=3.10 anaconda
$ source activate yourenvname
```

### 4. Install Python Dependencies
```bash
$ pip install -r ~/catkin_ws_2/src/juno_item_finder/requirements.txt
```

### 5. Build Catkin Workspace Again
```bash
$ cd ~/catkin_ws_2
$ catkin_make
```

## 🔑 Gemini API Setup

1. Create a `.env` file in your package directory.
2. Get your API key from: https://aistudio.google.com/app/apikey
3. Add the following to `.env`:
```plaintext
GEMINI_API_KEY=yourapikeyhere
```

## 🧪 Run the Application

In separate terminals, run the following:

Terminal 1:
```bash
$ roscore
```

Terminal 2:
```bash
$ roslaunch med_buddy med_buddy.launch
```

## 🧩 Dependencies

- rospy, roscpp, std_msgs
- Python packages listed in requirements.txt  
- Google Generative AI SDK