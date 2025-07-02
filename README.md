# Jupiter Juno Driver Drowsiness Detection System

A ROS 1-based intelligent co-driver system that monitors driver alertness and provides timely interventions to enhance road safety.

## Prerequisites

- **Operating System**: Ubuntu 20.04 (or 18.04 for ROS Melodic)
- **ROS Version**: ROS 1 Noetic (or Melodic)
- **Python**: Python 3.8+
- **Hardware Requirements**:
  - Webcam/Camera for face detection
  - Microphone for voice input
  - Speakers/Headphones for audio output
- **Network**: Internet connection (required for gTTS and Gemini API calls)

### System Dependencies

```bash
# Update package list
sudo apt update

# Install system dependencies
sudo apt install -y \
    python3-pip \
    python3-opencv \
    python3-yaml \
    python3-numpy \
    python3-scipy \
    portaudio19-dev \
    espeak \
    ffmpeg \
    python3-pyqt5 \
    python3-pyqt5-dev \
    ros-noetic-cv-bridge \
    ros-noetic-rospy \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-rqt-gui \
    ros-noetic-rqt-gui-py
```

## Setup

### 1. Connect to Jupiter Juno Robot
Connect to your Jupiter Juno robot system and ensure ROS 1 Noetic is installed and properly configured.

### 2. Clone the Project
```bash
# Navigate to your catkin workspace source directory
mkdir ~/assignment
cd ~/assignment

# Clone the Jupiter Juno drowsiness detection system
git clone https://github.com/yangding14/robot_assignment.git
```

### 3. Make Scripts Executable
```bash
# Make Python node scripts executable
chmod +x ~/assignment/robot_assignment/src/jupiter_juno/src/jupiter_juno/*.py
chmod +x ~/assignment/robot_assignment/src/jupiter_juno/src/standalone_gui.py
chmod +x ~/assignment/robot_assignment/src/jupiter_juno/src/test_system_standalone.py
```

### 4. Add Package to ROS Environment
```bash
# Add the package path to ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/assignment/robot_assignment/jupiter_juno/src

# Add this line to your ~/.bashrc for persistence
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/assignment/robot_assignment/jupiter_juno/src" >> ~/.bashrc
```

### 5. Build the Package
```bash
# Navigate to catkin workspace root
cd ~/assignment/robot_assignment

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Build the Jupiter Juno package
catkin_make

# Source the workspace setup
source devel/setup.bash
```

### 6. Configure API Keys
```bash
# Set up Gemini API key for AI conversations (recommended)
export GEMINI_API_KEY="your-gemini-api-key-here"

source ~/.bashrc
```

**Note**: The system will work without API keys by using pre-programmed fallback jokes and safety tips instead of AI-generated conversations.

### 7. Setup Python Environment
```bash
# Navigate to the package directory
cd ~/assignment/robot_assignment/jupiter_juno

# Create and activate virtual environment (optional but recommended)
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
pip3 install -r requirements.txt
```

### 8. Launch the System
```bash
# Ensure you're in the catkin workspace and environment is sourced
cd ~/assignment/robot_assignment
source devel/setup.bash

# Launch the complete system with GUI monitor
roslaunch robot_assignment jupiter_juno_complete_system.launch
```

## Monitoring the System

### View Active Topics
```bash
# List all active ROS topics
rostopic list
```

### Monitor System Data
```bash
# Monitor real-time Eye Aspect Ratio values
rostopic echo /jupiter_juno/ear_data

# Monitor drowsiness detection alerts
rostopic echo /jupiter_juno/drowsiness_alert

# Monitor speech recognition results
rostopic echo /jupiter_juno/speech_result

# Monitor system state transitions
rostopic echo /jupiter_juno/system_state

# Monitor TTS requests
rostopic echo /jupiter_juno/tts_request
```

### Visualize System Architecture
```bash
# View ROS computation graph showing node connections
rqt_graph

# View message flow and topic relationships
rqt_plot
```

## Features

- **🔍 Real-time Eye Detection**: Advanced computer vision monitoring of driver's Eye Aspect Ratio (EAR) to detect drowsiness patterns
- **🚨 Multi-stage Alert System**: Progressive alerting starting with audio alerts, followed by engaging AI conversations to maintain driver attention
- **🎤 Voice Interaction**: Complete speech recognition and text-to-speech capabilities using Google TTS for natural communication
- **🤖 AI-Powered Conversations**: Intelligent conversations powered by Gemini API with OpenAI fallback, featuring context-aware responses and driving-focused content
- **🧩 Modular ROS Architecture**: Built with ROS 1 for seamless integration with robotic systems and hardware platforms
- **⚙️ Configurable Parameters**: Easily adjustable detection thresholds, alert settings, and conversation parameters via YAML configuration
- **📱 GUI Monitoring**: Comprehensive RQt-based graphical interface with real-time camera feed, EAR visualization, and system status
- **🔧 Hardware Integration**: Designed for integration with Jupiter Juno robot hardware including camera, microphone, and speaker systems
- **🌐 Robust Error Handling**: Graceful fallbacks for network issues, missing hardware, and API failures
- **📊 Real-time Data Logging**: Complete ROS topic-based data flow for monitoring, debugging, and system analysis

## System Architecture

The Jupiter Juno drowsiness detection system operates as a distributed ROS 1 application with five core nodes working in coordination:

### Core Nodes

1. **Eye Detector Node** (`eye_detector_node.py`)
   - **Primary Function**: Computer vision processing and drowsiness detection
   - **Capabilities**: 
     - Captures video feed from camera
     - Performs real-time face detection using dlib
     - Calculates Eye Aspect Ratio (EAR) for both eyes
     - Monitors consecutive frames for drowsiness patterns
     - Publishes detection results and annotated video feed
   - **Topics Published**: `/jupiter_juno/ear_data`, `/jupiter_juno/drowsiness_alert`, `/jupiter_juno/eye_detection_image`

2. **Alert System Node** (`alert_system_node.py`)
   - **Primary Function**: Central coordination and alert management
   - **Capabilities**:
     - Monitors drowsiness alerts from eye detector
     - Manages system state transitions (monitoring → alerting → conversation)
     - Triggers audio alerts and conversation initiation
     - Coordinates between detection and response systems
   - **Topics Subscribed**: `/jupiter_juno/drowsiness_alert`
   - **Topics Published**: `/jupiter_juno/system_state`, `/jupiter_juno/tts_request`

3. **Text-to-Speech Node** (`tts_node.py`)
   - **Primary Function**: Audio output and speech synthesis
   - **Capabilities**:
     - Converts text messages to natural speech using Google TTS
     - Handles audio playback through system speakers
     - Supports multiple languages and speech parameters
     - Manages audio queue and timing
   - **Topics Subscribed**: `/jupiter_juno/tts_request`

4. **Speech Recognition Node** (`speech_recognition_node.py`)
   - **Primary Function**: Voice input processing
   - **Capabilities**:
     - Captures audio from microphone
     - Converts speech to text using Google Speech Recognition
     - Handles background noise and audio processing
     - Publishes recognized speech for conversation handling
   - **Topics Published**: `/jupiter_juno/speech_result`

5. **Gemini Conversation Node** (`gemini_conversation_node.py`)
   - **Primary Function**: AI-powered conversation management
   - **Capabilities**:
     - Processes driver responses and generates contextual replies
     - Uses Gemini API for natural language processing
     - Provides driving-related jokes and safety tips
     - Falls back to pre-programmed responses if API unavailable
     - Manages conversation flow and turn limits
   - **Topics Subscribed**: `/jupiter_juno/speech_result`
   - **Topics Published**: `/jupiter_juno/tts_request`

### Integration Points

- **ROS Topics**: All communication via publish/subscribe messaging
- **Configuration**: Centralized YAML-based parameter management
- **Hardware**: Camera, microphone, and speaker integration
- **External APIs**: Gemini and OpenAI for conversation intelligence
- **GUI**: RQt-based monitoring interface for real-time visualization

### Key Package Components

- **Launch File**: Launch configurations for different deployment scenarios
- **Configuration**: YAML-based parameter system for easy customization
- **GUI Options**: Both RQt plugin and standalone PyQt5 interfaces
- **Model Files**: Pre-trained dlib model for facial landmark detection
- **Documentation**: Comprehensive setup and usage instructions 