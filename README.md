# Jupiter Juno Driver Drowsiness Detection System

A ROS 1-based intelligent co-driver system that monitors driver alertness and provides timely interventions to enhance road safety.

## Features

- **Real-time Eye Detection**: Monitors driver's eye aspect ratio (EAR) to detect drowsiness
- **Multi-stage Alert System**: Audio alerts followed by engaging AI conversations
- **Voice Interaction**: Speech recognition and text-to-speech capabilities using gTTS
- **AI-Powered Conversations**: Uses Gemini API (via direct HTTP calls) or OpenAI as fallback for natural conversations
- **Modular Architecture**: Built with ROS 1 for easy integration with robotic systems
- **Configurable Parameters**: Easily adjustable thresholds and settings via YAML configuration

## System Architecture

The system consists of five main ROS 1 nodes:

1. **Eye Detector Node**: Captures video, detects faces, and calculates Eye Aspect Ratio
2. **Alert System Node**: Manages drowsiness alerts and system state
3. **TTS Node**: Converts text to speech using Google Text-to-Speech (gTTS)
4. **Speech Recognition Node**: Processes driver voice input
5. **Gemini Conversation Node**: Handles AI-powered conversations via direct API calls

## Project Structure

```
robot_assignment/                    # This directory IS the ROS package
├── CMakeLists.txt                  # ROS build configuration
├── package.xml                     # ROS package manifest (package name: jupiter_juno)
├── requirements.txt                # Python dependencies
├── jupiter_juno/                   # Python node modules
│   ├── __init__.py
│   ├── eye_detector_node.py        # Main eye tracking node
│   ├── alert_system_node.py        # Alert management
│   ├── tts_node.py                 # Text-to-speech
│   ├── speech_recognition_node.py  # Voice input processing
│   ├── gemini_conversation_node.py # AI conversation handler
│   └── shape_predictor_68_face_landmarks.dat  # Face detection model
├── launch/
│   └── jupiter_juno_launch.launch  # Main launch file
├── config/
│   └── drowsiness_config.yaml      # System configuration
├── test_system_standalone.py       # Standalone testing script
├── quick_start.sh                  # Setup helper script
└── README.md                       # This file
```

**Note**: 
- The package name in `package.xml` is `jupiter_juno`, but the directory can be named anything when copied to your catkin workspace
- This entire `robot_assignment` directory IS the ROS package - there's no nested `src/` directory within it
- When you copy it to `~/catkin_ws/src/`, it becomes a complete ROS package

## Prerequisites

- Ubuntu 20.04 (or 18.04 for ROS Melodic)
- ROS 1 Noetic (or Melodic)
- Python 3.8+
- Webcam/Camera
- Microphone and speakers
- Internet connection (required for gTTS and Gemini API)

## Installation

### 1. Install ROS 1 Noetic

Follow the official ROS 1 Noetic installation guide:
- [ROS 1 Noetic (Ubuntu 20.04)](http://wiki.ros.org/noetic/Installation/Ubuntu)

### 2. Install System Dependencies

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
    ros-noetic-cv-bridge \
    ros-noetic-rospy \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs
```

### 3. Setup ROS Workspace and Package

```bash
# Create catkin workspace if it doesn't exist
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Copy the entire robot_assignment directory as the ROS package
# Option 1: If you have git repository
git clone <repository-url> jupiter_juno

# Option 2: If you have the package folder locally
cp -r /path/to/robot_assignment jupiter_juno

# Install Python dependencies
cd jupiter_juno
pip3 install -r requirements.txt
```

### 4. Build the Package

```bash
# Go to workspace root
cd ~/catkin_ws

# Source ROS 1
source /opt/ros/noetic/setup.bash

# Build the package
catkin_make

# Source the workspace
source devel/setup.bash

# Make Python scripts executable (from catkin_ws directory)
chmod +x src/jupiter_juno/jupiter_juno/*.py
```

## Configuration

### Environment Variables

Set up your API keys in your `~/.bashrc`:

```bash
# For Gemini API (recommended)
export GEMINI_API_KEY="your-gemini-api-key"

# For OpenAI API (fallback)
export OPENAI_API_KEY="your-openai-api-key"

# Apply changes
source ~/.bashrc
```

### Configuration File

Edit `config/drowsiness_config.yaml` to adjust:

- **Eye Detection Parameters**:
  - `ear_threshold`: EAR threshold for detecting closed eyes (default: 0.20)
  - `consecutive_frames`: Frames required to confirm drowsiness (default: 48)
  
- **Alert Settings**:
  - `alert_duration`: Duration of alert sound in seconds
  - `alert_frequency`: Frequency of alert beep in Hz
  
- **TTS Settings**:
  - `language`: Language for gTTS (default: 'en')
  - `slow_speech`: Enable slower speech for clarity
  
- **Conversation Settings**:
  - `max_rounds`: Maximum conversation rounds (default: 3)
  - `system_prompt`: AI assistant personality and behavior

## Usage

### Quick Test (Without ROS)

For a quick test without ROS setup:
```bash
cd robot_assignment
python3 test_system_standalone.py
```

### Running with ROS

1. **Start all nodes with the launch file:**

```bash
# Make sure you're in the workspace and sourced
cd ~/catkin_ws
source devel/setup.bash

# Launch the complete system
roslaunch jupiter_juno jupiter_juno_launch.launch
```

2. **Or run individual nodes for testing:**

```bash
# Terminal 1: ROS Master
roscore

# Terminal 2: Eye Detector (in workspace directory)
cd ~/catkin_ws && source devel/setup.bash
rosrun jupiter_juno eye_detector_node.py

# Terminal 3: Alert System
rosrun jupiter_juno alert_system_node.py

# Terminal 4: TTS
rosrun jupiter_juno tts_node.py

# Terminal 5: Speech Recognition
rosrun jupiter_juno speech_recognition_node.py

# Terminal 6: Gemini Conversation
rosrun jupiter_juno gemini_conversation_node.py
```

### Monitoring the System

View active topics:
```bash
rostopic list
```

Monitor drowsiness detection:
```bash
rostopic echo /jupiter_juno/drowsiness_alert
```

View EAR values in real-time:
```bash
rostopic echo /jupiter_juno/ear_data
```

Monitor system state:
```bash
rostopic echo /jupiter_juno/system_state
```

## How It Works

1. **Startup**: System greets the driver and begins monitoring
2. **Monitoring**: Continuously tracks eye aspect ratio (EAR) using computer vision
3. **Detection**: When EAR falls below threshold for 2+ seconds, drowsiness is detected
4. **Alert**: Audio alert sounds for 3 seconds to wake the driver
5. **Engagement**: System asks if driver wants a joke or safety tip to maintain alertness
6. **Conversation**: Driver can respond verbally, limited to 3 exchanges to avoid distraction
7. **Return**: System returns to monitoring mode automatically

## Troubleshooting

### Common Issues

1. **"Shape predictor file not found"**
   - The model file should already be included in `jupiter_juno/shape_predictor_68_face_landmarks.dat`
   - If missing, download from [dlib model repository](http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2)
   - Extract and place in the `jupiter_juno/` directory

2. **"No module named 'jupiter_juno'"**
   - Ensure you've built the catkin workspace: `catkin_make`
   - Source the workspace: `source ~/catkin_ws/devel/setup.bash`
   - Check package is in `~/catkin_ws/src/jupiter_juno`

3. **"Package 'jupiter_juno' not found"**
   - Verify package.xml exists in the package root
   - Make sure you ran `catkin_make` without errors
   - Check ROS can find the package: `rospack find jupiter_juno`

4. **No audio output**
   - Check system volume and audio devices
   - Test with: `espeak "test"`
   - Install missing audio packages: `sudo apt install pulseaudio pavucontrol`
   - Ensure internet connection for gTTS

5. **gTTS not working**
   - Check internet connection (gTTS requires online access)
   - Verify gTTS installation: `pip3 show gTTS`
   - Check for rate limiting if making many requests

6. **Speech recognition not working**
   - Check microphone permissions
   - Test microphone: `arecord -d 3 test.wav && aplay test.wav`
   - Ensure internet connection for Google Speech Recognition
   - Install audio dependencies: `sudo apt install python3-pyaudio`

7. **Camera not detected**
   - Check camera permissions and connection
   - Try different camera index in config (0, 1, 2...)
   - Test with: `cheese` or `ls /dev/video*`
   - Ensure camera is not being used by another application

8. **API errors (Gemini/OpenAI)**
   - Verify API keys are set correctly: `echo $GEMINI_API_KEY`
   - Check API key permissions and quotas
   - Ensure internet connection for API calls

### Debug Mode

Enable verbose logging:
```bash
# Set ROS logging level
export ROSCONSOLE_CONFIG_FILE=/path/to/custom_rosconsole.conf

# Or run individual nodes with debug output
rosrun jupiter_juno eye_detector_node.py
```

View all ROS logs:
```bash
rqt_console
```

## Integration with Hardware

To integrate with your Jupiter Juno robot hardware:

1. **Camera Integration**: Modify `camera_index` in config to match your robot's camera
2. **Audio Output**: Configure audio to route through robot's speakers  
3. **Custom Topics**: Subscribe to additional sensor data as needed
4. **GPIO Integration**: Add nodes for LED indicators or other hardware alerts

## Technical Notes

### AI Implementation
- **Gemini API**: Uses direct HTTP calls instead of google-generativeai library for better compatibility
- **Fallback**: OpenAI API available if Gemini fails
- **Error Handling**: Robust error handling with graceful fallbacks

### TTS Implementation
- Uses Google Text-to-Speech (gTTS) for high-quality speech synthesis
- Requires internet connection for speech generation
- Supports multiple languages (configure in YAML)
- Audio played back using pygame

### Dependencies
- **Core**: opencv-python, dlib, scipy, numpy, pyyaml
- **Audio**: gTTS, pygame, pydub, simpleaudio, pyaudio
- **AI**: requests (for Gemini API calls), openai, SpeechRecognition

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the Apache License 2.0.

## Acknowledgments

- dlib for facial landmark detection
- Google Text-to-Speech for high-quality speech synthesis
- Google Generative AI for conversation capabilities
- ROS 1 community for the robotics framework 