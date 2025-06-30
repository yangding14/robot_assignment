# Jupiter Juno Driver Drowsiness Detection System

A ROS 1-based intelligent co-driver system that monitors driver alertness and provides timely interventions to enhance road safety.

## Features

- **Real-time Eye Detection**: Monitors driver's eye aspect ratio (EAR) to detect drowsiness
- **Multi-stage Alert System**: Audio alerts followed by engaging AI conversations
- **Voice Interaction**: Speech recognition and text-to-speech capabilities using gTTS
- **AI-Powered Conversations**: Uses Gemini API (or OpenAI as fallback) for natural conversations
- **Modular Architecture**: Built with ROS 1 for easy integration with robotic systems
- **Configurable Parameters**: Easily adjustable thresholds and settings via YAML configuration

## System Architecture

The system consists of five main ROS 1 nodes:

1. **Eye Detector Node**: Captures video, detects faces, and calculates Eye Aspect Ratio
2. **Alert System Node**: Manages drowsiness alerts and system state
3. **TTS Node**: Converts text to speech using Google Text-to-Speech (gTTS)
4. **Speech Recognition Node**: Processes driver voice input
5. **Gemini Conversation Node**: Handles AI-powered conversations

## Prerequisites

- Ubuntu 20.04
- ROS 1 Noetic
- Python 3.8+
- Webcam/Camera
- Microphone and speakers
- Internet connection (required for gTTS)

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

### 3. Clone and Setup the Package

```bash
# Create workspace if not exists
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Copy the jupiter_juno_robot directory here
cp -r /path/to/jupiter_juno_robot/src/jupiter_juno .

# Install Python dependencies
cd jupiter_juno
pip3 install -r ../../requirements.txt

# Copy the face landmark model
cp ../../Eye_Detector_Script/shape_predictor_68_face_landmarks.dat \
   jupiter_juno/
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
```

## Configuration

### Environment Variables

Set up your API keys:

```bash
# For Gemini API (recommended)
export GEMINI_API_KEY="your-gemini-api-key"

# For OpenAI API (fallback)
export OPENAI_API_KEY="your-openai-api-key"
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

### Running the System

1. **Start all nodes with the launch file:**

```bash
roslaunch jupiter_juno jupiter_juno_launch.launch
```

2. **Or run individual nodes for testing:**

```bash
# Terminal 1: ROS Master
roscore

# Terminal 2: Eye Detector
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

View topics:
```bash
rostopic list
```

Monitor drowsiness detection:
```bash
rostopic echo /jupiter_juno/drowsiness_alert
```

View EAR values:
```bash
rostopic echo /jupiter_juno/ear_data
```

## How It Works

1. **Startup**: System greets the driver and begins monitoring
2. **Monitoring**: Continuously tracks eye aspect ratio (EAR)
3. **Detection**: When EAR falls below threshold for 2+ seconds, drowsiness is detected
4. **Alert**: Audio alert sounds for 3 seconds
5. **Engagement**: Robot asks if driver wants a joke or safety tip
6. **Conversation**: Driver can respond verbally, limited to 3 exchanges
7. **Return**: System returns to monitoring mode

## Troubleshooting

### Common Issues

1. **"Shape predictor file not found"**
   - Ensure the `.dat` file is in the correct location
   - Download from [dlib model repository](http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2)

2. **No audio output**
   - Check system volume
   - Test with: `espeak "test"`
   - Install missing audio packages
   - Ensure internet connection for gTTS

3. **gTTS not working**
   - Check internet connection (gTTS requires online access)
   - Verify gTTS installation: `pip3 show gTTS`
   - Check for rate limiting if making many requests

4. **Speech recognition not working**
   - Check microphone permissions
   - Test microphone with system settings
   - Ensure internet connection for Google Speech Recognition

5. **Camera not detected**
   - Check camera permissions
   - Try different camera index in config (0, 1, 2...)
   - Test with: `cheese` or `v4l2-ctl --list-devices`

### Debug Mode

Enable verbose logging:
```bash
rosrun jupiter_juno eye_detector_node.py --log-level debug
```

## Integration with Hardware

To integrate with your Jupiter Juno robot hardware:

1. **Camera Integration**: Modify `camera_index` in config to match your robot's camera
2. **Audio Output**: Configure audio to route through robot's speakers
3. **Custom Topics**: Subscribe to additional sensor data as needed
4. **GPIO Integration**: Add nodes for LED indicators or other hardware alerts

## Technical Notes

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