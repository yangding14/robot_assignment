# Jupiter Juno Driver Drowsiness Detection System

A ROS 2-based intelligent co-driver system that monitors driver alertness and provides timely interventions to enhance road safety.

## Features

- **Real-time Eye Detection**: Monitors driver's eye aspect ratio (EAR) to detect drowsiness
- **Multi-stage Alert System**: Audio alerts followed by engaging AI conversations
- **Voice Interaction**: Speech recognition and text-to-speech capabilities
- **AI-Powered Conversations**: Uses Gemini API (or OpenAI as fallback) for natural conversations
- **Modular Architecture**: Built with ROS 2 for easy integration with robotic systems
- **Configurable Parameters**: Easily adjustable thresholds and settings via YAML configuration

## System Architecture

The system consists of five main ROS 2 nodes:

1. **Eye Detector Node**: Captures video, detects faces, and calculates Eye Aspect Ratio
2. **Alert System Node**: Manages drowsiness alerts and system state
3. **TTS Node**: Converts text to speech for robot communication
4. **Speech Recognition Node**: Processes driver voice input
5. **Gemini Conversation Node**: Handles AI-powered conversations

## Prerequisites

- Ubuntu 20.04 or 22.04
- ROS 2 (Foxy, Galactic, or Humble)
- Python 3.8+
- Webcam/Camera
- Microphone and speakers

## Installation

### 1. Install ROS 2

Follow the official ROS 2 installation guide for your distribution:
- [ROS 2 Humble (Ubuntu 22.04)](https://docs.ros.org/en/humble/Installation.html)
- [ROS 2 Foxy (Ubuntu 20.04)](https://docs.ros.org/en/foxy/Installation.html)

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
    ffmpeg
```

### 3. Clone and Setup the Package

```bash
# Navigate to your workspace
cd ~/ros2_ws/src

# Copy the jupiter_juno_robot directory here
cp -r /path/to/jupiter_juno_robot .

# Install Python dependencies
cd jupiter_juno_robot
pip3 install -r requirements.txt

# Copy the face landmark model
cp ../../../Eye_Detector_Script/shape_predictor_68_face_landmarks.dat \
   src/jupiter_juno/jupiter_juno/
```

### 4. Build the Package

```bash
# Go to workspace root
cd ~/ros2_ws

# Source ROS 2
source /opt/ros/humble/setup.bash  # or your ROS 2 version

# Build the package
colcon build --packages-select jupiter_juno

# Source the workspace
source install/setup.bash
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

Edit `src/jupiter_juno/config/drowsiness_config.yaml` to adjust:

- **Eye Detection Parameters**:
  - `ear_threshold`: EAR threshold for detecting closed eyes (default: 0.20)
  - `consecutive_frames`: Frames required to confirm drowsiness (default: 48)
  
- **Alert Settings**:
  - `alert_duration`: Duration of alert sound in seconds
  - `alert_frequency`: Frequency of alert beep in Hz
  
- **Conversation Settings**:
  - `max_rounds`: Maximum conversation rounds (default: 3)
  - `system_prompt`: AI assistant personality and behavior

## Usage

### Running the System

1. **Start all nodes with the launch file:**

```bash
ros2 launch jupiter_juno jupiter_juno_launch.py
```

2. **Or run individual nodes for testing:**

```bash
# Terminal 1: Eye Detector
ros2 run jupiter_juno eye_detector_node

# Terminal 2: Alert System
ros2 run jupiter_juno alert_system_node

# Terminal 3: TTS
ros2 run jupiter_juno tts_node

# Terminal 4: Speech Recognition
ros2 run jupiter_juno speech_recognition_node

# Terminal 5: Gemini Conversation
ros2 run jupiter_juno gemini_conversation_node
```

### Monitoring the System

View topics:
```bash
ros2 topic list
```

Monitor drowsiness detection:
```bash
ros2 topic echo /jupiter_juno/drowsiness_alert
```

View EAR values:
```bash
ros2 topic echo /jupiter_juno/ear_data
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

3. **Speech recognition not working**
   - Check microphone permissions
   - Test microphone with system settings
   - Ensure internet connection for Google Speech Recognition

4. **Camera not detected**
   - Check camera permissions
   - Try different camera index in config (0, 1, 2...)
   - Test with: `cheese` or `v4l2-ctl --list-devices`

### Debug Mode

Enable verbose logging:
```bash
ros2 run jupiter_juno eye_detector_node --ros-args --log-level debug
```

## Integration with Hardware

To integrate with your Jupiter Juno robot hardware:

1. **Camera Integration**: Modify `camera_index` in config to match your robot's camera
2. **Audio Output**: Configure audio to route through robot's speakers
3. **Custom Topics**: Subscribe to additional sensor data as needed
4. **GPIO Integration**: Add nodes for LED indicators or other hardware alerts

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
- Google Generative AI for conversation capabilities
- ROS 2 community for the robotics framework 