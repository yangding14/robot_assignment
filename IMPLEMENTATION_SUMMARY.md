# Jupiter Juno Implementation Summary

## System Overview

The Jupiter Juno Driver Drowsiness Detection System is a comprehensive ROS 1-based solution that monitors driver alertness and provides intelligent interventions. The system combines computer vision, audio alerts, and AI-powered conversations to help keep drivers safe.

## Core Components

### 1. Eye Detector Node (`eye_detector_node.py`)
- **Purpose**: Real-time eye monitoring and drowsiness detection
- **Key Features**:
  - Uses dlib for face detection and facial landmarks
  - Calculates Eye Aspect Ratio (EAR)
  - Publishes EAR values and drowsiness alerts
  - Configurable detection thresholds
  - Runs at 24 FPS by default

### 2. Alert System Node (`alert_system_node.py`)
- **Purpose**: Manages alerts and system state coordination
- **Key Features**:
  - Generates audio alert sounds
  - Manages conversation triggering
  - Coordinates system state transitions
  - Handles startup and shutdown messages

### 3. TTS Node (`tts_node.py`)
- **Purpose**: Text-to-speech conversion using Google Text-to-Speech
- **Key Features**:
  - Uses gTTS for high-quality speech synthesis
  - Supports multiple languages (configurable)
  - Requires internet connection for speech generation
  - Uses pygame for audio playback
  - Thread-safe operation with queued processing

### 4. Speech Recognition Node (`speech_recognition_node.py`)
- **Purpose**: Voice input processing
- **Key Features**:
  - Google Speech Recognition integration
  - Automatic noise calibration
  - Timeout handling
  - Fallback to text input if unavailable

### 5. Gemini Conversation Node (`gemini_conversation_node.py`)
- **Purpose**: AI-powered conversations
- **Key Features**:
  - Gemini API integration (OpenAI as fallback)
  - Context-aware responses
  - Limited conversation rounds (default: 3)
  - Joke and safety tip generation

## System Flow

1. **Initialization**
   - System starts with greeting message
   - Camera and microphone calibration
   - All nodes initialize and connect via ROS topics

2. **Monitoring Phase**
   - Continuous eye tracking
   - EAR calculation for each frame
   - Drowsiness detection after 2 seconds of closed eyes

3. **Alert Phase**
   - Audio beeps for 3 seconds
   - Warning message via TTS
   - System state changes to "conversation_ready"

4. **Conversation Phase**
   - Prompt user for preference (joke or tip)
   - Listen for voice response
   - Generate and speak AI response
   - Limited to 3 exchanges

5. **Return to Monitoring**
   - Conversation ends
   - System returns to monitoring state
   - Process repeats as needed

## Configuration (`drowsiness_config.yaml`)

### Key Parameters:
- **EAR Threshold**: 0.20 (adjustable based on user)
- **Consecutive Frames**: 48 (2 seconds at 24 FPS)
- **Alert Duration**: 3 seconds
- **Max Conversation Rounds**: 3
- **TTS Engine**: gTTS (Google Text-to-Speech)
- **Language**: 'en' (configurable)

## ROS 1 Topics

- `/jupiter_juno/ear_data` - Eye Aspect Ratio values
- `/jupiter_juno/drowsiness_alert` - Drowsiness detection events
- `/jupiter_juno/tts_request` - Text-to-speech requests
- `/jupiter_juno/speech_result` - Speech recognition results
- `/jupiter_juno/system_state` - System state changes

## Installation Requirements

### System Dependencies:
- Ubuntu 20.04
- ROS 1 Noetic
- Python 3.8+
- Camera and microphone
- Internet connection (for gTTS)

### Python Packages:
- **Core**: opencv-python, dlib, scipy, numpy, pyyaml
- **Audio**: gTTS, pygame, pydub, simpleaudio, pyaudio
- **AI**: requests (for Gemini API calls), openai, SpeechRecognition

### Critical Files:
- `shape_predictor_68_face_landmarks.dat` - Required for face detection
- Must be placed in the package directory

## Testing

### Standalone Test (No ROS 1):
```bash
python3 test_system_standalone.py
```

### Full ROS 1 System:
```bash
roslaunch jupiter_juno jupiter_juno_launch.launch
```

### Quick Setup Check:
```bash
./quick_start.sh
```

## TTS Implementation Details

### Google Text-to-Speech (gTTS):
- **Advantages**:
  - High-quality, natural-sounding speech
  - Supports 100+ languages
  - Reliable Google infrastructure
  
- **Requirements**:
  - Internet connection for speech generation
  - First request may have slight delay
  
- **Audio Pipeline**:
  1. Text → gTTS API → MP3 file
  2. MP3 file → pygame mixer → Audio output
  3. Temporary file cleanup

### Alternative Fallback:
- System can be modified to use offline TTS like `espeak` if needed
- Configuration allows switching TTS engines

## Hardware Integration

### For Robot Integration:
1. **Camera**: Configure `camera_index` in YAML
2. **Audio**: Route through robot speakers
3. **Custom Topics**: Subscribe to relevant topics
4. **GPIO Integration**: Add nodes for LED indicators

## Key Design Decisions

1. **Modular Architecture**: Each component is a separate ROS node for flexibility
2. **Configuration-Driven**: YAML file for easy parameter adjustment
3. **gTTS for Quality**: Chose gTTS over offline options for better speech quality
4. **Thread Safety**: Separate threads for blocking operations
5. **State Management**: Centralized state coordination

## Performance Considerations

- Eye detection runs at 24 FPS (configurable)
- 2-second confirmation prevents false positives
- Conversation limited to prevent distraction
- Audio alerts prioritized over conversation
- gTTS caching could be added for repeated phrases

## Future Enhancements

1. Add local TTS caching to reduce internet dependency
2. Support for offline TTS engines as fallback
3. Multi-language support with language detection
4. Voice cloning for personalized responses
5. Integration with vehicle audio systems
6. Compression for audio files to reduce bandwidth

## Troubleshooting Tips

1. **No Speech Output**: Check internet connection and gTTS installation
2. **Poor Audio Quality**: Verify pygame installation and audio drivers
3. **TTS Delays**: Check network latency and consider caching
4. **API Rate Limits**: Monitor gTTS usage for rate limiting
5. **Language Issues**: Ensure correct language codes in configuration

## ROS 1 Specific Notes

- Uses `rospy` instead of `rclpy`
- XML launch files instead of Python
- `catkin_make` build system
- Traditional ROS 1 package structure
- Compatible with existing ROS 1 ecosystems

## Safety Notes

- System is an assistant, not a replacement for driver awareness
- Regular breaks still recommended
- Adjust thresholds based on individual characteristics
- Test thoroughly before deployment
- Internet dependency should be considered for safety-critical applications 