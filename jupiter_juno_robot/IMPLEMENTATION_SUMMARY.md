# Jupiter Juno Implementation Summary

## System Overview

The Jupiter Juno Driver Drowsiness Detection System is a comprehensive ROS 2-based solution that monitors driver alertness and provides intelligent interventions. The system combines computer vision, audio alerts, and AI-powered conversations to help keep drivers safe.

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
- **Purpose**: Text-to-speech conversion
- **Key Features**:
  - Supports multiple TTS engines (pyttsx3, gTTS)
  - Queued speech processing
  - Configurable voice and speech rate
  - Thread-safe operation

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
- **TTS Engine**: pyttsx3 (with gTTS fallback)

## ROS 2 Topics

- `/jupiter_juno/ear_data` - Eye Aspect Ratio values
- `/jupiter_juno/drowsiness_alert` - Drowsiness detection events
- `/jupiter_juno/tts_request` - Text-to-speech requests
- `/jupiter_juno/speech_result` - Speech recognition results
- `/jupiter_juno/system_state` - System state changes

## Installation Requirements

### System Dependencies:
- Ubuntu 20.04/22.04
- ROS 2 (Foxy/Galactic/Humble)
- Python 3.8+
- Camera and microphone

### Python Packages:
- **Core**: opencv-python, dlib, scipy, numpy, pyyaml
- **Audio**: pyttsx3, pygame, simpleaudio, pyaudio
- **AI**: google-generativeai, openai, SpeechRecognition

### Critical Files:
- `shape_predictor_68_face_landmarks.dat` - Required for face detection
- Must be placed in the package directory

## Testing

### Standalone Test (No ROS 2):
```bash
python3 test_system_standalone.py
```

### Full ROS 2 System:
```bash
ros2 launch jupiter_juno jupiter_juno_launch.py
```

### Quick Setup Check:
```bash
./quick_start.sh
```

## Hardware Integration

### For Robot Integration:
1. **Camera**: Configure `camera_index` in YAML
2. **Audio**: Route through robot speakers
3. **GPIO**: Add nodes for LED indicators
4. **Additional Sensors**: Subscribe to relevant topics

## Key Design Decisions

1. **Modular Architecture**: Each component is a separate ROS node for flexibility
2. **Configuration-Driven**: YAML file for easy parameter adjustment
3. **Multiple Fallbacks**: Alternative engines for TTS and AI
4. **Thread Safety**: Separate threads for blocking operations
5. **State Management**: Centralized state coordination

## Performance Considerations

- Eye detection runs at 24 FPS (configurable)
- 2-second confirmation prevents false positives
- Conversation limited to prevent distraction
- Audio alerts prioritized over conversation

## Future Enhancements

1. Add visual alerts (LED indicators)
2. Integration with vehicle CAN bus
3. Cloud-based analytics
4. Multi-language support
5. Personalized EAR thresholds
6. Integration with emergency services

## Troubleshooting Tips

1. **No Face Detection**: Check lighting and camera angle
2. **False Positives**: Adjust EAR threshold in config
3. **No Audio**: Check system volume and audio permissions
4. **API Errors**: Verify API keys are set correctly
5. **Performance Issues**: Reduce processing FPS

## Safety Notes

- System is an assistant, not a replacement for driver awareness
- Regular breaks still recommended
- Adjust thresholds based on individual characteristics
- Test thoroughly before deployment 