#!/bin/bash

# Jupiter Juno Quick Start Script
# This script helps set up and test the system

echo "====================================="
echo "Jupiter Juno Drowsiness Detection"
echo "Quick Start Setup Script"
echo "====================================="
echo ""

# Check Python version
echo "Checking Python version..."
python3 --version

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Install basic requirements
echo "Installing basic requirements..."
pip install --upgrade pip
pip install opencv-python numpy scipy pyyaml

# Check for optional dependencies
echo ""
echo "Checking optional dependencies..."

# TTS (gTTS)
if pip show gTTS &> /dev/null; then
    echo "✓ Text-to-Speech (gTTS) installed"
else
    echo "✗ Text-to-Speech (gTTS) not installed"
    echo "  Install with: pip install gTTS pygame"
fi

# Audio playback
if pip show pygame &> /dev/null; then
    echo "✓ Audio playback (pygame) installed"
else
    echo "✗ Audio playback (pygame) not installed"
    echo "  Install with: pip install pygame"
fi

# Speech Recognition
if pip show SpeechRecognition &> /dev/null; then
    echo "✓ Speech Recognition installed"
else
    echo "✗ Speech Recognition not installed"
    echo "  Install with: pip install SpeechRecognition pyaudio"
fi

# Requests library for API calls
if pip show requests &> /dev/null; then
    echo "✓ Requests library installed (needed for Gemini API calls)"
else
    echo "✗ Requests library not installed"
    echo "  Install with: pip install requests"
fi

# Check for dlib
if pip show dlib &> /dev/null; then
    echo "✓ dlib installed"
else
    echo "✗ dlib not installed"
    echo "  Note: dlib installation can take time"
    echo "  Install with: pip install dlib"
fi

# Check for shape predictor file
echo ""
echo "Checking for shape predictor file..."
PREDICTOR_PATHS=(
    "src/jupiter_juno/jupiter_juno/shape_predictor_68_face_landmarks.dat"
    "../Eye_Detector_Script/shape_predictor_68_face_landmarks.dat"
)

PREDICTOR_FOUND=false
for path in "${PREDICTOR_PATHS[@]}"; do
    if [ -f "$path" ]; then
        echo "✓ Shape predictor found at: $path"
        PREDICTOR_FOUND=true
        break
    fi
done

if [ "$PREDICTOR_FOUND" = false ]; then
    echo "✗ Shape predictor file not found!"
    echo "  Please download from: http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2"
    echo "  Extract and place in: src/jupiter_juno/jupiter_juno/"
fi

# Check camera
echo ""
echo "Checking camera availability..."
if [ -e /dev/video0 ]; then
    echo "✓ Camera detected at /dev/video0"
else
    echo "✗ No camera detected at /dev/video0"
    echo "  Check camera connection or try different index in config"
fi

# Check internet connection
echo ""
echo "Checking internet connection (required for gTTS)..."
if ping -c 1 google.com &> /dev/null; then
    echo "✓ Internet connection available"
else
    echo "✗ No internet connection detected"
    echo "  Internet connection required for gTTS (Google Text-to-Speech)"
fi

# Check ROS 1 installation
echo ""
echo "Checking ROS 1 installation..."
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    echo "✓ ROS 1 Noetic detected"
    source /opt/ros/noetic/setup.bash
    if command -v roscore &> /dev/null; then
        echo "✓ ROS 1 commands available"
    else
        echo "✗ ROS 1 commands not found in PATH"
    fi
else
    echo "✗ ROS 1 Noetic not found!"
    echo "  Please install ROS 1 Noetic from: http://wiki.ros.org/noetic/Installation/Ubuntu"
fi

# Environment variables info
echo ""
echo "Environment Setup:"
echo "=================="
echo "For Gemini API, set: export GEMINI_API_KEY='your-key'"
echo "For OpenAI API, set: export OPENAI_API_KEY='your-key'"

# Test options
echo ""
echo "Test Options:"
echo "============="
echo "1. Run standalone test (no ROS 1 required):"
echo "   python3 test_system_standalone.py"
echo ""
echo "2. Run with ROS 1 (requires ROS 1 installation):"
echo "   # First, set up workspace:"
echo "   mkdir -p ~/catkin_ws/src"
echo "   cp -r src/jupiter_juno ~/catkin_ws/src/"
echo "   cd ~/catkin_ws && catkin_make && source devel/setup.bash"
echo "   # Then launch:"
echo "   roslaunch jupiter_juno jupiter_juno_launch.launch"
echo ""
echo "3. Test eye detection only:"
echo "   cd ../Eye_Detector_Script && python3 main.py"

echo ""
echo "Notes:"
echo "======"
echo "- gTTS (Google Text-to-Speech) requires internet connection"
echo "- First run may be slower as gTTS downloads audio"
echo "- For offline TTS, consider alternatives like espeak"

echo ""
echo "Setup check complete!"
echo "=====================================" 