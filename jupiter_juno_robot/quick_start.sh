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

# TTS
if pip show pyttsx3 &> /dev/null; then
    echo "✓ Text-to-Speech (pyttsx3) installed"
else
    echo "✗ Text-to-Speech (pyttsx3) not installed"
    echo "  Install with: pip install pyttsx3"
fi

# Speech Recognition
if pip show SpeechRecognition &> /dev/null; then
    echo "✓ Speech Recognition installed"
else
    echo "✗ Speech Recognition not installed"
    echo "  Install with: pip install SpeechRecognition pyaudio"
fi

# Gemini API
if pip show google-generativeai &> /dev/null; then
    echo "✓ Gemini API installed"
else
    echo "✗ Gemini API not installed"
    echo "  Install with: pip install google-generativeai"
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
echo "1. Run standalone test (no ROS 2 required):"
echo "   python3 test_system_standalone.py"
echo ""
echo "2. Run with ROS 2 (requires ROS 2 installation):"
echo "   ros2 launch jupiter_juno jupiter_juno_launch.py"
echo ""
echo "3. Test eye detection only:"
echo "   cd ../Eye_Detector_Script && python3 main.py"

echo ""
echo "Setup check complete!"
echo "=====================================" 