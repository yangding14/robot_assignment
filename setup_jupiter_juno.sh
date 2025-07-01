#!/bin/bash

# Jupiter Juno Setup Script for ROS Noetic and Python 3.8.10
# This script sets up the complete Jupiter Juno drowsiness detection system

echo "================================================"
echo "Jupiter Juno Setup for ROS Noetic & Python 3.8.10"
echo "================================================"

# Check if running on Ubuntu 20.04 (recommended for ROS Noetic)
if ! lsb_release -d | grep -q "Ubuntu 20.04"; then
    echo "Warning: This system is optimized for Ubuntu 20.04 with ROS Noetic"
    echo "Current system: $(lsb_release -d)"
    read -p "Continue anyway? [y/N]: " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check if ROS Noetic is installed
if ! command -v roscore &> /dev/null; then
    echo "Error: ROS Noetic not found. Please install ROS Noetic first:"
    echo "http://wiki.ros.org/noetic/Installation/Ubuntu"
    exit 1
fi

echo "✓ ROS Noetic detected"

# Install system dependencies
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-pyqt5 \
    python3-pyqt5.qtwidgets \
    python3-yaml \
    python3-numpy \
    python3-scipy \
    libportaudio2 \
    portaudio19-dev \
    python3-pyaudio \
    cmake \
    libboost-all-dev

echo "✓ System dependencies installed"

# Make Python scripts executable
echo "Making Python scripts executable..."
chmod +x jupiter_juno/*.py
chmod +x *.py

echo "✓ Python scripts made executable"

# Install Python dependencies
echo "Installing Python dependencies..."
if [ -f "venv/bin/activate" ]; then
    echo "Activating virtual environment..."
    source venv/bin/activate
fi

pip3 install -r requirements.txt

echo "✓ Python dependencies installed"

# Download dlib face predictor if not present
PREDICTOR_FILE="jupiter_juno/shape_predictor_68_face_landmarks.dat"
if [ ! -f "$PREDICTOR_FILE" ]; then
    echo "Downloading dlib face landmark predictor..."
    wget -O shape_predictor_68_face_landmarks.dat.bz2 \
        http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2
    bunzip2 shape_predictor_68_face_landmarks.dat.bz2
    mv shape_predictor_68_face_landmarks.dat "$PREDICTOR_FILE"
    echo "✓ Face landmark predictor downloaded"
else
    echo "✓ Face landmark predictor already present"
fi

# Build the ROS package
echo "Building ROS package..."
cd ..
if [ -d "devel" ] || [ -d "build" ]; then
    echo "Catkin workspace detected, building..."
    catkin_make
else
    echo "Warning: Not in a catkin workspace. Please ensure this package is in a catkin workspace."
fi

echo "✓ ROS package built"

# Set up environment variables
echo "Setting up environment..."
if ! grep -q "GEMINI_API_KEY" ~/.bashrc; then
    echo "export GEMINI_API_KEY=\"your_gemini_api_key_here\"" >> ~/.bashrc
    echo "Please set your Gemini API key in ~/.bashrc or run:"
    echo "export GEMINI_API_KEY=\"your_actual_api_key\""
fi

echo "================================================"
echo "Setup Complete!"
echo "================================================"
echo ""
echo "To start the Jupiter Juno system:"
echo "1. Source your ROS environment: source devel/setup.bash"
echo "2. Launch the system: roslaunch jupiter_juno jupiter_juno_complete_system.launch"
echo ""
echo "Optional GUI-only launch:"
echo "roslaunch jupiter_juno jupiter_juno_complete_system.launch use_gui:=true"
echo ""
echo "For testing individual components:"
echo "rosrun jupiter_juno gui_monitor_node.py"
echo ""
echo "Make sure to set your Gemini API key before running!" 