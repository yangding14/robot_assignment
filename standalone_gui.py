#!/usr/bin/env python3
"""
Standalone GUI for Jupiter Juno Drowsiness Detection System
This bypasses RQt and runs as a standalone PyQt5 application
"""

import sys
import os
import cv2
import numpy as np
from datetime import datetime

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(__file__))

import rospy
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                QHBoxLayout, QLabel, QPushButton, QTextEdit, 
                                QProgressBar, QFrame)
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal
    from PyQt5.QtGui import QPixmap, QImage, QFont, QPalette, QColor
    HAS_QT = True
except ImportError:
    print("Error: PyQt5 not available. Install with:")
    print("sudo apt install python3-pyqt5 python3-pyqt5-dev")
    sys.exit(1)


class JupiterJunoStandaloneGUI(QMainWindow):
    """
    Standalone GUI for monitoring Jupiter Juno drowsiness detection system
    This version doesn't use RQt and should avoid Qt binding issues
    """
    
    def __init__(self):
        super(JupiterJunoStandaloneGUI, self).__init__()
        
        # Initialize ROS
        rospy.init_node('jupiter_juno_standalone_gui', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Data storage
        self.current_ear = 0.0
        self.is_drowsy = False
        self.last_speech = ""
        self.last_tts = ""
        self.system_state = "initializing"
        self.current_image = None
        self.last_update_time = None
        
        # Setup GUI
        self.setWindowTitle("Jupiter Juno Drowsiness Detection Monitor")
        self.setGeometry(100, 100, 1200, 800)
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        self.setup_gui()
        self.setup_ros_subscribers()
        
        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # Update at 10Hz
        
        rospy.loginfo("Jupiter Juno Standalone GUI started")
    
    def setup_gui(self):
        """Setup the GUI layout and widgets"""
        # Main layout
        main_layout = QVBoxLayout()
        
        # Title
        title = QLabel("Jupiter Juno Drowsiness Detection Monitor (Standalone)")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #2c3e50; padding: 10px;")
        main_layout.addWidget(title)
        
        # Top row - Camera and Status
        top_layout = QHBoxLayout()
        
        # Camera section
        camera_frame = QFrame()
        camera_frame.setFrameStyle(QFrame.StyledPanel)
        camera_layout = QVBoxLayout()
        
        camera_title = QLabel("Camera Feed with Face Detection")
        camera_title.setAlignment(Qt.AlignCenter)
        camera_title.setStyleSheet("font-weight: bold; padding: 5px;")
        camera_layout.addWidget(camera_title)
        
        self.camera_label = QLabel()
        self.camera_label.setMinimumSize(480, 360)
        self.camera_label.setStyleSheet("border: 2px solid #34495e; background-color: #ecf0f1;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setText("Waiting for camera feed...")
        camera_layout.addWidget(self.camera_label)
        
        camera_frame.setLayout(camera_layout)
        top_layout.addWidget(camera_frame)
        
        # Status section
        status_frame = QFrame()
        status_frame.setFrameStyle(QFrame.StyledPanel)
        status_frame.setMinimumWidth(300)
        status_layout = QVBoxLayout()
        
        status_title = QLabel("System Status")
        status_title.setAlignment(Qt.AlignCenter)
        status_title.setStyleSheet("font-weight: bold; padding: 5px;")
        status_layout.addWidget(status_title)
        
        # EAR Display
        self.ear_label = QLabel("EAR: 0.00")
        self.ear_label.setStyleSheet("font-size: 16px; padding: 5px; border: 1px solid #bdc3c7;")
        status_layout.addWidget(self.ear_label)
        
        # EAR Progress Bar
        self.ear_progress = QProgressBar()
        self.ear_progress.setRange(0, 50)  # EAR typically 0.0 to 0.5
        self.ear_progress.setValue(25)
        self.ear_progress.setStyleSheet("""
            QProgressBar { border: 2px solid #bdc3c7; border-radius: 5px; text-align: center; }
            QProgressBar::chunk { background-color: #27ae60; }
        """)
        status_layout.addWidget(self.ear_progress)
        
        # Drowsiness Status
        self.drowsy_label = QLabel("Status: ALERT")
        self.drowsy_label.setAlignment(Qt.AlignCenter)
        self.drowsy_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; border-radius: 5px; background-color: #2ecc71; color: white;")
        status_layout.addWidget(self.drowsy_label)
        
        # System State
        self.state_label = QLabel("System: Initializing")
        self.state_label.setStyleSheet("font-size: 12px; padding: 5px; border: 1px solid #bdc3c7;")
        status_layout.addWidget(self.state_label)
        
        # Last EAR Update Time
        self.time_label = QLabel("Last Update: --:--:--")
        self.time_label.setStyleSheet("font-size: 10px; color: #7f8c8d; padding: 2px;")
        status_layout.addWidget(self.time_label)
        
        # Connection Status
        self.connection_label = QLabel("ROS: Connecting...")
        self.connection_label.setStyleSheet("font-size: 10px; color: #e74c3c; padding: 2px;")
        status_layout.addWidget(self.connection_label)
        
        status_frame.setLayout(status_layout)
        top_layout.addWidget(status_frame)
        
        main_layout.addLayout(top_layout)
        
        # Bottom row - Speech and TTS
        bottom_layout = QHBoxLayout()
        
        # Speech Recognition section
        speech_frame = QFrame()
        speech_frame.setFrameStyle(QFrame.StyledPanel)
        speech_layout = QVBoxLayout()
        
        speech_title = QLabel("Speech Recognition")
        speech_title.setAlignment(Qt.AlignCenter)
        speech_title.setStyleSheet("font-weight: bold; padding: 5px;")
        speech_layout.addWidget(speech_title)
        
        self.speech_display = QTextEdit()
        self.speech_display.setMaximumHeight(120)
        self.speech_display.setStyleSheet("border: 1px solid #bdc3c7; background-color: #f8f9fa;")
        self.speech_display.setPlainText("Waiting for speech input...")
        speech_layout.addWidget(self.speech_display)
        
        speech_frame.setLayout(speech_layout)
        bottom_layout.addWidget(speech_frame)
        
        # TTS section
        tts_frame = QFrame()
        tts_frame.setFrameStyle(QFrame.StyledPanel)
        tts_layout = QVBoxLayout()
        
        tts_title = QLabel("Text-to-Speech Output")
        tts_title.setAlignment(Qt.AlignCenter)
        tts_title.setStyleSheet("font-weight: bold; padding: 5px;")
        tts_layout.addWidget(tts_title)
        
        self.tts_display = QTextEdit()
        self.tts_display.setMaximumHeight(120)
        self.tts_display.setStyleSheet("border: 1px solid #bdc3c7; background-color: #f8f9fa;")
        self.tts_display.setPlainText("No TTS output yet...")
        tts_layout.addWidget(self.tts_display)
        
        tts_frame.setLayout(tts_layout)
        bottom_layout.addWidget(tts_frame)
        
        main_layout.addLayout(bottom_layout)
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        clear_speech_btn = QPushButton("Clear Speech Log")
        clear_speech_btn.clicked.connect(self.clear_speech_log)
        clear_speech_btn.setStyleSheet("padding: 5px;")
        button_layout.addWidget(clear_speech_btn)
        
        clear_tts_btn = QPushButton("Clear TTS Log")
        clear_tts_btn.clicked.connect(self.clear_tts_log)
        clear_tts_btn.setStyleSheet("padding: 5px;")
        button_layout.addWidget(clear_tts_btn)
        
        main_layout.addLayout(button_layout)
        
        # Set main layout
        self.centralWidget().setLayout(main_layout)
    
    def setup_ros_subscribers(self):
        """Setup ROS subscribers for system data"""
        try:
            # Subscribe to system topics
            rospy.Subscriber('/jupiter_juno/ear_data', Float32, self.ear_callback)
            rospy.Subscriber('/jupiter_juno/drowsiness_alert', Bool, self.drowsiness_callback)
            rospy.Subscriber('/jupiter_juno/eye_detection_image', Image, self.image_callback)
            rospy.Subscriber('/jupiter_juno/speech_result', String, self.speech_callback)
            rospy.Subscriber('/jupiter_juno/tts_output', String, self.tts_callback)
            rospy.Subscriber('/jupiter_juno/system_state', String, self.state_callback)
            
            self.connection_label.setText("ROS: Connected")
            self.connection_label.setStyleSheet("font-size: 10px; color: #27ae60; padding: 2px;")
            rospy.loginfo("ROS subscribers initialized")
            
        except Exception as e:
            rospy.logerr(f"Failed to setup ROS subscribers: {e}")
            self.connection_label.setText("ROS: Error")
            self.connection_label.setStyleSheet("font-size: 10px; color: #e74c3c; padding: 2px;")
    
    def ear_callback(self, msg):
        """Handle EAR data updates"""
        self.current_ear = msg.data
        self.last_update_time = datetime.now()
    
    def drowsiness_callback(self, msg):
        """Handle drowsiness alert updates"""
        self.is_drowsy = msg.data
    
    def image_callback(self, msg):
        """Handle camera image updates"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
        except Exception as e:
            rospy.logwarn(f"Failed to convert image: {e}")
    
    def speech_callback(self, msg):
        """Handle speech recognition updates"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.last_speech = f"[{timestamp}] {msg.data}"
    
    def tts_callback(self, msg):
        """Handle TTS output updates"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.last_tts = f"[{timestamp}] {msg.data}"
    
    def state_callback(self, msg):
        """Handle system state updates"""
        self.system_state = msg.data
    
    def update_display(self):
        """Update all GUI elements with current data"""
        # Update EAR display
        self.ear_label.setText(f"EAR: {self.current_ear:.3f}")
        ear_percentage = min(int(self.current_ear * 100), 50)
        self.ear_progress.setValue(ear_percentage)
        
        # Update drowsiness status
        if self.is_drowsy:
            self.drowsy_label.setText("Status: DROWSY")
            self.drowsy_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; border-radius: 5px; background-color: #e74c3c; color: white;")
            self.ear_progress.setStyleSheet("""
                QProgressBar { border: 2px solid #bdc3c7; border-radius: 5px; text-align: center; }
                QProgressBar::chunk { background-color: #e74c3c; }
            """)
        else:
            self.drowsy_label.setText("Status: ALERT")
            self.drowsy_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; border-radius: 5px; background-color: #2ecc71; color: white;")
            self.ear_progress.setStyleSheet("""
                QProgressBar { border: 2px solid #bdc3c7; border-radius: 5px; text-align: center; }
                QProgressBar::chunk { background-color: #27ae60; }
            """)
        
        # Update system state
        self.state_label.setText(f"System: {self.system_state}")
        
        # Update timestamp
        if self.last_update_time:
            time_str = self.last_update_time.strftime("%H:%M:%S")
            self.time_label.setText(f"Last Update: {time_str}")
        
        # Update camera feed
        if self.current_image is not None:
            self.display_image(self.current_image)
        
        # Update speech and TTS displays
        if self.last_speech:
            current_speech = self.speech_display.toPlainText()
            if self.last_speech not in current_speech:
                self.speech_display.append(self.last_speech)
                # Keep only last 10 lines
                lines = self.speech_display.toPlainText().split('\n')
                if len(lines) > 10:
                    self.speech_display.setPlainText('\n'.join(lines[-10:]))
        
        if self.last_tts:
            current_tts = self.tts_display.toPlainText()
            if self.last_tts not in current_tts:
                self.tts_display.append(self.last_tts)
                # Keep only last 10 lines
                lines = self.tts_display.toPlainText().split('\n')
                if len(lines) > 10:
                    self.tts_display.setPlainText('\n'.join(lines[-10:]))
    
    def display_image(self, cv_image):
        """Convert OpenCV image to Qt and display it"""
        try:
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            # Create QImage
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # Scale to fit label while maintaining aspect ratio
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            
            self.camera_label.setPixmap(scaled_pixmap)
            
        except Exception as e:
            rospy.logwarn(f"Failed to display image: {e}")
    
    def clear_speech_log(self):
        """Clear speech recognition log"""
        self.speech_display.setPlainText("Speech log cleared...")
    
    def clear_tts_log(self):
        """Clear TTS output log"""
        self.tts_display.setPlainText("TTS log cleared...")
    
    def closeEvent(self, event):
        """Handle window close event"""
        rospy.loginfo("Shutting down Jupiter Juno GUI")
        rospy.signal_shutdown("GUI closed")
        event.accept()


def main():
    """Main function to start the standalone GUI"""
    # Create QApplication
    app = QApplication(sys.argv)
    
    # Create and show main window
    window = JupiterJunoStandaloneGUI()
    window.show()
    
    # Start Qt event loop
    try:
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main() 