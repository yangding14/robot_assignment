#!/usr/bin/env python3
"""
Jupiter Juno GUI Monitor Node
A ROS Noetic compatible GUI for monitoring the Jupiter Juno drowsiness detection system
Compatible with Python 3.8.10
"""

import sys
import os
import cv2
import numpy as np
from datetime import datetime

import rospy
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                QHBoxLayout, QLabel, QPushButton, QTextEdit, 
                                QProgressBar, QFrame, QSizePolicy)
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
    from PyQt5.QtGui import QPixmap, QImage, QFont, QPalette, QColor
    HAS_QT = True
except ImportError:
    try:
        # Fallback to python_qt_binding for better ROS integration
        from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                                                QLabel, QPushButton, QTextEdit, 
                                                QProgressBar, QFrame, QSizePolicy)
        from python_qt_binding.QtCore import Qt, QTimer, pyqtSignal
        from python_qt_binding.QtGui import QPixmap, QImage, QFont, QPalette, QColor
        HAS_QT = True
    except ImportError:
        HAS_QT = False
        rospy.logerr("Neither PyQt5 nor python_qt_binding available. Install PyQt5:")
        rospy.logerr("sudo apt install python3-pyqt5 python3-pyqt5-dev")
        sys.exit(1)


class JupiterJunoGUINode(QWidget):
    """
    ROS Node with PyQt5 GUI for monitoring Jupiter Juno drowsiness detection system
    Compatible with ROS Noetic and Python 3.8.10
    """
    
    # Qt Signals for thread-safe GUI updates
    ear_signal = pyqtSignal(float)
    drowsiness_signal = pyqtSignal(bool)
    image_signal = pyqtSignal(np.ndarray)
    speech_signal = pyqtSignal(str)
    tts_signal = pyqtSignal(str)
    state_signal = pyqtSignal(str)
    
    def __init__(self):
        super(JupiterJunoGUINode, self).__init__()
        
        # Initialize ROS node
        rospy.init_node('jupiter_juno_gui_monitor', anonymous=True)
        rospy.loginfo("Starting Jupiter Juno GUI Monitor Node")
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Data storage
        self.current_ear = 0.0
        self.is_drowsy = False
        self.last_speech = ""
        self.last_tts = ""
        self.system_state = "Initializing"
        self.current_image = None
        self.last_update_time = None
        self.node_status = "Starting..."
        
        # Setup GUI
        if not HAS_QT:
            rospy.logerr("Qt not available - cannot create GUI")
            return
            
        self.setWindowTitle("Jupiter Juno Drowsiness Detection Monitor")
        self.setGeometry(100, 100, 1400, 900)
        self.setMinimumSize(1200, 800)
        
        # Connect signals to slots for thread-safe updates
        self.ear_signal.connect(self.update_ear_display)
        self.drowsiness_signal.connect(self.update_drowsiness_display)
        self.image_signal.connect(self.update_image_display)
        self.speech_signal.connect(self.update_speech_display)
        self.tts_signal.connect(self.update_tts_display)
        self.state_signal.connect(self.update_state_display)
        
        self.setup_gui()
        self.setup_ros_subscribers()
        
        # Setup update timer for connection status
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_connection_status)
        self.status_timer.start(1000)  # Update connection status every second
        
        # Setup display refresh timer
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self.refresh_display)
        self.display_timer.start(100)  # Refresh at 10Hz
        
        rospy.loginfo("Jupiter Juno GUI Monitor Node initialized successfully")
    
    def setup_gui(self):
        """Setup the GUI layout and widgets"""
        # Main layout
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(15, 15, 15, 15)
        
        # Title
        title = QLabel("Jupiter Juno Drowsiness Detection Monitor")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            font-size: 20px; 
            font-weight: bold; 
            color: #2c3e50; 
            padding: 15px;
            background-color: #ecf0f1;
            border-radius: 8px;
            margin-bottom: 10px;
        """)
        main_layout.addWidget(title)
        
        # Top row - Camera and Status
        top_layout = QHBoxLayout()
        top_layout.setSpacing(15)
        
        # Camera section
        camera_frame = QFrame()
        camera_frame.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        camera_frame.setLineWidth(2)
        camera_layout = QVBoxLayout()
        camera_layout.setContentsMargins(10, 10, 10, 10)
        
        camera_title = QLabel("Live Camera Feed with Face Detection")
        camera_title.setAlignment(Qt.AlignCenter)
        camera_title.setStyleSheet("font-weight: bold; font-size: 14px; padding: 8px; color: #34495e;")
        camera_layout.addWidget(camera_title)
        
        self.camera_label = QLabel()
        self.camera_label.setMinimumSize(640, 480)
        self.camera_label.setMaximumSize(800, 600)
        self.camera_label.setStyleSheet("""
            border: 3px solid #34495e; 
            background-color: #ecf0f1;
            border-radius: 8px;
        """)
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setText("Waiting for camera feed...")
        self.camera_label.setScaledContents(True)
        camera_layout.addWidget(self.camera_label)
        
        camera_frame.setLayout(camera_layout)
        top_layout.addWidget(camera_frame, 2)  # Give camera section more space
        
        # Status section
        status_frame = QFrame()
        status_frame.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        status_frame.setLineWidth(2)
        status_frame.setMinimumWidth(350)
        status_frame.setMaximumWidth(400)
        status_layout = QVBoxLayout()
        status_layout.setContentsMargins(15, 10, 15, 10)
        status_layout.setSpacing(10)
        
        status_title = QLabel("System Status")
        status_title.setAlignment(Qt.AlignCenter)
        status_title.setStyleSheet("font-weight: bold; font-size: 16px; padding: 8px; color: #34495e;")
        status_layout.addWidget(status_title)
        
        # EAR Display
        self.ear_label = QLabel("EAR: 0.000")
        self.ear_label.setStyleSheet("""
            font-size: 18px; 
            font-weight: bold;
            padding: 10px; 
            border: 2px solid #bdc3c7;
            border-radius: 5px;
            background-color: #f8f9fa;
            color: #2c3e50;
        """)
        self.ear_label.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.ear_label)
        
        # EAR Progress Bar
        ear_progress_label = QLabel("Eye Openness Level")
        ear_progress_label.setStyleSheet("font-size: 12px; color: #7f8c8d; margin-top: 5px;")
        status_layout.addWidget(ear_progress_label)
        
        self.ear_progress = QProgressBar()
        self.ear_progress.setRange(0, 50)  # EAR typically 0.0 to 0.5
        self.ear_progress.setValue(25)
        self.ear_progress.setStyleSheet("""
            QProgressBar { 
                border: 2px solid #bdc3c7; 
                border-radius: 8px; 
                text-align: center; 
                font-weight: bold;
                background-color: #ecf0f1;
            }
            QProgressBar::chunk { 
                background-color: #27ae60; 
                border-radius: 6px;
            }
        """)
        status_layout.addWidget(self.ear_progress)
        
        # Drowsiness Status
        self.drowsy_label = QLabel("Status: ALERT")
        self.drowsy_label.setAlignment(Qt.AlignCenter)
        self.drowsy_label.setStyleSheet("""
            font-size: 16px; 
            font-weight: bold; 
            padding: 12px; 
            border-radius: 8px; 
            background-color: #2ecc71; 
            color: white;
            margin: 5px 0px;
        """)
        status_layout.addWidget(self.drowsy_label)
        
        # System State
        self.state_label = QLabel("System: Initializing")
        self.state_label.setStyleSheet("""
            font-size: 13px; 
            padding: 8px; 
            border: 1px solid #bdc3c7;
            border-radius: 5px;
            background-color: #f8f9fa;
            color: #34495e;
        """)
        status_layout.addWidget(self.state_label)
        
        # Last Update Time
        self.time_label = QLabel("Last Update: --:--:--")
        self.time_label.setStyleSheet("font-size: 11px; color: #7f8c8d; padding: 3px;")
        status_layout.addWidget(self.time_label)
        
        # Connection Status
        self.connection_label = QLabel("ROS: Connecting...")
        self.connection_label.setStyleSheet("font-size: 11px; color: #e74c3c; padding: 3px; font-weight: bold;")
        status_layout.addWidget(self.connection_label)
        
        # Add stretch to push everything to top
        status_layout.addStretch()
        
        status_frame.setLayout(status_layout)
        top_layout.addWidget(status_frame, 1)  # Give status section less space
        
        main_layout.addLayout(top_layout)
        
        # Bottom row - Speech and TTS
        bottom_layout = QHBoxLayout()
        bottom_layout.setSpacing(15)
        
        # Speech Recognition section
        speech_frame = QFrame()
        speech_frame.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        speech_frame.setLineWidth(2)
        speech_layout = QVBoxLayout()
        speech_layout.setContentsMargins(10, 10, 10, 10)
        
        speech_title = QLabel("🎤 Speech Recognition")
        speech_title.setAlignment(Qt.AlignCenter)
        speech_title.setStyleSheet("font-weight: bold; font-size: 14px; padding: 8px; color: #34495e;")
        speech_layout.addWidget(speech_title)
        
        self.speech_display = QTextEdit()
        self.speech_display.setMaximumHeight(150)
        self.speech_display.setStyleSheet("""
            border: 2px solid #bdc3c7; 
            background-color: #f8f9fa;
            border-radius: 5px;
            font-family: 'Courier New', monospace;
            font-size: 12px;
            padding: 5px;
        """)
        self.speech_display.setPlainText("Waiting for speech input...")
        speech_layout.addWidget(self.speech_display)
        
        speech_clear_btn = QPushButton("Clear Speech Log")
        speech_clear_btn.clicked.connect(self.clear_speech_log)
        speech_clear_btn.setStyleSheet("""
            QPushButton {
                padding: 6px 12px;
                border: 1px solid #3498db;
                border-radius: 4px;
                background-color: #3498db;
                color: white;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
        """)
        speech_layout.addWidget(speech_clear_btn)
        
        speech_frame.setLayout(speech_layout)
        bottom_layout.addWidget(speech_frame)
        
        # TTS section
        tts_frame = QFrame()
        tts_frame.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        tts_frame.setLineWidth(2)
        tts_layout = QVBoxLayout()
        tts_layout.setContentsMargins(10, 10, 10, 10)
        
        tts_title = QLabel("🔊 Text-to-Speech Output")
        tts_title.setAlignment(Qt.AlignCenter)
        tts_title.setStyleSheet("font-weight: bold; font-size: 14px; padding: 8px; color: #34495e;")
        tts_layout.addWidget(tts_title)
        
        self.tts_display = QTextEdit()
        self.tts_display.setMaximumHeight(150)
        self.tts_display.setStyleSheet("""
            border: 2px solid #bdc3c7; 
            background-color: #f8f9fa;
            border-radius: 5px;
            font-family: 'Courier New', monospace;
            font-size: 12px;
            padding: 5px;
        """)
        self.tts_display.setPlainText("No TTS output yet...")
        tts_layout.addWidget(self.tts_display)
        
        tts_clear_btn = QPushButton("Clear TTS Log")
        tts_clear_btn.clicked.connect(self.clear_tts_log)
        tts_clear_btn.setStyleSheet("""
            QPushButton {
                padding: 6px 12px;
                border: 1px solid #e67e22;
                border-radius: 4px;
                background-color: #e67e22;
                color: white;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #d35400;
            }
        """)
        tts_layout.addWidget(tts_clear_btn)
        
        tts_frame.setLayout(tts_layout)
        bottom_layout.addWidget(tts_frame)
        
        main_layout.addLayout(bottom_layout)
        
        self.setLayout(main_layout)
    
    def setup_ros_subscribers(self):
        """Setup ROS subscribers for all topics"""
        try:
            # Eye detection data
            rospy.Subscriber('/jupiter_juno/ear_data', Float32, self.ear_callback, queue_size=10)
            rospy.Subscriber('/jupiter_juno/drowsiness_alert', Bool, self.drowsiness_callback, queue_size=10)
            rospy.Subscriber('/jupiter_juno/eye_detection_image', Image, self.image_callback, queue_size=5)
            
            # Speech and TTS
            rospy.Subscriber('/jupiter_juno/speech_result', String, self.speech_callback, queue_size=10)
            rospy.Subscriber('/jupiter_juno/tts_request', String, self.tts_callback, queue_size=10)
            
            # System state
            rospy.Subscriber('/jupiter_juno/system_state', String, self.state_callback, queue_size=10)
            
            rospy.loginfo("ROS subscribers initialized")
            self.node_status = "Connected"
            
        except Exception as e:
            rospy.logerr(f"Failed to setup ROS subscribers: {e}")
            self.node_status = "Error"
    
    def ear_callback(self, msg):
        """Handle EAR data updates"""
        self.ear_signal.emit(float(msg.data))
    
    def drowsiness_callback(self, msg):
        """Handle drowsiness alert updates"""
        self.drowsiness_signal.emit(bool(msg.data))
    
    def image_callback(self, msg):
        """Handle camera image updates"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_signal.emit(cv_image)
        except Exception as e:
            rospy.logwarn(f"Failed to convert image: {e}")
    
    def speech_callback(self, msg):
        """Handle speech recognition updates"""
        self.speech_signal.emit(str(msg.data))
    
    def tts_callback(self, msg):
        """Handle TTS request updates"""
        self.tts_signal.emit(str(msg.data))
    
    def state_callback(self, msg):
        """Handle system state updates"""
        self.state_signal.emit(str(msg.data))
    
    def update_ear_display(self, ear_value):
        """Update EAR display (Qt slot)"""
        self.current_ear = ear_value
        self.last_update_time = datetime.now()
        
        # Update EAR label
        self.ear_label.setText(f"EAR: {ear_value:.3f}")
        
        # Update progress bar (convert EAR to percentage, typical range 0.0-0.5)
        progress_value = min(50, max(0, int(ear_value * 100)))
        self.ear_progress.setValue(progress_value)
        
        # Change color based on EAR value
        if ear_value < 0.20:  # Very low EAR - likely closed eyes
            progress_color = "#e74c3c"  # Red
        elif ear_value < 0.25:  # Low EAR - getting drowsy
            progress_color = "#f39c12"  # Orange
        else:  # Normal EAR
            progress_color = "#27ae60"  # Green
        
        self.ear_progress.setStyleSheet(f"""
            QProgressBar {{ 
                border: 2px solid #bdc3c7; 
                border-radius: 8px; 
                text-align: center; 
                font-weight: bold;
                background-color: #ecf0f1;
            }}
            QProgressBar::chunk {{ 
                background-color: {progress_color}; 
                border-radius: 6px;
            }}
        """)
    
    def update_drowsiness_display(self, is_drowsy):
        """Update drowsiness status display (Qt slot)"""
        self.is_drowsy = is_drowsy
        
        if is_drowsy:
            self.drowsy_label.setText("⚠️ Status: DROWSY")
            self.drowsy_label.setStyleSheet("""
                font-size: 16px; 
                font-weight: bold; 
                padding: 12px; 
                border-radius: 8px; 
                background-color: #e74c3c; 
                color: white;
                margin: 5px 0px;
            """)
        else:
            self.drowsy_label.setText("✅ Status: ALERT")
            self.drowsy_label.setStyleSheet("""
                font-size: 16px; 
                font-weight: bold; 
                padding: 12px; 
                border-radius: 8px; 
                background-color: #2ecc71; 
                color: white;
                margin: 5px 0px;
            """)
    
    def update_image_display(self, cv_image):
        """Update camera image display (Qt slot)"""
        try:
            self.current_image = cv_image
            
            # Convert to RGB for Qt
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # Scale image to fit label while maintaining aspect ratio
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            
            self.camera_label.setPixmap(scaled_pixmap)
            
        except Exception as e:
            rospy.logwarn(f"Failed to display image: {e}")
    
    def update_speech_display(self, speech_text):
        """Update speech recognition display (Qt slot)"""
        if speech_text and speech_text.strip():
            self.last_speech = speech_text
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.speech_display.append(f"[{timestamp}] {speech_text}")
            
            # Keep only last 10 lines
            content = self.speech_display.toPlainText().split('\n')
            if len(content) > 10:
                self.speech_display.setPlainText('\n'.join(content[-10:]))
    
    def update_tts_display(self, tts_text):
        """Update TTS display (Qt slot)"""
        if tts_text and tts_text.strip():
            self.last_tts = tts_text
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.tts_display.append(f"[{timestamp}] {tts_text}")
            
            # Keep only last 10 lines
            content = self.tts_display.toPlainText().split('\n')
            if len(content) > 10:
                self.tts_display.setPlainText('\n'.join(content[-10:]))
    
    def update_state_display(self, state):
        """Update system state display (Qt slot)"""
        self.system_state = state
        self.state_label.setText(f"System: {state.title()}")
    
    def update_connection_status(self):
        """Update ROS connection status"""
        if rospy.is_shutdown():
            self.connection_label.setText("ROS: Disconnected")
            self.connection_label.setStyleSheet("font-size: 11px; color: #e74c3c; padding: 3px; font-weight: bold;")
        else:
            self.connection_label.setText("ROS: Connected")
            self.connection_label.setStyleSheet("font-size: 11px; color: #27ae60; padding: 3px; font-weight: bold;")
    
    def refresh_display(self):
        """Refresh time display and other dynamic elements"""
        if self.last_update_time:
            time_str = self.last_update_time.strftime("%H:%M:%S")
            self.time_label.setText(f"Last Update: {time_str}")
    
    def clear_speech_log(self):
        """Clear speech recognition log"""
        self.speech_display.setPlainText("Speech log cleared...")
    
    def clear_tts_log(self):
        """Clear TTS log"""
        self.tts_display.setPlainText("TTS log cleared...")
    
    def closeEvent(self, event):
        """Handle window close event"""
        rospy.loginfo("Shutting down Jupiter Juno GUI Monitor")
        if hasattr(self, 'status_timer'):
            self.status_timer.stop()
        if hasattr(self, 'display_timer'):
            self.display_timer.stop()
        
        event.accept()


def main():
    """Main function to run the GUI node"""
    # Initialize QApplication
    app = QApplication(sys.argv)
    app.setApplicationName("Jupiter Juno Monitor")
    app.setApplicationVersion("1.0")
    
    try:
        # Create and show the GUI
        gui_node = JupiterJunoGUINode()
        gui_node.show()
        
        # Set up ROS shutdown hook
        def shutdown_hook():
            app.quit()
        
        rospy.on_shutdown(shutdown_hook)
        
        # Start the Qt event loop
        sys.exit(app.exec_())
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Jupiter Juno GUI Monitor interrupted")
    except Exception as e:
        rospy.logerr(f"Jupiter Juno GUI Monitor error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main() 