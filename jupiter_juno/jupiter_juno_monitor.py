#!/usr/bin/env python3

import os
import cv2
import numpy as np
from datetime import datetime

import rospy
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from python_qt_binding import loadUi
    from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit, QProgressBar, QFrame
    from python_qt_binding.QtCore import Qt, QTimer, pyqtSignal
    from python_qt_binding.QtGui import QPixmap, QImage, QFont, QPalette, QColor
    from rqt_gui_py.plugin import Plugin
    HAS_QT = True
except ImportError:
    HAS_QT = False
    print("Warning: Qt not available, GUI will not work")


class JupiterJunoMonitor(Plugin):
    """
    RQt plugin for monitoring Jupiter Juno drowsiness detection system
    """
    
    def __init__(self, context):
        super(JupiterJunoMonitor, self).__init__(context)
        
        # Initialize ROS
        if not hasattr(rospy, '_initialized') or not rospy._initialized:
            rospy.init_node('jupiter_juno_monitor', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Data storage
        self.current_ear = 0.0
        self.is_drowsy = False
        self.last_speech = ""
        self.last_tts = ""
        self.system_state = "initializing"
        self.current_image = None
        
        # Create GUI
        self.setObjectName('JupiterJunoMonitor')
        self._widget = QWidget()
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number())
            )
        
        self.setup_gui()
        context.add_widget(self._widget)
        
        # Setup ROS subscribers
        self.setup_ros_subscribers()
        
        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # Update at 10Hz
    
    def setup_gui(self):
        """Setup the GUI layout and widgets"""
        if not HAS_QT:
            return
            
        # Main layout
        main_layout = QVBoxLayout()
        
        # Title
        title = QLabel("Jupiter Juno Drowsiness Detection Monitor")
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
        
        self.clear_speech_btn = QPushButton("Clear Speech Log")
        self.clear_speech_btn.clicked.connect(self.clear_speech_log)
        self.clear_speech_btn.setStyleSheet("padding: 5px;")
        button_layout.addWidget(self.clear_speech_btn)
        
        self.clear_tts_btn = QPushButton("Clear TTS Log")
        self.clear_tts_btn.clicked.connect(self.clear_tts_log)
        self.clear_tts_btn.setStyleSheet("padding: 5px;")
        button_layout.addWidget(self.clear_tts_btn)
        
        main_layout.addLayout(button_layout)
        
        self._widget.setLayout(main_layout)
    
    def setup_ros_subscribers(self):
        """Setup ROS topic subscribers"""
        self.ear_subscriber = rospy.Subscriber(
            '/jupiter_juno/ear_data',
            Float32,
            self.ear_callback,
            queue_size=10
        )
        
        self.drowsiness_subscriber = rospy.Subscriber(
            '/jupiter_juno/drowsiness_alert',
            Bool,
            self.drowsiness_callback,
            queue_size=10
        )
        
        self.image_subscriber = rospy.Subscriber(
            '/jupiter_juno/eye_detection_image',
            Image,
            self.image_callback,
            queue_size=1
        )
        
        self.speech_subscriber = rospy.Subscriber(
            '/jupiter_juno/speech_result',
            String,
            self.speech_callback,
            queue_size=10
        )
        
        self.tts_subscriber = rospy.Subscriber(
            '/jupiter_juno/tts_request',
            String,
            self.tts_callback,
            queue_size=10
        )
        
        self.state_subscriber = rospy.Subscriber(
            '/jupiter_juno/system_state',
            String,
            self.state_callback,
            queue_size=10
        )
    
    def ear_callback(self, msg):
        """Handle EAR data updates"""
        self.current_ear = msg.data
    
    def drowsiness_callback(self, msg):
        """Handle drowsiness alert updates"""
        self.is_drowsy = msg.data
    
    def image_callback(self, msg):
        """Handle camera image updates"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
    
    def speech_callback(self, msg):
        """Handle speech recognition updates"""
        if msg.data and msg.data.strip():
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.last_speech = f"[{timestamp}] User: {msg.data}"
    
    def tts_callback(self, msg):
        """Handle TTS request updates"""
        if msg.data and msg.data.strip():
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.last_tts = f"[{timestamp}] System: {msg.data}"
    
    def state_callback(self, msg):
        """Handle system state updates"""
        self.system_state = msg.data
    
    def update_display(self):
        """Update the GUI display with current data"""
        if not HAS_QT:
            return
            
        # Update EAR display
        self.ear_label.setText(f"EAR: {self.current_ear:.3f}")
        
        # Update EAR progress bar (scale 0.0-0.5 to 0-50)
        ear_scaled = int(self.current_ear * 100)
        self.ear_progress.setValue(ear_scaled)
        
        # Update drowsiness status
        if self.is_drowsy:
            self.drowsy_label.setText("Status: DROWSY!")
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
        self.state_label.setText(f"System: {self.system_state.replace('_', ' ').title()}")
        
        # Update timestamp
        self.time_label.setText(f"Last Update: {datetime.now().strftime('%H:%M:%S')}")
        
        # Update camera feed
        if self.current_image is not None:
            self.display_image(self.current_image)
        
        # Update speech log
        if self.last_speech:
            current_text = self.speech_display.toPlainText()
            if "Waiting for speech input..." in current_text:
                self.speech_display.setPlainText(self.last_speech)
            else:
                self.speech_display.append(self.last_speech)
            self.speech_display.verticalScrollBar().setValue(
                self.speech_display.verticalScrollBar().maximum()
            )
            self.last_speech = ""
        
        # Update TTS log
        if self.last_tts:
            current_text = self.tts_display.toPlainText()
            if "No TTS output yet..." in current_text:
                self.tts_display.setPlainText(self.last_tts)
            else:
                self.tts_display.append(self.last_tts)
            self.tts_display.verticalScrollBar().setValue(
                self.tts_display.verticalScrollBar().maximum()
            )
            self.last_tts = ""
    
    def display_image(self, cv_image):
        """Display OpenCV image in Qt label"""
        try:
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            # Create QImage
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            # Scale to fit label while maintaining aspect ratio
            label_size = self.camera_label.size()
            scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
                label_size.width() - 4,  # Account for border
                label_size.height() - 4,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
            
            self.camera_label.setPixmap(scaled_pixmap)
            
        except Exception as e:
            rospy.logerr(f"Error displaying image: {e}")
    
    def clear_speech_log(self):
        """Clear the speech recognition log"""
        self.speech_display.setPlainText("Speech log cleared...")
    
    def clear_tts_log(self):
        """Clear the TTS log"""
        self.tts_display.setPlainText("TTS log cleared...")
    
    def shutdown_plugin(self):
        """Clean shutdown of the plugin"""
        if hasattr(self, 'update_timer'):
            self.update_timer.stop()
        
        # Unregister subscribers
        if hasattr(self, 'ear_subscriber'):
            self.ear_subscriber.unregister()
        if hasattr(self, 'drowsiness_subscriber'):
            self.drowsiness_subscriber.unregister()
        if hasattr(self, 'image_subscriber'):
            self.image_subscriber.unregister()
        if hasattr(self, 'speech_subscriber'):
            self.speech_subscriber.unregister()
        if hasattr(self, 'tts_subscriber'):
            self.tts_subscriber.unregister()
        if hasattr(self, 'state_subscriber'):
            self.state_subscriber.unregister()
    
    def save_settings(self, plugin_settings, instance_settings):
        """Save plugin settings"""
        pass
    
    def restore_settings(self, plugin_settings, instance_settings):
        """Restore plugin settings"""
        pass 