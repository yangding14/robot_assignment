#!/usr/bin/env python3

import cv2
import dlib
import numpy as np
from scipy.spatial import distance as dist
import os
import yaml

import rospy
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading


class EyeDetectorNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('eye_detector_node', anonymous=True)
        
        # Load configuration
        self.load_config()
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize dlib's face detector and shape predictor
        current_path = os.path.dirname(os.path.abspath(__file__))
        predictor_path = os.path.join(current_path, "shape_predictor_68_face_landmarks.dat")
        
        if not os.path.exists(predictor_path):
            rospy.logerr(f"Shape predictor file not found at: {predictor_path}")
            raise FileNotFoundError(f"Shape predictor file not found at: {predictor_path}")
            
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(predictor_path)
        
        # Eye landmark indices
        self.lStart, self.lEnd = 42, 48  # Left eye
        self.rStart, self.rEnd = 36, 42  # Right eye
        
        # Initialize counters
        self.blink_counter = 0
        self.drowsiness_detected = False
        
        # Publishers
        self.ear_publisher = rospy.Publisher(self.ear_topic, Float32, queue_size=10)
        self.drowsiness_publisher = rospy.Publisher(self.drowsiness_topic, Bool, queue_size=10)
        self.image_publisher = rospy.Publisher('/jupiter_juno/eye_detection_image', Image, queue_size=10)
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera")
            raise RuntimeError("Failed to open camera")
        
        # Start detection thread
        self.running = True
        self.detection_thread = threading.Thread(target=self.detection_loop)
        self.detection_thread.start()
        
        rospy.loginfo("Eye Detector Node initialized successfully")
    
    def load_config(self):
        """Load configuration from YAML file"""
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config',
            'drowsiness_config.yaml'
        )
        
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                
            # Eye detection parameters
            eye_config = config['eye_detection']
            self.ear_threshold = eye_config['ear_threshold']
            self.consecutive_frames = eye_config['consecutive_frames']
            self.camera_index = eye_config['camera_index']
            self.processing_fps = eye_config['processing_fps']
            
            # ROS topics
            topics = config['ros_topics']
            self.ear_topic = topics['ear_data']
            self.drowsiness_topic = topics['drowsiness_alert']
            
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            # Use defaults
            self.ear_threshold = 0.20
            self.consecutive_frames = 48
            self.camera_index = 0
            self.processing_fps = 24
            self.ear_topic = '/jupiter_juno/ear_data'
            self.drowsiness_topic = '/jupiter_juno/drowsiness_alert'
    
    def eye_aspect_ratio(self, eye):
        """Calculate the Eye Aspect Ratio (EAR)"""
        # Compute the euclidean distances between the vertical eye landmarks
        A = dist.euclidean(eye[1], eye[5])
        B = dist.euclidean(eye[2], eye[4])
        # Compute the euclidean distance between the horizontal landmarks
        C = dist.euclidean(eye[0], eye[3])
        # Compute and return the eye aspect ratio
        ear = (A + B) / (2.0 * C)
        return ear
    
    def detection_loop(self):
        """Main detection loop running in separate thread"""
        rate = rospy.Rate(self.processing_fps)
        
        while self.running and not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to read frame from camera")
                continue
            
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            rects = self.detector(gray, 0)
            
            ear_values = []
            
            for rect in rects:
                # Get facial landmarks
                shape = self.predictor(gray, rect)
                coords = np.zeros((68, 2), dtype="int")
                for i in range(0, 68):
                    coords[i] = (shape.part(i).x, shape.part(i).y)
                
                # Extract eye coordinates
                leftEye = coords[self.lStart:self.lEnd]
                rightEye = coords[self.rStart:self.rEnd]
                
                # Calculate EAR for both eyes
                leftEAR = self.eye_aspect_ratio(leftEye)
                rightEAR = self.eye_aspect_ratio(rightEye)
                ear = (leftEAR + rightEAR) / 2.0
                ear_values.append(ear)
                
                # Draw eye contours
                leftHull = cv2.convexHull(leftEye)
                rightHull = cv2.convexHull(rightEye)
                cv2.drawContours(frame, [leftHull], -1, (0, 255, 0), 1)
                cv2.drawContours(frame, [rightHull], -1, (0, 255, 0), 1)
                
                # Check for drowsiness
                if ear < self.ear_threshold:
                    self.blink_counter += 1
                    if self.blink_counter >= self.consecutive_frames:
                        if not self.drowsiness_detected:
                            self.drowsiness_detected = True
                            self.publish_drowsiness_alert(True)
                            rospy.logwarn("Drowsiness detected!")
                else:
                    if self.blink_counter >= self.consecutive_frames and self.drowsiness_detected:
                        self.drowsiness_detected = False
                        self.publish_drowsiness_alert(False)
                    self.blink_counter = 0
                
                # Add text overlays
                status = "DROWSY" if self.drowsiness_detected else "ALERT"
                color = (0, 0, 255) if self.drowsiness_detected else (0, 255, 0)
                
                cv2.putText(frame, f"EAR: {ear:.3f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, f"Status: {status}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.putText(frame, f"Blink Count: {self.blink_counter}", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publish average EAR if faces detected
            if ear_values:
                avg_ear = sum(ear_values) / len(ear_values)
                self.publish_ear(avg_ear)
            
            # Publish processed image
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_publisher.publish(img_msg)
            except Exception as e:
                rospy.logerr(f"Failed to publish image: {e}")
            
            rate.sleep()
    
    def publish_ear(self, ear_value):
        """Publish Eye Aspect Ratio value"""
        msg = Float32()
        msg.data = float(ear_value)
        self.ear_publisher.publish(msg)
    
    def publish_drowsiness_alert(self, is_drowsy):
        """Publish drowsiness alert"""
        msg = Bool()
        msg.data = is_drowsy
        self.drowsiness_publisher.publish(msg)
    
    def shutdown(self):
        """Clean up resources"""
        self.running = False
        if hasattr(self, 'detection_thread'):
            self.detection_thread.join()
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()


def main():
    try:
        node = EyeDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.shutdown()


if __name__ == '__main__':
    main() 