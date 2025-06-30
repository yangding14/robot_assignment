#!/usr/bin/env python3

"""
Standalone test script for Jupiter Juno Drowsiness Detection System
This script demonstrates the system functionality without requiring ROS 2
"""

import cv2
import dlib
import numpy as np
from scipy.spatial import distance as dist
import os
import time
import threading
import queue
import yaml

# Try to import optional libraries
try:
    import pyttsx3
    HAS_TTS = True
except:
    HAS_TTS = False
    print("Warning: pyttsx3 not available, TTS disabled")

try:
    import speech_recognition as sr
    HAS_SPEECH = True
except:
    HAS_SPEECH = False
    print("Warning: speech_recognition not available, speech input disabled")


class JupiterJunoStandalone:
    def __init__(self):
        # Load configuration
        self.load_config()
        
        # Initialize components
        self.init_eye_detector()
        self.init_tts()
        self.init_speech_recognition()
        
        # State variables
        self.blink_counter = 0
        self.drowsiness_detected = False
        self.conversation_active = False
        self.running = True
        
        # Message queue
        self.tts_queue = queue.Queue()
        
        print("Jupiter Juno Standalone System Initialized")
        self.speak("Good morning! Ready for a safe drive? I'll keep an eye on your alertness.")
    
    def load_config(self):
        """Load configuration from YAML file"""
        config_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'src/jupiter_juno/config/drowsiness_config.yaml'
        )
        
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            
            # Eye detection parameters
            eye_config = config['eye_detection']
            self.ear_threshold = eye_config['ear_threshold']
            self.consecutive_frames = eye_config['consecutive_frames']
            self.camera_index = eye_config['camera_index']
            
            # Messages
            self.messages = config['messages']
            
            print("Configuration loaded successfully")
            
        except Exception as e:
            print(f"Failed to load config: {e}")
            # Use defaults
            self.ear_threshold = 0.20
            self.consecutive_frames = 48
            self.camera_index = 0
            self.messages = {
                'drowsiness_detected': "Warning: I've detected signs of drowsiness. Please stay alert!",
                'conversation_prompt': "Would you like to hear a quick driving safety tip or a joke to wake you up?",
                'conversation_end': "Great! Let's keep you alert and cruising safely."
            }
    
    def init_eye_detector(self):
        """Initialize eye detection components"""
        # Find the shape predictor file
        predictor_paths = [
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'src/jupiter_juno/jupiter_juno/shape_predictor_68_face_landmarks.dat'),
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '../Eye_Detector_Script/shape_predictor_68_face_landmarks.dat'),
            'shape_predictor_68_face_landmarks.dat'
        ]
        
        predictor_path = None
        for path in predictor_paths:
            if os.path.exists(path):
                predictor_path = path
                break
        
        if not predictor_path:
            raise FileNotFoundError("Shape predictor file not found. Please ensure shape_predictor_68_face_landmarks.dat is available.")
        
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(predictor_path)
        
        # Eye landmark indices
        self.lStart, self.lEnd = 42, 48  # Left eye
        self.rStart, self.rEnd = 36, 42  # Right eye
        
        print(f"Eye detector initialized with predictor: {predictor_path}")
    
    def init_tts(self):
        """Initialize text-to-speech"""
        if HAS_TTS:
            try:
                self.tts_engine = pyttsx3.init()
                self.tts_engine.setProperty('rate', 150)
                
                # Try to set female voice
                voices = self.tts_engine.getProperty('voices')
                if voices:
                    for voice in voices:
                        if 'female' in voice.name.lower():
                            self.tts_engine.setProperty('voice', voice.id)
                            break
                
                # Start TTS thread
                self.tts_thread = threading.Thread(target=self.tts_worker)
                self.tts_thread.start()
                
                print("TTS initialized successfully")
            except Exception as e:
                print(f"Failed to initialize TTS: {e}")
                HAS_TTS = False
    
    def init_speech_recognition(self):
        """Initialize speech recognition"""
        if HAS_SPEECH:
            try:
                self.recognizer = sr.Recognizer()
                self.microphone = sr.Microphone()
                
                # Calibrate for ambient noise
                with self.microphone as source:
                    print("Calibrating for ambient noise...")
                    self.recognizer.adjust_for_ambient_noise(source, duration=2)
                
                print("Speech recognition initialized successfully")
            except Exception as e:
                print(f"Failed to initialize speech recognition: {e}")
                HAS_SPEECH = False
    
    def eye_aspect_ratio(self, eye):
        """Calculate the Eye Aspect Ratio (EAR)"""
        A = dist.euclidean(eye[1], eye[5])
        B = dist.euclidean(eye[2], eye[4])
        C = dist.euclidean(eye[0], eye[3])
        ear = (A + B) / (2.0 * C)
        return ear
    
    def speak(self, text):
        """Add text to TTS queue"""
        if HAS_TTS:
            self.tts_queue.put(text)
        else:
            print(f"[TTS]: {text}")
    
    def tts_worker(self):
        """Worker thread for TTS"""
        while self.running:
            try:
                text = self.tts_queue.get(timeout=0.5)
                if HAS_TTS:
                    self.tts_engine.say(text)
                    self.tts_engine.runAndWait()
                self.tts_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"TTS error: {e}")
    
    def listen_for_speech(self, timeout=5):
        """Listen for speech input"""
        if not HAS_SPEECH:
            # Simulate user input
            print("Speech recognition not available. Please type your response:")
            return input("> ").strip()
        
        try:
            with self.microphone as source:
                print("Listening...")
                audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=10)
                
            print("Recognizing...")
            text = self.recognizer.recognize_google(audio)
            print(f"You said: {text}")
            return text
            
        except sr.WaitTimeoutError:
            print("No speech detected")
            return ""
        except sr.UnknownValueError:
            print("Could not understand speech")
            return ""
        except Exception as e:
            print(f"Speech recognition error: {e}")
            return ""
    
    def handle_conversation(self):
        """Handle conversation with the driver"""
        self.conversation_active = True
        
        # Ask what they want
        self.speak(self.messages['conversation_prompt'])
        time.sleep(4)  # Wait for TTS to complete
        
        # Get user response
        response = self.listen_for_speech()
        
        if not response:
            self.speak("I didn't catch that. Stay alert and drive safely!")
        else:
            response_lower = response.lower()
            
            if any(word in response_lower for word in ['joke', 'funny', 'laugh']):
                self.speak("Alright, here's one: Why did the bicycle fall over? Because it was two-tired!")
            elif any(word in response_lower for word in ['tip', 'advice', 'safety']):
                self.speak("Here's a tip: Take a 15-minute break every 2 hours of driving to stay fresh and alert!")
            else:
                self.speak("Stay focused on the road. Remember, your safety is the top priority!")
        
        time.sleep(3)
        self.speak(self.messages['conversation_end'])
        self.conversation_active = False
    
    def run(self):
        """Main detection loop"""
        cap = cv2.VideoCapture(self.camera_index)
        
        if not cap.isOpened():
            print("Error: Could not open camera")
            return
        
        print("Starting drowsiness detection. Press 'q' to quit.")
        
        while self.running:
            ret, frame = cap.read()
            if not ret:
                continue
            
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            rects = self.detector(gray, 0)
            
            for rect in rects:
                # Get facial landmarks
                shape = self.predictor(gray, rect)
                coords = np.zeros((68, 2), dtype="int")
                for i in range(0, 68):
                    coords[i] = (shape.part(i).x, shape.part(i).y)
                
                # Extract eye coordinates
                leftEye = coords[self.lStart:self.lEnd]
                rightEye = coords[self.rStart:self.rEnd]
                
                # Calculate EAR
                leftEAR = self.eye_aspect_ratio(leftEye)
                rightEAR = self.eye_aspect_ratio(rightEye)
                ear = (leftEAR + rightEAR) / 2.0
                
                # Draw eye contours
                leftHull = cv2.convexHull(leftEye)
                rightHull = cv2.convexHull(rightEye)
                cv2.drawContours(frame, [leftHull], -1, (0, 255, 0), 1)
                cv2.drawContours(frame, [rightHull], -1, (0, 255, 0), 1)
                
                # Check for drowsiness
                if ear < self.ear_threshold:
                    self.blink_counter += 1
                    if self.blink_counter >= self.consecutive_frames:
                        if not self.drowsiness_detected and not self.conversation_active:
                            self.drowsiness_detected = True
                            print("DROWSINESS DETECTED!")
                            
                            # Trigger alert sequence
                            alert_thread = threading.Thread(target=self.alert_sequence)
                            alert_thread.start()
                else:
                    if self.blink_counter >= self.consecutive_frames:
                        self.drowsiness_detected = False
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
            
            # Show frame
            cv2.imshow("Jupiter Juno - Driver Monitoring", frame)
            
            # Check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Cleanup
        self.running = False
        cap.release()
        cv2.destroyAllWindows()
        
        # Wait for threads
        if HAS_TTS and hasattr(self, 'tts_thread'):
            self.tts_thread.join()
        
        print("System shutdown complete")
    
    def alert_sequence(self):
        """Execute alert sequence when drowsiness detected"""
        # Play alert sound (simulated with prints)
        print("\n🚨 ALERT! ALERT! ALERT! 🚨")
        print("*BEEP* *BEEP* *BEEP*")
        
        # Speak warning
        self.speak(self.messages['drowsiness_detected'])
        time.sleep(3)
        
        # Start conversation
        self.handle_conversation()
        
        # Reset drowsiness flag
        self.drowsiness_detected = False


def main():
    """Main entry point"""
    print("=== Jupiter Juno Drowsiness Detection System (Standalone) ===")
    print("\nThis is a demonstration version that runs without ROS 2.")
    print("For full functionality, please use the ROS 2 version.\n")
    
    try:
        system = JupiterJunoStandalone()
        system.run()
    except KeyboardInterrupt:
        print("\nShutdown requested")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main() 