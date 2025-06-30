#!/usr/bin/env python3

import os
import yaml
import threading
import queue

import rospy
from std_msgs.msg import String, Bool

try:
    import speech_recognition as sr
    HAS_SPEECH_RECOGNITION = True
except ImportError:
    HAS_SPEECH_RECOGNITION = False
    print("Warning: speech_recognition not available")


class SpeechRecognitionNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('speech_recognition_node', anonymous=True)
        
        if not HAS_SPEECH_RECOGNITION:
            rospy.logerr("speech_recognition library not available!")
            raise ImportError("speech_recognition library required")
        
        # Load configuration
        self.load_config()
        
        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Adjust for ambient noise
        with self.microphone as source:
            rospy.loginfo("Calibrating for ambient noise...")
            self.recognizer.adjust_for_ambient_noise(source, duration=2)
            self.recognizer.energy_threshold = self.energy_threshold
        
        # State management
        self.is_listening = False
        self.listen_queue = queue.Queue()
        
        # Publishers
        self.speech_result_publisher = rospy.Publisher(
            self.speech_result_topic,
            String,
            queue_size=10
        )
        
        # Subscribers
        self.state_subscriber = rospy.Subscriber(
            self.system_state_topic,
            String,
            self.state_callback,
            queue_size=10
        )
        
        # Start listening thread
        self.running = True
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.start()
        
        rospy.loginfo("Speech Recognition Node initialized successfully")
    
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
            
            # Speech recognition parameters
            sr_config = config['speech_recognition']
            self.engine = sr_config['engine']
            self.listen_timeout = sr_config['listen_timeout']
            self.phrase_time_limit = sr_config['phrase_time_limit']
            self.energy_threshold = sr_config['energy_threshold']
            
            # ROS topics
            topics = config['ros_topics']
            self.speech_result_topic = topics['speech_result']
            self.system_state_topic = topics['system_state']
            
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            # Use defaults
            self.engine = 'google'
            self.listen_timeout = 5
            self.phrase_time_limit = 10
            self.energy_threshold = 1000
            self.speech_result_topic = '/jupiter_juno/speech_result'
            self.system_state_topic = '/jupiter_juno/system_state'
    
    def state_callback(self, msg):
        """Handle system state changes"""
        state = msg.data
        
        if state == "conversation_ready":
            # Start listening for user response
            rospy.loginfo("Starting to listen for user response")
            self.listen_queue.put("start")
        elif state == "conversation_ended":
            # Stop listening
            self.is_listening = False
    
    def listen_loop(self):
        """Main listening loop running in separate thread"""
        while self.running and not rospy.is_shutdown():
            try:
                # Wait for listen command
                command = self.listen_queue.get(timeout=0.5)
                
                if command == "start":
                    self.is_listening = True
                    self.listen_for_speech()
                    
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error in listen loop: {e}")
    
    def listen_for_speech(self):
        """Listen for speech and publish results"""
        try:
            with self.microphone as source:
                rospy.loginfo("Listening for speech...")
                
                # Listen for speech with timeout
                try:
                    audio = self.recognizer.listen(
                        source,
                        timeout=self.listen_timeout,
                        phrase_time_limit=self.phrase_time_limit
                    )
                except sr.WaitTimeoutError:
                    rospy.logwarn("No speech detected within timeout")
                    self.publish_result("")
                    return
                
                # Recognize speech
                rospy.loginfo("Recognizing speech...")
                text = self.recognize_speech(audio)
                
                if text:
                    rospy.loginfo(f"Recognized: {text}")
                    self.publish_result(text)
                else:
                    rospy.logwarn("Could not understand speech")
                    self.publish_result("")
                    
        except Exception as e:
            rospy.logerr(f"Error listening for speech: {e}")
            self.publish_result("")
        finally:
            self.is_listening = False
    
    def recognize_speech(self, audio):
        """Recognize speech from audio using configured engine"""
        text = None
        
        try:
            if self.engine == 'google':
                # Use Google Speech Recognition
                text = self.recognizer.recognize_google(audio)
            elif self.engine == 'sphinx':
                # Use CMU Sphinx (offline)
                try:
                    text = self.recognizer.recognize_sphinx(audio)
                except sr.UnknownValueError:
                    rospy.logwarn("Sphinx could not understand audio")
                except sr.RequestError as e:
                    rospy.logerr(f"Sphinx error: {e}")
            else:
                # Default to Google
                text = self.recognizer.recognize_google(audio)
                
        except sr.UnknownValueError:
            rospy.logwarn("Speech recognition could not understand audio")
        except sr.RequestError as e:
            rospy.logerr(f"Could not request results from speech recognition service: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error in speech recognition: {e}")
        
        return text
    
    def publish_result(self, text):
        """Publish speech recognition result"""
        msg = String()
        msg.data = text if text else ""
        self.speech_result_publisher.publish(msg)
        rospy.loginfo(f"Published speech result: {msg.data}")
    
    def shutdown(self):
        """Clean up resources"""
        self.running = False
        self.is_listening = False
        
        # Stop listening thread
        if hasattr(self, 'listen_thread'):
            self.listen_thread.join(timeout=2.0)


def main():
    try:
        node = SpeechRecognitionNode()
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