#!/usr/bin/env python3

import os
import yaml
import threading
import time
import numpy as np

import rospy
from std_msgs.msg import Bool, String

# Audio libraries
try:
    import simpleaudio as sa
except ImportError:
    sa = None
    print("Warning: simpleaudio not available, using alternative")

try:
    import pygame
    pygame.mixer.init()
except ImportError:
    pygame = None


class AlertSystemNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('alert_system_node', anonymous=True)
        
        # Load configuration
        self.load_config()
        
        # State management
        self.is_alerting = False
        self.alert_thread = None
        self.conversation_active = False
        
        # Subscribers
        self.drowsiness_subscriber = rospy.Subscriber(
            self.drowsiness_topic,
            Bool,
            self.drowsiness_callback,
            queue_size=10
        )
        
        self.system_state_subscriber = rospy.Subscriber(
            self.system_state_topic,
            String,
            self.system_state_callback,
            queue_size=10
        )
        
        # Publishers
        self.tts_publisher = rospy.Publisher(self.tts_topic, String, queue_size=10)
        self.state_publisher = rospy.Publisher(self.system_state_topic, String, queue_size=10)
        
        # Generate alert sound
        self.generate_alert_sound()
        
        # Send startup message
        rospy.sleep(1.0)  # Give time for publishers to connect
        self.send_tts_request(self.messages['startup'])
        
        rospy.loginfo("Alert System Node initialized successfully")
    
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
            
            # Alert parameters
            alert_config = config['alert_system']
            self.alert_duration = alert_config['alert_duration']
            self.alert_frequency = alert_config['alert_frequency']
            self.alert_volume = alert_config['alert_volume']
            
            # Messages
            self.messages = config['messages']
            
            # ROS topics
            topics = config['ros_topics']
            self.drowsiness_topic = topics['drowsiness_alert']
            self.tts_topic = topics['tts_request']
            self.system_state_topic = topics['system_state']
            
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            # Use defaults
            self.alert_duration = 3
            self.alert_frequency = 1000
            self.alert_volume = 0.7
            self.messages = {
                'startup': "Jupiter Juno initialized.",
                'drowsiness_detected': "Warning: Drowsiness detected!",
                'conversation_prompt': "Would you like to hear a tip or joke?"
            }
            self.drowsiness_topic = '/jupiter_juno/drowsiness_alert'
            self.tts_topic = '/jupiter_juno/tts_request'
            self.system_state_topic = '/jupiter_juno/system_state'
    
    def generate_alert_sound(self):
        """Generate alert beep sound"""
        sample_rate = 44100
        duration = 0.5  # Half second beep
        
        # Generate sine wave
        t = np.linspace(0, duration, int(sample_rate * duration))
        frequency = self.alert_frequency
        wave = np.sin(frequency * 2 * np.pi * t)
        
        # Apply envelope to avoid clicks
        envelope = np.ones_like(wave)
        fade_samples = int(0.05 * sample_rate)  # 50ms fade
        envelope[:fade_samples] = np.linspace(0, 1, fade_samples)
        envelope[-fade_samples:] = np.linspace(1, 0, fade_samples)
        wave = wave * envelope * self.alert_volume
        
        # Convert to 16-bit PCM
        self.alert_audio = (wave * 32767).astype(np.int16)
        self.alert_sample_rate = sample_rate
    
    def play_alert_sound(self):
        """Play the alert sound"""
        try:
            if sa:
                # Use simpleaudio
                play_obj = sa.play_buffer(
                    self.alert_audio,
                    1,  # num channels
                    2,  # bytes per sample
                    self.alert_sample_rate
                )
                play_obj.wait_done()
            elif pygame:
                # Use pygame as fallback
                import tempfile
                import wave
                
                # Save to temporary WAV file
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
                    with wave.open(tmp_file.name, 'w') as wav_file:
                        wav_file.setnchannels(1)
                        wav_file.setsampwidth(2)
                        wav_file.setframerate(self.alert_sample_rate)
                        wav_file.writeframes(self.alert_audio.tobytes())
                    
                    # Play with pygame
                    pygame.mixer.music.load(tmp_file.name)
                    pygame.mixer.music.play()
                    while pygame.mixer.music.get_busy():
                        time.sleep(0.1)
                    
                    # Clean up
                    os.unlink(tmp_file.name)
            else:
                rospy.logwarn("No audio backend available for alert sound")
                
        except Exception as e:
            rospy.logerr(f"Failed to play alert sound: {e}")
    
    def drowsiness_callback(self, msg):
        """Handle drowsiness detection events"""
        if msg.data and not self.is_alerting and not self.conversation_active:
            rospy.loginfo("Drowsiness alert received - starting alert sequence")
            self.is_alerting = True
            
            # Start alert in separate thread
            self.alert_thread = threading.Thread(target=self.alert_sequence)
            self.alert_thread.start()
    
    def alert_sequence(self):
        """Execute the full alert sequence"""
        try:
            # Play alert sound for specified duration
            end_time = time.time() + self.alert_duration
            while time.time() < end_time and self.is_alerting:
                self.play_alert_sound()
                time.sleep(0.5)  # Short pause between beeps
            
            # Send drowsiness message
            self.send_tts_request(self.messages['drowsiness_detected'])
            time.sleep(3)  # Wait for TTS to complete
            
            # Send conversation prompt and wait for it to complete
            self.send_tts_request(self.messages['conversation_prompt'])
            time.sleep(4)  # Wait longer for prompt to complete
            
            # Only AFTER prompt is spoken, signal conversation ready
            self.publish_state("conversation_ready")
            
        except Exception as e:
            rospy.logerr(f"Error in alert sequence: {e}")
        finally:
            self.is_alerting = False
    
    def system_state_callback(self, msg):
        """Handle system state updates"""
        state = msg.data
        
        if state == "conversation_started":
            self.conversation_active = True
            self.is_alerting = False  # Stop alerts during conversation
        elif state == "conversation_ended":
            self.conversation_active = False
            # Send end message after a short delay to ensure clean transition
            def send_end_message():
                try:
                    time.sleep(1)  # Short delay for clean transition
                    # self.send_tts_request(self.messages['conversation_end'])
                except Exception as e:
                    rospy.logerr(f"Error sending end message: {e}")
            
            end_thread = threading.Thread(target=send_end_message)
            end_thread.daemon = True  # Make thread daemon so it doesn't block shutdown
            end_thread.start()
        elif state == "shutdown":
            # Send shutdown message
            self.send_tts_request(self.messages['shutdown'])
    
    def send_tts_request(self, text):
        """Send text to TTS node"""
        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)
        rospy.loginfo(f"TTS request sent: {text}")
    
    def publish_state(self, state):
        """Publish system state"""
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)
    
    def shutdown(self):
        """Clean up resources"""
        self.is_alerting = False
        if self.alert_thread and self.alert_thread.is_alive():
            self.alert_thread.join(timeout=1.0)


def main():
    try:
        node = AlertSystemNode()
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