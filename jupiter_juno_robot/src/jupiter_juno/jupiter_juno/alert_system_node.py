#!/usr/bin/env python3

import os
import yaml
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
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


class AlertSystemNode(Node):
    def __init__(self):
        super().__init__('alert_system_node')
        
        # Load configuration
        self.load_config()
        
        # State management
        self.is_alerting = False
        self.alert_thread = None
        self.conversation_active = False
        
        # Subscribers
        self.drowsiness_subscriber = self.create_subscription(
            Bool,
            self.drowsiness_topic,
            self.drowsiness_callback,
            10
        )
        
        self.system_state_subscriber = self.create_subscription(
            String,
            self.system_state_topic,
            self.system_state_callback,
            10
        )
        
        # Publishers
        self.tts_publisher = self.create_publisher(String, self.tts_topic, 10)
        self.state_publisher = self.create_publisher(String, self.system_state_topic, 10)
        
        # Generate alert sound
        self.generate_alert_sound()
        
        # Send startup message
        self.send_tts_request(self.messages['startup'])
        
        self.get_logger().info("Alert System Node initialized successfully")
    
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
            self.get_logger().error(f"Failed to load config: {e}")
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
                self.get_logger().warn("No audio backend available for alert sound")
                
        except Exception as e:
            self.get_logger().error(f"Failed to play alert sound: {e}")
    
    def drowsiness_callback(self, msg):
        """Handle drowsiness detection events"""
        if msg.data and not self.is_alerting and not self.conversation_active:
            self.get_logger().info("Drowsiness alert received - starting alert sequence")
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
            
            # Update state to trigger conversation
            self.publish_state("conversation_ready")
            
            # Send conversation prompt
            self.send_tts_request(self.messages['conversation_prompt'])
            
        except Exception as e:
            self.get_logger().error(f"Error in alert sequence: {e}")
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
            # Send end message
            self.send_tts_request(self.messages['conversation_end'])
        elif state == "shutdown":
            # Send shutdown message
            self.send_tts_request(self.messages['shutdown'])
    
    def send_tts_request(self, text):
        """Send text to TTS node"""
        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)
        self.get_logger().info(f"TTS request sent: {text}")
    
    def publish_state(self, state):
        """Publish system state"""
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)
    
    def destroy_node(self):
        """Clean up resources"""
        self.is_alerting = False
        if self.alert_thread and self.alert_thread.is_alive():
            self.alert_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AlertSystemNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 