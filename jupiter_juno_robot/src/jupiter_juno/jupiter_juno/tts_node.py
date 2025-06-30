#!/usr/bin/env python3

import os
import yaml
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# TTS libraries
try:
    import pyttsx3
    HAS_PYTTSX3 = True
except ImportError:
    HAS_PYTTSX3 = False
    print("Warning: pyttsx3 not available")

try:
    from gtts import gTTS
    import pygame
    pygame.mixer.init()
    HAS_GTTS = True
except ImportError:
    HAS_GTTS = False
    print("Warning: gtts not available")


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        
        # Load configuration
        self.load_config()
        
        # Initialize TTS engine
        self.init_tts_engine()
        
        # Queue for TTS requests
        self.tts_queue = queue.Queue()
        self.is_speaking = False
        
        # Subscriber
        self.tts_subscriber = self.create_subscription(
            String,
            self.tts_topic,
            self.tts_callback,
            10
        )
        
        # Start TTS processing thread
        self.running = True
        self.tts_thread = threading.Thread(target=self.process_tts_queue)
        self.tts_thread.start()
        
        self.get_logger().info(f"TTS Node initialized with {self.engine} engine")
    
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
            
            # TTS parameters
            tts_config = config['tts']
            self.engine = tts_config['engine']
            self.rate = tts_config['rate']
            self.voice_gender = tts_config['voice_gender']
            
            # ROS topics
            topics = config['ros_topics']
            self.tts_topic = topics['tts_request']
            
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            # Use defaults
            self.engine = 'pyttsx3'
            self.rate = 150
            self.voice_gender = 1
            self.tts_topic = '/jupiter_juno/tts_request'
    
    def init_tts_engine(self):
        """Initialize the TTS engine based on configuration"""
        if self.engine == 'pyttsx3' and HAS_PYTTSX3:
            self.init_pyttsx3()
        elif self.engine == 'gtts' and HAS_GTTS:
            self.init_gtts()
        else:
            # Fallback selection
            if HAS_PYTTSX3:
                self.engine = 'pyttsx3'
                self.init_pyttsx3()
            elif HAS_GTTS:
                self.engine = 'gtts'
                self.init_gtts()
            else:
                self.get_logger().error("No TTS engine available!")
                self.engine = None
    
    def init_pyttsx3(self):
        """Initialize pyttsx3 engine"""
        try:
            self.tts_engine = pyttsx3.init()
            
            # Set properties
            self.tts_engine.setProperty('rate', self.rate)
            
            # Set voice (try to select by gender)
            voices = self.tts_engine.getProperty('voices')
            if voices:
                # Try to find a voice matching the gender preference
                for voice in voices:
                    if self.voice_gender == 1 and 'female' in voice.name.lower():
                        self.tts_engine.setProperty('voice', voice.id)
                        break
                    elif self.voice_gender == 0 and 'male' in voice.name.lower():
                        self.tts_engine.setProperty('voice', voice.id)
                        break
                else:
                    # Use first available voice if no match
                    self.tts_engine.setProperty('voice', voices[0].id)
            
            self.get_logger().info("pyttsx3 TTS engine initialized")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize pyttsx3: {e}")
            self.engine = None
    
    def init_gtts(self):
        """Initialize gTTS engine"""
        # gTTS doesn't need initialization, just set parameters
        self.gtts_lang = 'en'
        self.gtts_slow = False
        self.get_logger().info("gTTS engine initialized")
    
    def tts_callback(self, msg):
        """Handle incoming TTS requests"""
        text = msg.data
        if text:
            self.tts_queue.put(text)
            self.get_logger().info(f"TTS request queued: {text}")
    
    def process_tts_queue(self):
        """Process TTS requests from queue"""
        while self.running:
            try:
                # Get text from queue with timeout
                text = self.tts_queue.get(timeout=0.5)
                self.is_speaking = True
                
                # Speak the text
                self.speak(text)
                
                self.is_speaking = False
                self.tts_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error processing TTS: {e}")
                self.is_speaking = False
    
    def speak(self, text):
        """Speak the given text using the configured engine"""
        if not self.engine:
            self.get_logger().warn(f"No TTS engine available to speak: {text}")
            return
        
        try:
            if self.engine == 'pyttsx3' and HAS_PYTTSX3:
                self.speak_pyttsx3(text)
            elif self.engine == 'gtts' and HAS_GTTS:
                self.speak_gtts(text)
                
        except Exception as e:
            self.get_logger().error(f"Error speaking text: {e}")
    
    def speak_pyttsx3(self, text):
        """Speak using pyttsx3"""
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"pyttsx3 speak error: {e}")
            # Reinitialize engine in case of error
            self.init_pyttsx3()
    
    def speak_gtts(self, text):
        """Speak using gTTS"""
        try:
            import tempfile
            
            # Generate speech
            tts = gTTS(text=text, lang=self.gtts_lang, slow=self.gtts_slow)
            
            # Save to temporary file
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as tmp_file:
                tmp_filename = tmp_file.name
                tts.save(tmp_filename)
            
            # Play the audio file
            pygame.mixer.music.load(tmp_filename)
            pygame.mixer.music.play()
            
            # Wait for playback to complete
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
            
            # Clean up
            os.unlink(tmp_filename)
            
        except Exception as e:
            self.get_logger().error(f"gTTS speak error: {e}")
    
    def destroy_node(self):
        """Clean up resources"""
        self.running = False
        
        # Wait for TTS thread to finish
        if hasattr(self, 'tts_thread'):
            self.tts_thread.join(timeout=2.0)
        
        # Clean up TTS engine
        if self.engine == 'pyttsx3' and hasattr(self, 'tts_engine'):
            try:
                self.tts_engine.stop()
            except:
                pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TTSNode()
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