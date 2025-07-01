#!/usr/bin/env python3

import os
import yaml
import queue
import threading
import tempfile
import time

import rospy
from std_msgs.msg import String

# TTS libraries
try:
    from gtts import gTTS
    import pygame
    pygame.mixer.init()
    HAS_GTTS = True
except ImportError:
    HAS_GTTS = False
    print("Warning: gtts and pygame not available")

try:
    import pydub
    from pydub import AudioSegment
    from pydub.playback import play
    HAS_PYDUB = True
except ImportError:
    HAS_PYDUB = False


class TTSNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('tts_node', anonymous=True)
        
        # Load configuration
        self.load_config()
        
        # Initialize TTS engine
        self.init_tts_engine()
        
        # Queue for TTS requests
        self.tts_queue = queue.Queue()
        self.is_speaking = False
        
        # Subscriber
        self.tts_subscriber = rospy.Subscriber(
            self.tts_topic,
            String,
            self.tts_callback,
            queue_size=10
        )
        
        # Publisher for TTS status
        self.tts_status_publisher = rospy.Publisher(
            self.tts_status_topic,
            String,
            queue_size=10
        )
        
        # Start TTS processing thread
        self.running = True
        self.tts_thread = threading.Thread(target=self.process_tts_queue)
        self.tts_thread.start()
        
        rospy.loginfo(f"TTS Node initialized with {self.engine} engine")
    
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
            self.language = tts_config.get('language', 'en')
            self.slow_speech = tts_config.get('slow_speech', False)
            
            # ROS topics
            topics = config['ros_topics']
            self.tts_topic = topics['tts_request']
            self.tts_status_topic = topics['tts_status']
            
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            # Use defaults
            self.engine = 'gtts'
            self.language = 'en'
            self.slow_speech = False
            self.tts_topic = '/jupiter_juno/tts_request'
            self.tts_status_topic = '/jupiter_juno/tts_status'
    
    def init_tts_engine(self):
        """Initialize the TTS engine based on configuration"""
        if self.engine == 'gtts' and HAS_GTTS:
            self.init_gtts()
        else:
            # Check what's available and use fallback
            if HAS_GTTS:
                self.engine = 'gtts'
                self.init_gtts()
            else:
                rospy.logerr("No TTS engine available!")
                self.engine = None
    
    def init_gtts(self):
        """Initialize gTTS engine"""
        # gTTS doesn't need initialization, just set parameters
        self.gtts_lang = self.language
        self.gtts_slow = self.slow_speech
        rospy.loginfo("gTTS engine initialized")
    
    def tts_callback(self, msg):
        """Handle incoming TTS requests"""
        text = msg.data
        if text:
            self.tts_queue.put(text)
            rospy.loginfo(f"TTS request queued: {text}")
    
    def process_tts_queue(self):
        """Process TTS requests from queue"""
        while self.running and not rospy.is_shutdown():
            try:
                # Get text from queue with timeout
                text = self.tts_queue.get(timeout=0.5)
                
                # Signal TTS start
                self.publish_tts_status("speaking")
                self.is_speaking = True
                
                # Speak the text
                self.speak(text)
                
                # Signal TTS completion
                self.is_speaking = False
                self.publish_tts_status("finished")
                self.tts_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error processing TTS: {e}")
                self.is_speaking = False
                self.publish_tts_status("error")
    
    def publish_tts_status(self, status):
        """Publish TTS status for coordination with other nodes"""
        msg = String()
        msg.data = status
        self.tts_status_publisher.publish(msg)
        rospy.loginfo(f"TTS status: {status}")
    
    def speak(self, text):
        """Speak the given text using the configured engine"""
        if not self.engine:
            rospy.logwarn(f"No TTS engine available to speak: {text}")
            return
        
        try:
            if self.engine == 'gtts' and HAS_GTTS:
                self.speak_gtts(text)
                
        except Exception as e:
            rospy.logerr(f"Error speaking text: {e}")
    
    def speak_gtts(self, text):
        """Speak using gTTS"""
        try:
            # Generate speech
            tts = gTTS(text=text, lang=self.gtts_lang, slow=self.gtts_slow)
            
            # Save to temporary file
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as tmp_file:
                tmp_filename = tmp_file.name
                tts.save(tmp_filename)
            
            # Play the audio file using pygame
            if HAS_GTTS:  # pygame is included with gtts check
                pygame.mixer.music.load(tmp_filename)
                pygame.mixer.music.play()
                
                # Wait for playback to complete
                while pygame.mixer.music.get_busy():
                    pygame.time.Clock().tick(10)
            elif HAS_PYDUB:
                # Alternative playback with pydub
                audio = AudioSegment.from_mp3(tmp_filename)
                play(audio)
            else:
                rospy.logwarn("No audio playback library available")
            
            # Clean up temporary file
            try:
                os.unlink(tmp_filename)
            except OSError:
                pass  # File might already be deleted
            
        except Exception as e:
            rospy.logerr(f"gTTS speak error: {e}")
    
    def shutdown(self):
        """Clean up resources"""
        self.running = False
        
        # Wait for TTS thread to finish
        if hasattr(self, 'tts_thread'):
            self.tts_thread.join(timeout=2.0)


def main():
    try:
        node = TTSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()


if __name__ == '__main__':
    main() 