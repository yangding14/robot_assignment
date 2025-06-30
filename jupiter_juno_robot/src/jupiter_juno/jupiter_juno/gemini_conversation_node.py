#!/usr/bin/env python3

import os
import yaml
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import google.generativeai as genai
    HAS_GEMINI = True
except ImportError:
    HAS_GEMINI = False
    print("Warning: google-generativeai not available")

# Alternative: OpenAI API
try:
    import openai
    HAS_OPENAI = True
except ImportError:
    HAS_OPENAI = False


class GeminiConversationNode(Node):
    def __init__(self):
        super().__init__('gemini_conversation_node')
        
        # Load configuration
        self.load_config()
        
        # Initialize AI model
        self.init_ai_model()
        
        # Conversation state
        self.conversation_active = False
        self.conversation_count = 0
        self.conversation_history = []
        
        # Publishers
        self.tts_publisher = self.create_publisher(String, self.tts_topic, 10)
        self.state_publisher = self.create_publisher(String, self.system_state_topic, 10)
        
        # Subscribers
        self.speech_subscriber = self.create_subscription(
            String,
            self.speech_result_topic,
            self.speech_callback,
            10
        )
        
        self.state_subscriber = self.create_subscription(
            String,
            self.system_state_topic,
            self.state_callback,
            10
        )
        
        self.get_logger().info(f"Gemini Conversation Node initialized with {self.model_name}")
    
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
            
            # Gemini parameters
            gemini_config = config['gemini']
            self.api_key_env = gemini_config['api_key_env']
            self.model_name = gemini_config['model']
            self.max_rounds = gemini_config['max_rounds']
            self.system_prompt = gemini_config['system_prompt']
            
            # ROS topics
            topics = config['ros_topics']
            self.tts_topic = topics['tts_request']
            self.speech_result_topic = topics['speech_result']
            self.system_state_topic = topics['system_state']
            
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            # Use defaults
            self.api_key_env = 'GEMINI_API_KEY'
            self.model_name = 'gemini-pro'
            self.max_rounds = 3
            self.system_prompt = "You are a helpful driving assistant."
            self.tts_topic = '/jupiter_juno/tts_request'
            self.speech_result_topic = '/jupiter_juno/speech_result'
            self.system_state_topic = '/jupiter_juno/system_state'
    
    def init_ai_model(self):
        """Initialize the AI model (Gemini or alternative)"""
        api_key = os.environ.get(self.api_key_env)
        
        if HAS_GEMINI and api_key:
            try:
                genai.configure(api_key=api_key)
                self.model = genai.GenerativeModel(self.model_name)
                self.chat = self.model.start_chat(history=[])
                self.ai_backend = 'gemini'
                self.get_logger().info("Gemini API initialized successfully")
                return
            except Exception as e:
                self.get_logger().error(f"Failed to initialize Gemini: {e}")
        
        # Try OpenAI as fallback
        openai_key = os.environ.get('OPENAI_API_KEY')
        if HAS_OPENAI and openai_key:
            try:
                openai.api_key = openai_key
                self.ai_backend = 'openai'
                self.get_logger().info("OpenAI API initialized as fallback")
                return
            except Exception as e:
                self.get_logger().error(f"Failed to initialize OpenAI: {e}")
        
        # No AI backend available
        self.ai_backend = None
        self.get_logger().warn("No AI backend available - using simple responses")
    
    def state_callback(self, msg):
        """Handle system state changes"""
        state = msg.data
        
        if state == "conversation_ready":
            self.start_conversation()
    
    def start_conversation(self):
        """Start a new conversation session"""
        self.conversation_active = True
        self.conversation_count = 0
        self.conversation_history = []
        self.publish_state("conversation_started")
        self.get_logger().info("Conversation started")
    
    def speech_callback(self, msg):
        """Handle speech recognition results"""
        if not self.conversation_active:
            return
        
        user_input = msg.data.strip()
        
        # Handle empty input
        if not user_input:
            self.get_logger().warn("No speech detected, ending conversation")
            self.end_conversation()
            return
        
        self.get_logger().info(f"User said: {user_input}")
        
        # Generate AI response
        response = self.generate_response(user_input)
        
        if response:
            # Send response to TTS
            self.send_tts(response)
            
            # Update conversation history
            self.conversation_history.append({
                'user': user_input,
                'assistant': response
            })
            
            # Increment conversation count
            self.conversation_count += 1
            
            # Check if max rounds reached
            if self.conversation_count >= self.max_rounds:
                self.get_logger().info("Max conversation rounds reached")
                self.end_conversation()
        else:
            self.get_logger().error("Failed to generate response")
            self.end_conversation()
    
    def generate_response(self, user_input):
        """Generate AI response based on user input"""
        try:
            # Check for keywords that need specific responses
            lower_input = user_input.lower()
            
            if any(word in lower_input for word in ['joke', 'funny', 'laugh']):
                prompt = f"{self.system_prompt}\n\nUser asked for a joke. Please tell a short, clean, driving-related joke."
            elif any(word in lower_input for word in ['tip', 'advice', 'safety']):
                prompt = f"{self.system_prompt}\n\nUser asked for a driving tip. Please give a brief safety tip about staying alert while driving."
            else:
                prompt = f"{self.system_prompt}\n\nUser said: {user_input}\n\nPlease respond briefly and helpfully."
            
            if self.ai_backend == 'gemini':
                response = self.generate_gemini_response(prompt)
            elif self.ai_backend == 'openai':
                response = self.generate_openai_response(prompt, user_input)
            else:
                response = self.generate_simple_response(user_input)
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error generating response: {e}")
            return "I'm having trouble understanding. Let me tell you a quick tip: Take regular breaks during long drives to stay alert!"
    
    def generate_gemini_response(self, prompt):
        """Generate response using Gemini API"""
        try:
            response = self.chat.send_message(prompt)
            return response.text
        except Exception as e:
            self.get_logger().error(f"Gemini API error: {e}")
            return None
    
    def generate_openai_response(self, prompt, user_input):
        """Generate response using OpenAI API"""
        try:
            messages = [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_input}
            ]
            
            # Add conversation history
            for item in self.conversation_history[-2:]:  # Last 2 exchanges
                messages.append({"role": "user", "content": item['user']})
                messages.append({"role": "assistant", "content": item['assistant']})
            
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=messages,
                max_tokens=100,
                temperature=0.7
            )
            
            return response.choices[0].message.content
            
        except Exception as e:
            self.get_logger().error(f"OpenAI API error: {e}")
            return None
    
    def generate_simple_response(self, user_input):
        """Generate simple response without AI"""
        lower_input = user_input.lower()
        
        jokes = [
            "Why did the bicycle fall over? Because it was two-tired!",
            "What do you call a car that never stops? Exhausted!",
            "Why don't cars ever get lonely? Because they come with a lot of company!",
        ]
        
        tips = [
            "Remember to take a 15-minute break every 2 hours of driving to stay fresh!",
            "Keep your eyes moving and scan the road ahead - it helps maintain alertness!",
            "Stay hydrated! Dehydration can cause fatigue while driving.",
        ]
        
        if any(word in lower_input for word in ['joke', 'funny', 'laugh']):
            import random
            return random.choice(jokes)
        elif any(word in lower_input for word in ['tip', 'advice', 'safety']):
            import random
            return random.choice(tips)
        else:
            return "Stay alert and drive safely! Would you like a joke or a safety tip?"
    
    def send_tts(self, text):
        """Send text to TTS node"""
        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)
    
    def end_conversation(self):
        """End the conversation session"""
        self.conversation_active = False
        self.publish_state("conversation_ended")
        self.get_logger().info("Conversation ended")
    
    def publish_state(self, state):
        """Publish system state"""
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)
    
    def destroy_node(self):
        """Clean up resources"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GeminiConversationNode()
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