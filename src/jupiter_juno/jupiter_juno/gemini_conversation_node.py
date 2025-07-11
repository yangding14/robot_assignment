#!/usr/bin/env python3

import os
import yaml
import json
import requests
import time
import threading

import rospy
from std_msgs.msg import String

# Alternative: OpenAI API
try:
    import openai
    HAS_OPENAI = True
except ImportError:
    HAS_OPENAI = False


class GeminiConversationNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('gemini_conversation_node', anonymous=True)
        
        # Load configuration
        self.load_config()
        
        # Initialize AI model
        self.init_ai_model()
        
        # Conversation state
        self.conversation_active = False
        self.conversation_count = 0
        self.conversation_history = []
        
        # Publishers
        self.tts_publisher = rospy.Publisher(self.tts_topic, String, queue_size=10)
        self.state_publisher = rospy.Publisher(self.system_state_topic, String, queue_size=10)
        
        # Subscribers
        self.speech_subscriber = rospy.Subscriber(
            self.speech_result_topic,
            String,
            self.speech_callback,
            queue_size=10
        )
        
        self.state_subscriber = rospy.Subscriber(
            self.system_state_topic,
            String,
            self.state_callback,
            queue_size=10
        )
        
        rospy.loginfo(f"Gemini Conversation Node initialized with {self.model_name}")
    
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
            rospy.logerr(f"Failed to load config: {e}")
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
        
        if api_key:
            try:
                # Test Gemini API connection with a simple request
                test_response = self.test_gemini_connection(api_key)
                if test_response:
                    self.gemini_api_key = api_key
                    self.gemini_api_url = f"https://generativelanguage.googleapis.com/v1beta/models/{self.model_name}:generateContent"
                    self.ai_backend = 'gemini'
                    rospy.loginfo("Gemini API initialized successfully")
                    return
                else:
                    rospy.logerr("Failed to connect to Gemini API")
            except Exception as e:
                rospy.logerr(f"Failed to initialize Gemini: {e}")
        
        # Try OpenAI as fallback
        openai_key = os.environ.get('OPENAI_API_KEY')
        if HAS_OPENAI and openai_key:
            try:
                openai.api_key = openai_key
                self.ai_backend = 'openai'
                rospy.loginfo("OpenAI API initialized as fallback")
                return
            except Exception as e:
                rospy.logerr(f"Failed to initialize OpenAI: {e}")
        
        # No AI backend available
        self.ai_backend = None
        rospy.logwarn("No AI backend available - using simple responses")
    
    def test_gemini_connection(self, api_key):
        """Test connection to Gemini API"""
        try:
            url = f"https://generativelanguage.googleapis.com/v1beta/models/{self.model_name}:generateContent"
            headers = {
                'Content-Type': 'application/json',
            }
            params = {
                'key': api_key
            }
            data = {
                "contents": [{
                    "parts": [{"text": "Hello, respond with 'OK' if you can hear me."}]
                }]
            }
            
            response = requests.post(url, headers=headers, params=params, json=data, timeout=10)
            return response.status_code == 200
        except Exception as e:
            rospy.logerr(f"Gemini connection test failed: {e}")
            return False
    
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
        rospy.loginfo("Conversation started")
    
    def speech_callback(self, msg):
        """Handle speech recognition results"""
        if not self.conversation_active:
            return
        
        user_input = msg.data.strip()
        
        # Handle empty input
        if not user_input:
            rospy.logwarn("No speech detected, ending conversation")
            self.end_conversation()
            return
        
        rospy.loginfo(f"User said: '{user_input}'")
        
        # Generate AI response
        response = self.generate_response(user_input)
        
        if response:
            # Update conversation history
            self.conversation_history.append({
                'user': user_input,
                'assistant': response
            })
            
            # Increment conversation count
            self.conversation_count += 1
            
            # Send response to TTS
            self.send_tts(response)
            rospy.loginfo(f"Sent to TTS: '{response}'")
            
            # End conversation immediately after responding - no more rounds
            rospy.loginfo("Content delivered, ending conversation after TTS completes")
            
            def end_after_response():
                try:
                    time.sleep(4)  # Wait for TTS response to complete
                    if self.conversation_active:
                        rospy.loginfo("Ending conversation now")
                        self.end_conversation()
                except Exception as e:
                    rospy.logerr(f"Error ending conversation: {e}")
            
            end_thread = threading.Thread(target=end_after_response)
            end_thread.daemon = True
            end_thread.start()
            
        else:
            rospy.logerr("Failed to generate response")
            self.end_conversation()
    
    def generate_response(self, user_input):
        """Generate AI response based on user input"""
        try:
            # Check for keywords that need specific responses
            lower_input = user_input.lower()
            rospy.loginfo(f"Processing user input: '{user_input}' (round {self.conversation_count})")
            
            # Handle affirmative responses - directly give a joke
            if any(word in lower_input for word in ['yes', 'sure', 'okay', 'ok', 'please']):
                rospy.loginfo("User gave affirmative response, providing joke")
                prompt = f"{self.system_prompt}\n\nUser agreed to hear content. Please tell a short, clean, driving-related joke to help them stay alert."
            elif any(word in lower_input for word in ['joke', 'funny', 'laugh', 'humor']):
                rospy.loginfo("User requested a joke")
                prompt = f"{self.system_prompt}\n\nUser asked for a joke. Please tell a short, clean, driving-related joke."
            elif any(word in lower_input for word in ['tip', 'advice', 'safety', 'alert']):
                rospy.loginfo("User requested a safety tip")
                prompt = f"{self.system_prompt}\n\nUser asked for a driving tip. Please give a brief safety tip about staying alert while driving."
            elif any(word in lower_input for word in ['no', 'nothing', 'skip']):
                rospy.loginfo("User declined, ending conversation")
                return "No problem! Stay alert and drive safely. Remember, I'm here to help keep you safe on the road."
            else:
                # Default to joke for unclear responses
                rospy.loginfo("Unclear response, defaulting to joke")
                prompt = f"{self.system_prompt}\n\nUser gave an unclear response. Please tell a short, clean, driving-related joke to help them stay alert."
            
            # Generate response using AI backend
            rospy.loginfo(f"Using AI backend: {self.ai_backend}")
            if self.ai_backend == 'gemini':
                response = self.generate_gemini_response(prompt)
            elif self.ai_backend == 'openai':
                response = self.generate_openai_response(prompt, user_input)
            else:
                response = self.generate_simple_response(user_input)
            
            rospy.loginfo(f"Generated response: '{response}'")
            return response
            
        except Exception as e:
            rospy.logerr(f"Error generating response: {e}")
            return "Here's a quick joke to keep you alert: Why did the bicycle fall over? Because it was two-tired! Keep your eyes on the road!"
    
    def generate_gemini_response(self, prompt):
        """Generate response using Gemini API with direct HTTP calls"""
        try:
            headers = {
                'Content-Type': 'application/json',
            }
            params = {
                'key': self.gemini_api_key
            }
            
            # Build conversation context including history
            contents = []
            
            # Add conversation history
            for item in self.conversation_history[-2:]:  # Last 2 exchanges
                contents.append({
                    "parts": [{"text": f"User: {item['user']}"}],
                    "role": "user"
                })
                contents.append({
                    "parts": [{"text": item['assistant']}],
                    "role": "model"
                })
            
            # Add current prompt
            contents.append({
                "parts": [{"text": prompt}],
                "role": "user"
            })
            
            data = {
                "contents": contents,
                "generationConfig": {
                    "temperature": 0.7,
                    "topK": 40,
                    "topP": 0.95,
                    "maxOutputTokens": 100,
                }
            }
            
            response = requests.post(
                self.gemini_api_url, 
                headers=headers, 
                params=params, 
                json=data, 
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                if 'candidates' in result and len(result['candidates']) > 0:
                    candidate = result['candidates'][0]
                    if 'content' in candidate and 'parts' in candidate['content']:
                        return candidate['content']['parts'][0]['text']
                else:
                    rospy.logerr(f"Unexpected Gemini API response structure: {result}")
                    return None
            else:
                rospy.logerr(f"Gemini API error: {response.status_code} - {response.text}")
                return None
                
        except requests.exceptions.Timeout:
            rospy.logerr("Gemini API request timed out")
            return None
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Gemini API request error: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"Gemini API error: {e}")
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
            rospy.logerr(f"OpenAI API error: {e}")
            return None
    
    def generate_simple_response(self, user_input):
        """Generate simple response without AI"""
        lower_input = user_input.lower()
        
        jokes = [
            "What's a car's favorite meal? A traffic jam! But let's keep moving safely!",
        ]
        
        tips = [
            "Remember to take a 15-minute break every 2 hours of driving to stay fresh!",
            "Keep your eyes moving and scan the road ahead - it helps maintain alertness!",
            "Stay hydrated! Dehydration can cause fatigue while driving.",
            "If you feel drowsy, pull over safely and take a short nap. Better late than never!",
            "Roll down your windows or turn up the AC to get fresh air and stay alert!",
        ]
        
        # Handle affirmative responses - give a joke directly
        if any(word in lower_input for word in ['yes', 'sure', 'okay', 'ok', 'please']):
            import random
            return random.choice(jokes)
        
        # Handle specific requests
        elif any(word in lower_input for word in ['joke', 'funny', 'laugh', 'humor']):
            import random
            return random.choice(jokes)
        elif any(word in lower_input for word in ['tip', 'advice', 'safety', 'alert']):
            import random
            return random.choice(tips)
        elif any(word in lower_input for word in ['no', 'nothing', 'skip']):
            return "No problem! Stay alert and drive safely. I'm here to help keep you safe on the road."
        else:
            # Default to joke for any unclear response
            import random
            return random.choice(jokes)
    
    def send_tts(self, text):
        """Send text to TTS node"""
        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)
    
    def end_conversation(self):
        """End the conversation session"""
        self.conversation_active = False
        self.publish_state("conversation_ended")
        rospy.loginfo("Conversation ended")
    
    def publish_state(self, state):
        """Publish system state"""
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)


def main():
    try:
        node = GeminiConversationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")


if __name__ == '__main__':
    main() 