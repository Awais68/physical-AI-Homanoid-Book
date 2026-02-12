"""Gemini API client for chat completion with OpenAI fallback."""
from typing import Optional
import google.generativeai as genai
from src.config.settings import settings
from src.clients.openai_client import openai_client as openai_client_instance


# Initialize Gemini client
gemini_client = None
if settings.GEMINI_API_KEY:
    try:
        genai.configure(api_key=settings.GEMINI_API_KEY)
        gemini_client = genai.GenerativeModel(settings.GEMINI_CHAT_MODEL)
        print("✓ Gemini client initialized successfully")
    except Exception as e:
        print(f"Warning: Could not initialize Gemini client: {e}")


def chat_completion(
    messages: list[dict],
    system_prompt: Optional[str] = None,
    max_tokens: int = 1000,
    temperature: float = 0.7,
) -> str:
    """Generate chat completion using Gemini with OpenAI fallback.

    Args:
        messages: List of message dicts with role and content.
        system_prompt: Optional system prompt to prepend.
        max_tokens: Maximum tokens in response.
        temperature: Sampling temperature.

    Returns:
        Generated response text.
    """
    # Try Gemini first if available
    if gemini_client:
        try:
            # Build conversation for Gemini
            conversation_parts = []
            
            if system_prompt:
                conversation_parts.append(f"System: {system_prompt}\n\n")
            
            # Add message history
            for msg in messages:
                role = msg.get('role', 'user')
                content = msg.get('content', '')
                if role == 'user':
                    conversation_parts.append(f"User: {content}")
                elif role == 'assistant':
                    conversation_parts.append(f"Assistant: {content}")
            
            prompt = "\n\n".join(conversation_parts)
            
            # Generate response
            response = gemini_client.generate_content(
                prompt,
                generation_config=genai.GenerationConfig(
                    max_output_tokens=max_tokens,
                    temperature=temperature,
                )
            )
            
            return response.text
        
        except Exception as e:
            error_msg = str(e)
            print(f"⚠ Gemini API error: {error_msg}")
            # Fall through to OpenAI fallback
    
    # Try OpenAI as fallback
    if openai_client_instance and settings.OPENAI_API_KEY:
        try:
            from openai import OpenAI
            client = OpenAI(api_key=settings.OPENAI_API_KEY)
            
            full_messages = []
            if system_prompt:
                full_messages.append({"role": "system", "content": system_prompt})
            full_messages.extend(messages)
            
            response = client.chat.completions.create(
                model=settings.OPENAI_CHAT_MODEL,
                messages=full_messages,
                max_tokens=max_tokens,
                temperature=temperature,
            )
            return response.choices[0].message.content
        
        except Exception as e:
            print(f"⚠ OpenAI API error: {e}")
            # Fall through to basic fallback
    
    # Basic fallback response with comprehensive information
    print("⚠ All AI providers unavailable - using enhanced fallback response")
    last_message = messages[-1].get('content', '') if messages else ''
    
    # Enhanced fallback response with topic-specific information
    topic_lower = last_message.lower()
    
    # Check what topic the user is asking about
    if any(word in topic_lower for word in ['physical ai', 'embodied intelligence', 'embodied ai']):
        return """**🤖 Physical AI (Embodied Intelligence)**

Physical AI refers to artificial intelligence systems that combine machine learning with physical robotic platforms. Key characteristics include:

- **Sensory Integration**: AI systems that process real-world sensor data (cameras, lidar, touch sensors)
- **Real-time Decision Making**: Processing information in milliseconds for robot control
- **Embodied Learning**: Robots that learn through physical interaction with their environment
- **Human-Robot Interaction**: Natural communication between humans and robots

**Applications in Education:**
- Hands-on STEM learning experiences
- Demonstrating AI concepts through physical robots
- Teaching programming through robotics
- Understanding real-world AI constraints (power, processing, sensors)

Would you like more details on any specific aspect?"""

    elif any(word in topic_lower for word in ['humanoid', 'robot', 'robotics']):
        return """**🦾 Humanoid Robotics**

Humanoid robots are robots designed to resemble human body structure and behavior. They typically include:

- **Head**: Often with cameras for "vision" and speakers for "speech"
- **Torso**: Housing computing hardware and batteries
- **Arms**: For manipulation and gesturing
- **Legs/Wheels**: For mobility

**Key Technologies:**
- Inverse kinematics for movement planning
- Natural Language Processing for voice interaction
- Computer vision for object/face recognition
- Force feedback for safe human interaction

**Educational Uses:**
- Teaching programming and AI concepts
- Demonstrating human-robot collaboration
- Studying social robotics and ethics
- Hands-on engineering projects

What would you like to explore further?"""

    elif any(word in topic_lower for word in ['education', 'teaching', 'learning', 'k-12', 'stem']):
        return """**🎓 Physical AI in Education**

Integrating Physical AI and robotics into education offers transformative learning experiences:

**For Students:**
- **Engagement**: Robots capture attention and make abstract concepts tangible
- **Problem-Solving**: Real-world engineering challenges
- **Programming Skills**: Visual and text-based coding through robotics
- **Collaboration**: Teamwork on robot projects

**For Educators:**
- **Curriculum Integration**: Science, math, engineering, and arts
- **Assessment Tools**: Track student progress through robot performance
- **Accessibility**: Robots can adapt to different learning speeds

**Best Practices:**
- Start with simple concepts before complex robotics
- Encourage experimentation and "healthy failure"
- Connect robotics to real-world problems
- Include ethics discussions about AI and automation

Do you need help with specific educational scenarios?"""

    elif any(word in topic_lower for word in ['sensor', 'actuator', 'motor', 'hardware']):
        return """**🔧 Robotics Hardware Components**

Physical AI robots rely on essential hardware components:

**Sensors (Robot's "Senses"):**
- **Cameras**: Visual input for navigation and object recognition
- **Microphones**: Voice recognition and sound localization
- **LIDAR**: 3D mapping and distance measurement
- **Touch Sensors**: Force feedback and collision detection
- **IMU**: Accelerometers and gyroscopes for orientation

**Actuators (Robot's "Muscles"):**
- **Servo Motors**: Precise angular control
- **DC Motors**: Continuous rotation for wheels
- **Stepper Motors**: Position control without encoders
- **Pneumatic Systems**: Air-powered movement

**Processing Units:**
- **Microcontrollers**: Arduino, ESP32 for basic control
- **Single-Board Computers**: Raspberry Pi for complex AI
- **Edge AI Chips**: Specialized hardware for neural networks

What hardware aspect interests you most?"""

    elif any(word in topic_lower for word in ['edge', 'computing', 'processing']):
        return """**💻 Edge Computing in Robotics**

Edge computing brings AI processing directly to the robot, enabling:

**Benefits:**
- **Low Latency**: Real-time decisions without cloud round-trips
- **Privacy**: Data stays on the device
- **Reliability**: Works without internet connectivity
- **Efficiency**: Optimized for specific tasks

**Common Edge AI Hardware:**
- NVIDIA Jetson series
- Google Coral TPU
- Intel Movidius Myriad
- Raspberry Pi with AI accelerators

**Physical AI Edge Kit Features:**
- Real-time health monitoring
- ROS2 integration for robot control
- Local document storage and RAG
- Multi-language support

How can I help you explore edge computing further?"""

    else:
        return f"""**🤖 Humainoid Robotics Chatbot**

Thank you for your question about: *"{last_message}"*

I'm currently operating in **fallback mode** without full AI capabilities. However, I can help with topics including:

**Core Topics:**
- 🤖 Physical AI and Embodied Intelligence
- 🦾 Humanoid Robot Design and Control
- 🎓 Educational Applications (K-12 & Higher Ed)
- 🔧 Sensors, Actuators, and Robotics Hardware
- 💻 Edge Computing for Robots
- 📚 Documentation and Knowledge Management

**To Get Full AI Responses:**
Configure these environment variables:
- `GEMINI_API_KEY` - for Google Gemini
- `OPENAI_API_KEY` - for OpenAI (fallback)

**Quick Resources:**
- GitHub: github.com/Awais68/physical-AI-Homanoid-Book
- Documentation: awais68.github.io/physical-AI-Homanoid-Book/

What topic would you like to explore?"""
