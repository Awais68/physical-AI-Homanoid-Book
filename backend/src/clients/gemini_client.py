"""Gemini API client for chat completion."""
from typing import Optional
import google.generativeai as genai
from src.config.settings import settings


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
    """Generate chat completion using Gemini with fallback.

    Args:
        messages: List of message dicts with role and content.
        system_prompt: Optional system prompt to prepend.
        max_tokens: Maximum tokens in response.
        temperature: Sampling temperature.

    Returns:
        Generated response text.
    """
    # Fallback response if API not available
    if not gemini_client:
        print("⚠ Gemini client not available - using basic response")
        last_message = messages[-1].get('content', '') if messages else ''
        
        # Extract context from system prompt if available
        if system_prompt and "CONTEXT FROM DOCUMENTATION:" in system_prompt:
            context_part = system_prompt.split("CONTEXT FROM DOCUMENTATION:")[1].strip()
            if context_part and context_part != "No relevant documentation found.":
                # We have context, provide informative response
                return f"""Based on the documentation available, here's what I found regarding "{last_message}":\n\n{context_part[:500]}\n\nNote: Full AI analysis is unavailable due to API limits, but the relevant documentation has been retrieved for you."""
        
        # Generic fallback
        return f"""I can search the Physical AI & Humanoid Robotics documentation for: "{last_message}"\n\nHowever, AI-powered analysis is currently unavailable. The system includes information about:\n- Physical AI systems and embodied intelligence\n- Humanoid robotics in education\n- Technical concepts and implementations\n- Pedagogical approaches\n- Safety and ethical considerations\n\nPlease try rephrasing your question or check back later."""
    
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
        
        # Return fallback for any API issues
        last_message = messages[-1].get('content', '') if messages else ''
        return f"""Based on the available documentation, here's information about: \"{last_message}\"

**Note:** AI-powered responses are temporarily unavailable due to API limits.

The system can still search through the Physical AI & Humanoid Robotics documentation database. For detailed answers, please ensure your API credentials are configured and have sufficient quota.

**Quick Info:**
- Physical AI combines artificial intelligence with physical robotic systems
- Includes sensors, actuators, and edge computing
- Used in educational robotics and STEM learning
- Features hands-on learning platforms

Would you like to know more about a specific topic?"""
