"""OpenAI API client for RAG functionality."""
from typing import Optional
from openai import OpenAI
from src.config.settings import settings


# Initialize OpenAI client (only if API key is provided)
openai_client = None
if settings.OPENAI_API_KEY:
    try:
        openai_client = OpenAI(
            api_key=settings.OPENAI_API_KEY,
            timeout=30.0,
            # Remove proxies parameter as it's not supported in newer versions
        )
        print("✓ OpenAI client initialized")
    except Exception as e:
        print(f"⚠️ Could not initialize OpenAI client: {e}")
        openai_client = None


def get_embedding(text: str) -> list[float]:
    """Generate embedding for text using OpenAI.

    Args:
        text: Text to generate embedding for.

    Returns:
        List of floats representing the embedding vector.
    """
    if not openai_client:
        raise ValueError("OpenAI client not initialized")
    
    response = openai_client.embeddings.create(
        model=settings.OPENAI_EMBEDDING_MODEL,
        input=text,
    )
    return response.data[0].embedding


def chat_completion(
    messages: list[dict],
    system_prompt: Optional[str] = None,
    max_tokens: int = 1000,
    temperature: float = 0.7,
) -> str:
    """Generate chat completion using OpenAI with fallback.

    Args:
        messages: List of message dicts with role and content.
        system_prompt: Optional system prompt to prepend.
        max_tokens: Maximum tokens in response.
        temperature: Sampling temperature.

    Returns:
        Generated response text.
    """
    # Fallback response if API not available
    if not openai_client:
        print("⚠ OpenAI client not available - using fallback response")
        last_message = messages[-1].get('content', '') if messages else ''
        return f"""Thank you for your question about: "{last_message}"

I'm currently operating in offline mode. To get AI-powered responses, please:

1. Add your OpenAI API key to the `.env` file:
   ```
   OPENAI_API_KEY=your_key_here
   ```

2. Or use the Gemini API as an alternative:
   ```
   GEMINI_API_KEY=your_key_here
   ```

For now, I can search through the documentation database. The system has access to Physical AI and Humanoid Robotics educational materials."""
    
    try:
        full_messages = []

        if system_prompt:
            full_messages.append({"role": "system", "content": system_prompt})

        full_messages.extend(messages)

        response = openai_client.chat.completions.create(
            model=settings.OPENAI_CHAT_MODEL,
            messages=full_messages,
            max_tokens=max_tokens,
            temperature=temperature,
        )
        return response.choices[0].message.content
    
    except Exception as e:
        error_msg = str(e)
        print(f"⚠ OpenAI API error: {error_msg}")
        
        # Return fallback for billing/quota issues
        if "429" in error_msg or "billing" in error_msg.lower() or "quota" in error_msg.lower():
            last_message = messages[-1].get('content', '') if messages else ''
            return f"""Based on the available documentation, here's information about: "{last_message}"

# **Note:** AI-powered responses are temporarily unavailable due to API limits. 

# The system can still search through the Physical AI & Humanoid Robotics documentation database. For detailed answers, please ensure your API credentials are configured and have sufficient quota.

# **Quick Info:**
# - Physical AI combines artificial intelligence with physical robotic systems
# - Includes sensors, actuators, and edge computing
# - Used in educational robotics and STEM learning
# - Features hands-on learning platforms

# Would you like to know more about a specific topic?"""
        
        # Re-raise other errors
        raise
