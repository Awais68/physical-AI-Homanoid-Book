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
    """Generate chat completion using Gemini.

    Args:
        messages: List of message dicts with role and content.
        system_prompt: Optional system prompt to prepend.
        max_tokens: Maximum tokens in response.
        temperature: Sampling temperature.

    Returns:
        Generated response text.

    Raises:
        RuntimeError: If Gemini client is not available or API fails.
    """
    if not gemini_client:
        raise RuntimeError("Gemini client not available")

    # Build conversation for Gemini
    conversation_parts = []

    if system_prompt:
        conversation_parts.append(f"System: {system_prompt}\n\n")

    for msg in messages:
        role = msg.get("role", "user")
        content = msg.get("content", "")
        if role == "user":
            conversation_parts.append(f"User: {content}")
        elif role == "assistant":
            conversation_parts.append(f"Assistant: {content}")

    prompt = "\n\n".join(conversation_parts)

    response = gemini_client.generate_content(
        prompt,
        generation_config=genai.GenerationConfig(
            max_output_tokens=max_tokens,
            temperature=temperature,
        ),
    )

    return response.text
