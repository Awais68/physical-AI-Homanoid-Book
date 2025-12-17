"""OpenAI API client for RAG functionality."""
from typing import Optional
from openai import OpenAI
from src.config.settings import settings


# Initialize OpenAI client (only if API key is provided)
openai_client = None
if settings.OPENAI_API_KEY:
    try:
        openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)
    except Exception as e:
        print(f"Warning: Could not initialize OpenAI client: {e}")


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
    """Generate chat completion using OpenAI.

    Args:
        messages: List of message dicts with role and content.
        system_prompt: Optional system prompt to prepend.
        max_tokens: Maximum tokens in response.
        temperature: Sampling temperature.

    Returns:
        Generated response text.
    """
    if not openai_client:
        raise ValueError("OpenAI client not initialized. Please check OPENAI_API_KEY in .env file")
    
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
