"""OpenAI API client for RAG functionality."""
from typing import Optional
from openai import OpenAI
from src.config.settings import settings


# Initialize OpenAI client
openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)


async def get_embedding(text: str) -> list[float]:
    """Generate embedding for text using OpenAI.

    Args:
        text: Text to generate embedding for.

    Returns:
        List of floats representing the embedding vector.
    """
    response = openai_client.embeddings.create(
        model=settings.OPENAI_EMBEDDING_MODEL,
        input=text,
    )
    return response.data[0].embedding


async def chat_completion(
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
