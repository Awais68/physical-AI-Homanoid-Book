"""OpenRouter API client - DEFAULT provider for chat completion and embeddings.

Uses plain `requests` against OpenRouter's OpenAI-compatible REST API
(avoids openai-SDK/httpx version conflicts). Chat and embeddings both go
through OpenRouter first; Cohere/Gemini remain as fallbacks (handled by callers).
"""
from typing import Optional
import requests
from src.config.settings import settings


OPENROUTER_BASE_URL = "https://openrouter.ai/api/v1"

_HEADERS = {
    "Authorization": f"Bearer {settings.OPENROUTER_API_KEY}",
    "Content-Type": "application/json",
    "HTTP-Referer": "https://awais68.github.io/physical-AI-Homanoid-Book/",
    "X-Title": "Physical AI & Humanoid Robotics Book",
}

if settings.OPENROUTER_API_KEY:
    print("✓ OpenRouter client initialized (default provider)")
else:
    print("⚠️ OPENROUTER_API_KEY not set - OpenRouter disabled")


def is_available() -> bool:
    """Check if OpenRouter is configured."""
    return bool(settings.OPENROUTER_API_KEY)


def _post(path: str, payload: dict, timeout: float = 120.0) -> dict:
    """POST to OpenRouter API and return parsed JSON, raising on errors."""
    response = requests.post(
        f"{OPENROUTER_BASE_URL}{path}",
        headers=_HEADERS,
        json=payload,
        timeout=timeout,
    )
    response.raise_for_status()
    data = response.json()
    if "error" in data:
        raise RuntimeError(f"OpenRouter API error: {data['error']}")
    return data


def get_embedding(text: str) -> list[float]:
    """Generate embedding for text using OpenRouter embeddings API.

    Args:
        text: Text to generate embedding for.

    Returns:
        List of floats representing the embedding vector.
    """
    if not is_available():
        raise ValueError("OpenRouter client not initialized")

    data = _post("/embeddings", {
        "model": settings.OPENROUTER_EMBEDDING_MODEL,
        "input": text,
    })
    return data["data"][0]["embedding"]


def get_embeddings_batch(texts: list[str]) -> list[list[float]]:
    """Generate embeddings for multiple texts in one request (max ~96 inputs).

    Args:
        texts: List of texts to embed.

    Returns:
        List of embedding vectors (input order preserved).
    """
    if not is_available():
        raise ValueError("OpenRouter client not initialized")

    data = _post("/embeddings", {
        "model": settings.OPENROUTER_EMBEDDING_MODEL,
        "input": texts,
    })
    sorted_data = sorted(data["data"], key=lambda d: d["index"])
    return [d["embedding"] for d in sorted_data]


def chat_completion(
    messages: list[dict],
    system_prompt: Optional[str] = None,
    max_tokens: int = 1000,
    temperature: float = 0.7,
) -> str:
    """Generate chat completion using OpenRouter.

    Args:
        messages: List of message dicts with role and content.
        system_prompt: Optional system prompt to prepend.
        max_tokens: Maximum tokens in response.
        temperature: Sampling temperature.

    Returns:
        Generated response text.

    Raises:
        ValueError: If client is not initialized.
        Exception: On API errors (caller should fall back).
    """
    if not is_available():
        raise ValueError("OpenRouter client not initialized")

    full_messages = []
    if system_prompt:
        full_messages.append({"role": "system", "content": system_prompt})
    full_messages.extend(messages)

    data = _post("/chat/completions", {
        "model": settings.OPENROUTER_CHAT_MODEL,
        "messages": full_messages,
        "max_tokens": max_tokens,
        "temperature": temperature,
    })
    content = data["choices"][0]["message"]["content"]
    if not content or not content.strip():
        raise RuntimeError("OpenRouter returned empty response")
    return content.strip()
