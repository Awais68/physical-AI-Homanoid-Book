"""Unified chat completion provider - OpenRouter FIRST, Gemini fallback.

Default order:
  1. OpenRouter (settings.OPENROUTER_CHAT_MODEL) - DEFAULT (free, unlimited)
  2. Gemini (settings.GEMINI_CHAT_MODEL) - fallback
"""
from typing import Optional
from src.clients import openrouter_client as openrouter
from src.clients.gemini_client import chat_completion as gemini_chat_completion


def chat_completion(
    messages: list[dict],
    system_prompt: Optional[str] = None,
    max_tokens: int = 1000,
    temperature: float = 0.7,
) -> str:
    """Generate chat completion - OpenRouter first, Gemini fallback.

    Args:
        messages: List of message dicts with role and content.
        system_prompt: Optional system prompt to prepend.
        max_tokens: Maximum tokens in response.
        temperature: Sampling temperature.

    Returns:
        Generated response text.
    """
    # 1. OpenRouter (default provider - free tier, no daily limits)
    if openrouter.is_available():
        try:
            return openrouter.chat_completion(
                messages=messages,
                system_prompt=system_prompt,
                max_tokens=max_tokens,
                temperature=temperature,
            )
        except Exception as e:
            print(f"⚠ OpenRouter chat error: {e}, falling back to Gemini")

    # 2. Gemini fallback (has 20 req/day limit on free tier)
    try:
        return gemini_chat_completion(
            messages=messages,
            system_prompt=system_prompt,
            max_tokens=max_tokens,
            temperature=temperature,
        )
    except Exception as e:
        print(f"⚠ Gemini chat error: {e}")

    raise RuntimeError("All chat providers failed")
