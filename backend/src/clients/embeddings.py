"""Unified embeddings provider - OpenRouter FIRST, Cohere fallback.

Default order:
  1. OpenRouter (settings.OPENROUTER_EMBEDDING_MODEL, e.g. openai/text-embedding-3-large, 3072 dims)
  2. Cohere (embed-english-v3.0, 1024 dims) - legacy fallback
  3. Simple hash embedding - offline fallback (matches EMBEDDING_DIM)
"""
import hashlib
from src.config.settings import settings
from src.clients import openrouter_client as openrouter


def simple_embedding(text: str, dim: int = None) -> list[float]:
    """Deterministic hash-based embedding as last-resort fallback."""
    dim = dim or settings.EMBEDDING_DIM
    hash_obj = hashlib.sha256(text.encode())
    hash_bytes = hash_obj.digest()
    vector = []
    for i in range(dim):
        vector.append((hash_bytes[i % len(hash_bytes)] - 128) / 128.0)
    return vector


def _cohere_embedding(text: str, input_type: str) -> list[float]:
    """Cohere fallback embedding."""
    import cohere
    client = cohere.Client(settings.COHERE_API_KEY)
    response = client.embed(
        model=settings.COHERE_EMBEDDING_MODEL,
        input_type=input_type,
        texts=[text],
    )
    return response.embeddings[0]


def get_embedding(text: str) -> list[float]:
    """Generate query embedding - OpenRouter first, Cohere fallback."""
    # 1. OpenRouter (default)
    if openrouter.is_available():
        try:
            return openrouter.get_embedding(text)
        except Exception as e:
            print(f"⚠ OpenRouter embedding error: {e}, trying Cohere fallback")

    # 2. Cohere fallback
    if settings.COHERE_API_KEY:
        try:
            return _cohere_embedding(text, "search_query")
        except Exception as e:
            print(f"⚠ Cohere embedding error: {e}, using simple fallback")

    # 3. Offline fallback
    return simple_embedding(text)


def get_document_embedding(text: str) -> list[float]:
    """Generate document embedding - OpenRouter first, Cohere fallback."""
    if openrouter.is_available():
        try:
            return openrouter.get_embedding(text)
        except Exception as e:
            print(f"⚠ OpenRouter embedding error: {e}, trying Cohere fallback")

    if settings.COHERE_API_KEY:
        try:
            return _cohere_embedding(text, "search_document")
        except Exception as e:
            print(f"⚠ Cohere embedding error: {e}, using simple fallback")

    return simple_embedding(text)
