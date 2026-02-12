"""Gemini API client for embeddings."""
import google.generativeai as genai
import hashlib
from src.config.settings import settings


# Initialize Gemini for embeddings
if settings.GEMINI_API_KEY:
    genai.configure(api_key=settings.GEMINI_API_KEY)
    GEMINI_EMBEDDING_MODEL = "models/gemini-embedding-001"
    EMBEDDING_DIMENSION = 3072
else:
    GEMINI_EMBEDDING_MODEL = None
    EMBEDDING_DIMENSION = 768


def simple_embedding(text: str, dim: int = 768) -> list[float]:
    """Generate simple hash-based embedding as fallback.
    
    Args:
        text: Text to generate embedding for.
        dim: Dimension of the embedding vector.
        
    Returns:
        List of floats representing a simple embedding vector.
    """
    # Create deterministic hash-based vector
    hash_obj = hashlib.sha256(text.encode())
    hash_bytes = hash_obj.digest()
    vector = []
    for i in range(dim):
        # Normalize to [-1, 1] range
        vector.append((hash_bytes[i % len(hash_bytes)] - 128) / 128.0)
    return vector


def get_embedding(text: str) -> list[float]:
    """Generate embedding for text using Gemini or fallback.

    Args:
        text: Text to generate embedding for.

    Returns:
        List of floats representing the embedding vector (768 dimensions).
    """
    try:
        if GEMINI_EMBEDDING_MODEL:
            result = genai.embed_content(
                model=GEMINI_EMBEDDING_MODEL,
                content=text,
                task_type="retrieval_query"
            )
            return result['embedding']
        else:
            return simple_embedding(text)
    except Exception as e:
        print(f"Gemini Embedding API error: {e}, using fallback embedding")
        return simple_embedding(text)


def get_document_embedding(text: str) -> list[float]:
    """Generate embedding for document text using Gemini or fallback.

    Args:
        text: Document text to generate embedding for.

    Returns:
        List of floats representing the embedding vector (768 dimensions).
    """
    try:
        if GEMINI_EMBEDDING_MODEL:
            result = genai.embed_content(
                model=GEMINI_EMBEDDING_MODEL,
                content=text,
                task_type="retrieval_document"
            )
            return result['embedding']
        else:
            return simple_embedding(text)
    except Exception as e:
        print(f"Gemini Embedding API error: {e}, using fallback embedding")
        return simple_embedding(text)
