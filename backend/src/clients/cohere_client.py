"""Cohere API client for embeddings."""
import cohere
import hashlib
from src.config.settings import settings


# Initialize Cohere client
cohere_client = cohere.Client(settings.COHERE_API_KEY) if settings.COHERE_API_KEY else None


def simple_embedding(text: str, dim: int = 1024) -> list[float]:
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
    """Generate embedding for text using Cohere or fallback.

    Args:
        text: Text to generate embedding for.

    Returns:
        List of floats representing the embedding vector (1024 dimensions).
    """
    try:
        if cohere_client:
            response = cohere_client.embed(
                model=settings.COHERE_EMBEDDING_MODEL,
                input_type="search_query",  # Use search_query for queries
                texts=[text],
            )
            return response.embeddings[0]
        else:
            # Fallback to simple embedding
            return simple_embedding(text)
    except Exception as e:
        print(f"Cohere API error: {e}, using fallback embedding")
        return simple_embedding(text)


def get_document_embedding(text: str) -> list[float]:
    """Generate embedding for document text using Cohere or fallback.

    Args:
        text: Document text to generate embedding for.

    Returns:
        List of floats representing the embedding vector (1024 dimensions).
    """
    try:
        if cohere_client:
            response = cohere_client.embed(
                model=settings.COHERE_EMBEDDING_MODEL,
                input_type="search_document",  # Use search_document for indexing
                texts=[text],
            )
            return response.embeddings[0]
        else:
            return simple_embedding(text)
    except Exception as e:
        print(f"Cohere API error: {e}, using fallback embedding")
        return simple_embedding(text)
