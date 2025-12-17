"""Cohere API client for embeddings."""
import cohere
from src.config.settings import settings


# Initialize Cohere client
cohere_client = cohere.Client(settings.COHERE_API_KEY)


def get_embedding(text: str) -> list[float]:
    """Generate embedding for text using Cohere.

    Args:
        text: Text to generate embedding for.

    Returns:
        List of floats representing the embedding vector (1024 dimensions).
    """
    response = cohere_client.embed(
        model=settings.COHERE_EMBEDDING_MODEL,
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]


def get_document_embedding(text: str) -> list[float]:
    """Generate embedding for document text using Cohere.

    Args:
        text: Document text to generate embedding for.

    Returns:
        List of floats representing the embedding vector (1024 dimensions).
    """
    response = cohere_client.embed(
        model=settings.COHERE_EMBEDDING_MODEL,
        input_type="search_document",  # Use search_document for indexing
        texts=[text],
    )
    return response.embeddings[0]
