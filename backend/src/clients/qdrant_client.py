"""Qdrant vector database client."""
from typing import Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from src.config.settings import settings


def get_qdrant_client() -> QdrantClient:
    """Get Qdrant client instance.

    Returns:
        Configured QdrantClient instance.
    """
    if settings.QDRANT_API_KEY:
        return QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
        )
    return QdrantClient(url=settings.QDRANT_URL)


# Global client instance
qdrant_client = get_qdrant_client()


def ensure_collection_exists(
    collection_name: str,
    vector_size: int = 1536,
) -> bool:
    """Ensure a collection exists in Qdrant.

    Args:
        collection_name: Name of the collection.
        vector_size: Size of the embedding vectors.

    Returns:
        True if collection exists or was created.
    """
    try:
        collections = qdrant_client.get_collections()
        collection_names = [c.name for c in collections.collections]

        if collection_name not in collection_names:
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE,
                ),
            )
        return True
    except Exception as e:
        print(f"Error ensuring collection exists: {e}")
        return False


def search_similar(
    collection_name: str,
    query_vector: list[float],
    top_k: int = 5,
    score_threshold: float = 0.4,
) -> list[dict]:
    """Search for similar documents in Qdrant.

    Args:
        collection_name: Name of the collection to search.
        query_vector: Query embedding vector.
        top_k: Number of results to return.
        score_threshold: Minimum similarity score.

    Returns:
        List of search results with payload and scores.
    """
    results = qdrant_client.search(
        collection_name=collection_name,
        query_vector=query_vector,
        limit=top_k,
        score_threshold=score_threshold,
    )

    return [
        {
            "id": str(result.id),
            "score": result.score,
            "payload": result.payload,
        }
        for result in results
    ]


def upsert_documents(
    collection_name: str,
    documents: list[dict],
) -> bool:
    """Upsert documents into Qdrant collection.

    Args:
        collection_name: Name of the collection.
        documents: List of documents with id, vector, and payload.

    Returns:
        True if successful.
    """
    try:
        points = [
            models.PointStruct(
                id=doc["id"],
                vector=doc["vector"],
                payload=doc.get("payload", {}),
            )
            for doc in documents
        ]

        qdrant_client.upsert(
            collection_name=collection_name,
            points=points,
        )
        return True
    except Exception as e:
        print(f"Error upserting documents: {e}")
        return False
