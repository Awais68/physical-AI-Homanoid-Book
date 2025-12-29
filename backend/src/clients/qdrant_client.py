"""Qdrant vector database client."""
from typing import Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from src.config.settings import settings


def get_qdrant_client() -> Optional[QdrantClient]:
    """Get Qdrant client instance with retry logic.

    Returns:
        Configured QdrantClient instance or None if connection fails.
    """
    import time
    
    max_retries = 3
    retry_delay = 2  # seconds
    
    for attempt in range(max_retries):
        try:
            # Add timeout to prevent hanging
            # Only use API key if URL is HTTPS (cloud deployment)
            if settings.QDRANT_API_KEY and settings.QDRANT_URL.startswith('https://'):
                client = QdrantClient(
                    url=settings.QDRANT_URL,
                    api_key=settings.QDRANT_API_KEY,
                    timeout=10.0,  # 10 second timeout
                )
            else:
                # Local Qdrant doesn't need API key
                client = QdrantClient(
                    url=settings.QDRANT_URL,
                    timeout=10.0,
                )
            
            # Test connection
            client.get_collections()
            print(f"✓ Qdrant connected: {settings.QDRANT_URL}")
            return client
            
        except Exception as e:
            error_msg = str(e)
            if attempt < max_retries - 1:
                print(f"⚠️ Qdrant connection attempt {attempt + 1}/{max_retries} failed: {error_msg}")
                print(f"   Retrying in {retry_delay} seconds...")
                time.sleep(retry_delay)
            else:
                print(f"❌ Qdrant connection failed after {max_retries} attempts: {error_msg}")
                print(f"   URL: {settings.QDRANT_URL}")
                print(f"   API Key present: {bool(settings.QDRANT_API_KEY)}")
                print("   System will run without document search")
    
    return None


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


def get_collection_info(collection_name: str) -> Optional[dict]:
    """Get information about a collection.
    
    Args:
        collection_name: Name of the collection.
        
    Returns:
        Dictionary with collection info or None if error.
    """
    try:
        collection_info = qdrant_client.get_collection(collection_name)
        return {
            'points_count': collection_info.points_count,
            'status': collection_info.status,
        }
    except Exception as e:
        print(f"Error getting collection info: {e}")
        return None


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
    if not qdrant_client:
        # Only print warning once per session
        if not hasattr(search_similar, '_warned'):
            print("⚠️ Qdrant not available - document search disabled")
            search_similar._warned = True
        return []
    
    try:
        results = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_vector,
            limit=top_k,
            score_threshold=score_threshold,
            with_payload=True,
        )
        
        return [
            {
                "id": str(result.id),
                "score": result.score,
                "payload": result.payload,
            }
            for result in results.points
        ]
    except Exception as e:
        error_msg = str(e)
        # Only log connection errors once to avoid spam
        if "Connection refused" in error_msg or "timeout" in error_msg.lower():
            if not hasattr(search_similar, '_connection_error_logged'):
                print(f"❌ Qdrant search error: {error_msg}")
                print("   (This error will not be shown again this session)")
                search_similar._connection_error_logged = True
        else:
            print(f"Search error: {error_msg}")
        return []


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
