import cohere
from qdrant_client import QdrantClient

import os
COHERE_API_KEY = os.getenv("COHERE_API_KEY")

# Initialize Cohere client
cohere_client = cohere.Client("COHERE_API_KEY")

QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

# Connect to Qdrant
qdrant = QdrantClient(
    url="https://d781f662-2044-4110-a0aa-9f08549ea800.us-east4-0.gcp.cloud.qdrant.io",
    api_key="QDRANT_API_KEY",
    timeout=60,  # Increase timeout
)

def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding

def retrieve(query):
    embedding = get_embedding(query)
    result = qdrant.query_points(
        collection_name="physical_ai_docs",
        query=embedding,
        limit=5
    )
    return [point.payload["text"] for point in result.points]

# Test
print(retrieve("What data do you have?"))