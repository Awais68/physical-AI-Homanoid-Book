"""Check if Qdrant collection exists and has documents, ingest if needed."""
import os
import sys
from pathlib import Path

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent / "backend"))

from dotenv import load_dotenv
load_dotenv()

from qdrant_client import QdrantClient
from qdrant_client.http import models

QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION", "rag_chatbot")

print(f"📍 Connecting to Qdrant: {QDRANT_URL}")

try:
    client = QdrantClient(url=QDRANT_URL, timeout=10.0)
    collections = client.get_collections()
    collection_names = [c.name for c in collections.collections]
    
    if COLLECTION_NAME in collection_names:
        info = client.get_collection(COLLECTION_NAME)
        print(f"✅ Collection '{COLLECTION_NAME}' exists with {info.points_count} documents")
        
        if info.points_count > 0:
            print("✅ Documents already ingested, skipping ingestion")
            sys.exit(0)
    
    print(f"📥 Collection empty or doesn't exist, running ingestion...")
    
    # Create collection if doesn't exist
    if COLLECTION_NAME not in collection_names:
        print(f"Creating collection '{COLLECTION_NAME}'...")
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=1024,  # Cohere embed-english-v3.0
                distance=models.Distance.COSINE,
            ),
        )
    
    # Run ingestion
    print("Running document ingestion...")
    os.system("cd backend && python3 ingest_docs.py")
    
    print("✅ Document ingestion completed")

except Exception as e:
    print(f"❌ Error: {e}")
    sys.exit(1)
